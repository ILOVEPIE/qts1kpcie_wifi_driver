/**
 * Copyright (c) 2014 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>

#define QEVT_DEFAULT_PORT 3490
#define QEVT_RX_BUF_SIZE 1024
#define QEVT_CON_RETRY_DELAY 5U
#define QEVT_MAX_LOG_SIZE 256

#define QEVT_CLIENT_VERSION "v1.00"
#define QEVT_CONFIG "QEVT_CONFIG"
#define QEVT_CONFIG_RESET QEVT_CONFIG"_RESET"
#define QEVT_VERSION "QEVT_VERSION"
#define QEVT_DEFAULT_CONFIG QEVT_CONFIG" WPACTRL3:-"

char fpath[32] = "/tmp";
const char * logname = "qtn5g.log";

char fpath1[32] = "/tmp";
const char * log1name = "qtn5g.log.1";

int log_size = 5 * 1024;
char * log_buf = NULL;

struct qevt_client_config {
	struct sockaddr_in	dest;
	struct in_addr		ip_addr;
	int			client_socket;
	uint16_t		port;
	int			fd;
	char			rxbuffer[QEVT_RX_BUF_SIZE + 1];
	char			*qevt_config_cmd;
};

static int qevt_write_log(int * fd, void * buf, int len)
{
	off_t file_off;
	int bak_fd, log_bytes;
	int log_fd = *fd;

	if (log_fd > 0)
		close(log_fd);

	log_fd = open(fpath, O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP);

	if (log_fd < 0) {
		perror("fail to open qtn5g.log\n");
		return log_fd;
	}

	file_off = lseek(log_fd, 0, SEEK_END);

	if ((int)file_off + len > log_size) {
		file_off = lseek(log_fd, 0, SEEK_SET);

		log_bytes = read(log_fd, log_buf, log_size);
		if (log_bytes < 0 ) {
			perror("read error in qtn5g.log\n");
			return log_bytes;
		}
		close(log_fd);
		*fd = 0;

		bak_fd = open(fpath1, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP);
		if (bak_fd >= 0) {
			if (log_bytes != write(bak_fd, log_buf, log_bytes)) {
				perror ("fail to write qtn5g.log.1\n");
			}
			close (bak_fd);
		} else
			perror("fail to open qtn5g.log.1\n");

		log_fd = open(fpath, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP);

		if (log_fd < 0) {
			perror("fail to open qtn5g.log after change log\n");
			return log_fd;
		}

		*fd = log_fd;
	}

	log_bytes = write(log_fd, buf, len);

	return log_bytes;

}

static int qevt_send_host(struct qevt_client_config *const cfg, const char *msg)
{
	int sent_bytes = 0;
	int ret;

	do {
		ret = send(cfg->client_socket, msg + sent_bytes, strlen(msg) - sent_bytes, 0);

		if (ret > 0) {
			sent_bytes += ret;
		} else if (errno == EINTR) {
			continue;
		} else {
			break;
		}
	} while (sent_bytes < strlen(msg));

	if (ret <= 0) {
		fprintf(stderr, "%s: failure sending a message to event server\n", __func__);
	}
	return (ret > 0);
}

static int qevt_receive(struct qevt_client_config *const cfg)
{
	int received_bytes;

	do {
		received_bytes = recv(cfg->client_socket, cfg->rxbuffer, sizeof(cfg->rxbuffer) - 1, 0);

		if (received_bytes > 0) {
			cfg->rxbuffer[received_bytes] = '\0';
		} else if (received_bytes == 0) {
			printf("Connection closed\n");
			break;
		} else if (errno != EINTR && errno != ECONNREFUSED) {
			perror("Receive failed");
			break;
		}
	} while ((received_bytes < 0) && (errno == EINTR));

	return received_bytes;
}

static char * qevt_config_cmd(struct qevt_client_config *const cfg)
{
	char *msg;
	char *nl;

	/* first, clear to initial settings */
	if (!qevt_send_host(cfg, QEVT_CONFIG_RESET"\n")) {
		return NULL;
	}
	if (qevt_receive(cfg) <= 0) {
		return NULL;
	}

	/* then send config command */
	if (!qevt_send_host(cfg, cfg->qevt_config_cmd ? cfg->qevt_config_cmd :
							QEVT_DEFAULT_CONFIG"\n")) {
		return NULL;
	}
	if (qevt_receive(cfg) <= 0) {
		return NULL;
	}

	msg = strstr(cfg->rxbuffer, QEVT_CONFIG" ");
	if (msg) {
		msg = strstr(msg, " ");
	}
	if (msg && (nl = strstr(++msg, "\n"))) {
		*nl = 0;
	}
	return msg;
}

static int qevt_check_version(struct qevt_client_config *const cfg, char** report)
{
	char *version;
	char *nl;
	const char *cmd = QEVT_VERSION;

	if (report) {
		*report = "UNKNOWN";
	}
	if (!qevt_send_host(cfg, cmd) || (qevt_receive(cfg) < 0)) {
		return 0;
	}

	version = strstr(cfg->rxbuffer, cmd);
	if (!version) {
		return 0;
	}

	version += sizeof(QEVT_VERSION) - 1;
	version = strstr(version, "v");
	if (!version) {
		return 0;
	}

	nl = strstr(version, "\n");
	if (nl) {
		*nl = 0;
	}

	if (report) {
		*report = version;
	}
	return (strcmp(QEVT_CLIENT_VERSION, version) >= 0);
}

static void qevt_receiving_loop(struct qevt_client_config *const cfg)
{
	int received_bytes;
	char *buffer = cfg->rxbuffer;
    char cmd[512];

	for (;;) {
		received_bytes = qevt_receive(cfg);

		if (received_bytes <= 0) {
			break;
		}
		if (received_bytes != qevt_write_log(&(cfg->fd), (void *)buffer, received_bytes)) {
			fprintf(stderr, "write log fail\n");
		}
		printf("%s", buffer);

		sprintf(cmd, "logger \"%s\"", buffer);
		system(cmd);
	}
}

static int qevt_client_init(struct qevt_client_config *const cfg)
{
	if (cfg->client_socket >= 0)
		close(cfg->client_socket);

	cfg->client_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (cfg->client_socket < 0) {
		perror("Failed to create client socket");
		return -1;
	}

	memset(&cfg->dest, 0, sizeof(cfg->dest));
	cfg->dest.sin_family = AF_INET;
	cfg->dest.sin_port = htons(cfg->port);
	cfg->dest.sin_addr = cfg->ip_addr;
	return 0;
}

static int qevt_connected_to_server(const struct qevt_client_config *const cfg)
{
	fd_set fdset;
	struct timeval timeout = {.tv_sec = QEVT_CON_RETRY_DELAY, .tv_usec = 0};
	int connected = 0;

	FD_ZERO(&fdset);
	FD_SET(cfg->client_socket, &fdset);

	if (select(cfg->client_socket + 1, NULL, &fdset, NULL, &timeout) == 1) {
		int so_error;
		socklen_t so_len = sizeof(so_error);

		getsockopt(cfg->client_socket, SOL_SOCKET, SO_ERROR, &so_error, &so_len);
		if (!so_error) {
			connected = 1;
		}
	}

	return connected;
}

static void qevt_client_connect(struct qevt_client_config *const cfg)
{
	int ret;
	int connected = 0;

	while (!connected) {
		fcntl(cfg->client_socket, F_SETFL, O_NONBLOCK);
		ret = connect(cfg->client_socket, (struct sockaddr *)&cfg->dest,
				sizeof(struct sockaddr));

		if (ret < 0) {
			switch (errno) {
			case EINPROGRESS:
				connected = qevt_connected_to_server(cfg);
				if (connected)
					break;
				if (qevt_client_init(cfg) < 0) {
					fprintf(stderr, "fail to create client\n");
				}
				/* Fall through */
			case ECONNREFUSED:
			case ETIMEDOUT:
				fprintf(stderr,
					"Cannot connect to the server. Trying again in %u secs.\n",
						QEVT_CON_RETRY_DELAY);
				sleep(QEVT_CON_RETRY_DELAY);
				break;
			case EINTR:
				break;
			default:
				perror("Cannot connect");
			}
		}
	}

	fcntl(cfg->client_socket, F_SETFL, fcntl(cfg->client_socket, F_GETFL, 0) & ~O_NONBLOCK);
	printf("Connection established\n");
}

int main(int argc, char *argv[])
{
	static struct qevt_client_config client_cfg;
	int ch;
	int status = EXIT_FAILURE;
	char ip_addr[] = "255.255.255.255";

	memset(&client_cfg ,0 ,sizeof(struct qevt_client_config));
	client_cfg.port = QEVT_DEFAULT_PORT;

	while ((ch = getopt(argc, argv, "h:p:d:k:c:")) != -1) {
		switch (ch) {
		case 'h':
			strncpy(ip_addr, optarg, sizeof(ip_addr));
			ip_addr[sizeof(ip_addr) -1] = '\0';
			if (!inet_aton(optarg, &client_cfg.ip_addr)) {
				fprintf(stderr, "IP address specified is not valid\n");
				goto exit;
			}
			break;
		case 'p':
			client_cfg.port = atoi(optarg);
			break;
		case 'd':
			if ((int)strlen(optarg) > ((int)sizeof(fpath) - (int)strlen(log1name) - 2)) {
				printf("Directory path length should not exceed %d\n",
						((int)sizeof(fpath) - (int)strlen(log1name) -2));
				goto exit;
			}

			strncpy(fpath,optarg, strlen(optarg));
			strncpy(fpath1,optarg,strlen(optarg));
			break;
		case 'k':
			if (atoi(optarg) <= QEVT_MAX_LOG_SIZE) {
				log_size = atoi(optarg) * 1024;
			} else {
				fprintf(stderr, "Log size cannot be larger than %d\n",
					QEVT_MAX_LOG_SIZE);
				goto exit;
			}
			break;
		case 'c':
			/* calculate the space needed (include extra for space and '\n') */
			client_cfg.qevt_config_cmd = malloc(sizeof(QEVT_CONFIG) +
							    strlen(optarg) + 4);

			if (client_cfg.qevt_config_cmd) {
				sprintf(client_cfg.qevt_config_cmd, QEVT_CONFIG" %s\n",
					optarg);
			} else {
				fprintf(stderr, "fail to malloc memory %u bytes\n",
						(unsigned int)(sizeof(QEVT_CONFIG) +
						strlen(optarg) + 4));
				goto exit;
			}
			break;
		default:
			printf("Usage: %s -h <host ip addr> -p <host ip port> "
			       "-d <file directory> -k <file size, unit k> "
			       "-c <qevt config>\n", argv[0]);

			goto exit;
		}
	}

	if (!strncmp("255.255.255.255", ip_addr, strlen(ip_addr))) {
		fprintf(stderr, "please select a valid ip address\n");
		goto exit;
	}

	strncat(fpath, "/",1);
	strncat(fpath,logname,strlen(logname));
	strncat(fpath1, "/",1);
	strncat(fpath1,log1name,strlen(log1name));

	client_cfg.client_socket = -1;

	if (qevt_client_init(&client_cfg) < 0) {
		goto exit;
	}

	log_buf = malloc(log_size);

	if(log_buf == NULL) {
		fprintf(stderr, "fail to malloc memory %d bytes\n", log_size);
		goto exit;
	}
	for (;;) {
		char *report = NULL;
		qevt_client_connect(&client_cfg);

		if (!qevt_check_version(&client_cfg, &report)) {
			fprintf(stderr, "incompatible client version '"QEVT_CLIENT_VERSION
				"'/server version '%s'\n", report);
			goto exit;
		}
		if ((report = qevt_config_cmd(&client_cfg))) {
			printf("Server configuration '%s'\n", report);
		} else {
			fprintf(stderr, "unable to set/get config\n");
			goto exit;
		}
		qevt_receiving_loop(&client_cfg);
		qevt_client_init(&client_cfg);
	}
	/*
	 * This point is only reached if the above loop exits (which it should not currently).
	 * In case clean exit is added in future, exit with success status.
	 */
	status = EXIT_SUCCESS;

exit:
	if (log_buf) {
		free(log_buf);
	}
	if (client_cfg.qevt_config_cmd) {
		free(client_cfg.qevt_config_cmd);
	}
	return status;
}

