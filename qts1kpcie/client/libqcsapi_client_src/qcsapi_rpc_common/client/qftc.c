/*
 * Copyright (c) 2015 Quantenna Communications, Inc.
 * All rights reserved.
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#ifndef _GNU_SOURCE
#include <libgen.h>
#endif
#include <string.h>
#include "qcsapi_rpc_common/common/rpc_raw.h"

#define QFTC_READ_TIMEOUT	(1000 * 2)
#define QFTC_CONNECT_RET_LIMIT	5
#define QFTC_RECV_RETRY_LIMIT	100

static uint32_t qftc_compose_connect_cmd(struct qftp_connect_pkt * const connect_payload,
						const char *file_path_name)
{
	struct stat file_stat;

	memset(&file_stat, 0, sizeof(file_stat));
	if (!stat(file_path_name, &file_stat) && (file_stat.st_mode & S_IFREG)) {
		connect_payload->sub_type = QFTP_CONNECT_FRAME_TYPE;
		connect_payload->seq = 0;
		connect_payload->image_size = file_stat.st_size;
		strcpy(connect_payload->image_name, basename((char *)file_path_name));

		return (sizeof(struct qftp_connect_pkt) +
				strlen(connect_payload->image_name));
	}

	return 0;
}

int qftc_start(const char *file_path_name, const char *sif_name, const uint8_t *dmac_addr)
{
	struct qftp_raw_ethpkt send_buf __attribute__ ((aligned(4)));
	struct qftp_raw_ethpkt recv_buf __attribute__ ((aligned(4)));
	struct sockaddr_ll dst_addr;
	struct sockaddr_ll lladdr;
	struct qftp_ack_nack_pkt *recv_payload = (struct qftp_ack_nack_pkt *)&recv_buf.payload;
	struct qftp_data_pkt *send_payload;
	socklen_t addrlen = sizeof(lladdr);
	const size_t max_data_len = ETH_FRAME_LEN - QFTP_DATA_PKT_HDR_SIZE;
	ssize_t read_bytes;
	ssize_t bytes_recv = -1;
	ssize_t	sent_bytes;
	uint32_t connect_cmd_hdr_size;
	int if_index;
	int sock_fd;
	int retry_count;
	int op_failed;
	int fd;


	sock_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (sock_fd < 0)
		return -1;

	if (qrpc_set_prot_filter(sock_fd, QFTP_RAW_SOCK_PROT) < 0) {
		close(sock_fd);
		return -1;
	}

	if_index = qrpc_clnt_raw_config_dst(sock_fd, sif_name, &dst_addr, dmac_addr,
						(struct q_raw_ethoui_hdr *)&send_buf,
						QFTP_RAW_SOCK_PROT);

	if (if_index < 0) {
		close(sock_fd);
		return -1;
	}

	/* Trying to connect to server */
	connect_cmd_hdr_size = qftc_compose_connect_cmd((struct qftp_connect_pkt *)
								&send_buf.payload, file_path_name);
	if (!connect_cmd_hdr_size) {
		close(sock_fd);
		return -1;
	}
	connect_cmd_hdr_size += sizeof(struct q_raw_ethoui_hdr);

	retry_count = 0;
	op_failed = 0;
	do {
		do {
			sent_bytes = sendto(sock_fd, &send_buf, connect_cmd_hdr_size, 0,
						(struct sockaddr *)&dst_addr, sizeof(dst_addr));
		} while (sent_bytes < 0 && errno == EINTR);

		if (sent_bytes < 0) {
			op_failed = 1;
			break;
		}

		/* Waiting for ACK */
		if (!qrpc_raw_read_timeout(sock_fd, QFTC_READ_TIMEOUT)) {
			break;
		}
	} while (++retry_count < QFTC_CONNECT_RET_LIMIT);

	if (op_failed || retry_count > QFTC_CONNECT_RET_LIMIT) {
		close(sock_fd);
		return -1;
	}

	/* Reading reply for connect command */
	memset(&lladdr, 0, sizeof(lladdr));
	retry_count = 0;
	do {
		if (!qrpc_raw_read_timeout(sock_fd, QFTC_READ_TIMEOUT)) {
			do {
				bytes_recv = recvfrom(sock_fd, &recv_buf, sizeof(recv_buf), 0,
							(struct sockaddr *)&lladdr, &addrlen);
			} while (bytes_recv < 0 && errno == EINTR);
		} else if (++retry_count > QFTC_RECV_RETRY_LIMIT) {
			break;
		}
	} while ((lladdr.sll_ifindex != if_index) || (lladdr.sll_pkttype != PACKET_HOST));

	if ((bytes_recv != QFTP_ACK_NACK_FRAME_LEN) || (retry_count > QFTC_RECV_RETRY_LIMIT) ||
			(recv_payload->sub_type != QFTP_ACK_FRAME_TYPE)) {
		close(sock_fd);
		return -1;
	}

	/* Start transmitting image file */
	send_payload = (struct qftp_data_pkt *)&send_buf.payload;
	fd = open(file_path_name, O_RDONLY);

	if (fd < 0) {
		printf("Failed to open %s file\n", file_path_name);
		close(sock_fd);
		return -1;
	}

	read_bytes = read(fd, send_payload->data, max_data_len);

	op_failed = 0;
	send_payload->sub_type = QFTP_DATA_FRAME_TYPE;
	while (read_bytes > 0) {
		++send_payload->seq;
		do {
			sent_bytes = sendto(sock_fd, &send_buf,
						QFTP_DATA_PKT_HDR_SIZE + read_bytes, 0,
						(struct sockaddr *)&dst_addr, sizeof(dst_addr));
		} while (sent_bytes < 0 && errno == EINTR);

		if (sent_bytes < 0) {
			op_failed = 1;
			break;
		}
		retry_count = 0;
		do {
			if (!qrpc_raw_read_timeout(sock_fd, QFTC_READ_TIMEOUT)) {
				do {
					bytes_recv = recvfrom(sock_fd, &recv_buf, sizeof(recv_buf),
								MSG_DONTWAIT, (struct sockaddr *)&lladdr,
								&addrlen);
				} while (bytes_recv < 0 && errno == EINTR);
			} else if (++retry_count > QFTC_RECV_RETRY_LIMIT) {
				break;
			}
		} while ((lladdr.sll_ifindex != if_index) || (lladdr.sll_pkttype != PACKET_HOST));

		if (op_failed || (retry_count > QFTC_RECV_RETRY_LIMIT) ||
			(send_payload->seq != recv_payload->seq) ||
			(recv_payload->sub_type != QFTP_ACK_FRAME_TYPE)) {
			op_failed = 1;
			break;
		}

		read_bytes = read(fd, send_payload->data, max_data_len);
	}

	close(fd);
	close(sock_fd);

	if (op_failed || (read_bytes < 0))
		return -1;

	return 0;
}
