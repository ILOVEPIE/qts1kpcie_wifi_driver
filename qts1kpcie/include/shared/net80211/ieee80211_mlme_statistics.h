/*-
 * Copyright (c) 2014 Quantenna
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: ieee80211_mlme_statistics.h 1 2014-01-17 12:00:00Z vsaiapin $
 */


#ifndef IEEE80211MLMESTATISTICS_H
#define IEEE80211MLMESTATISTICS_H

#include <linux/ioctl.h>
#include "net80211/ieee80211.h"

//#define MLME_STATS_DEBUG					1
#define MLME_STATS_DEVFS					1
#define MLME_STATS_PROCFS					1

struct mlme_stats_record {
	unsigned char mac_addr[IEEE80211_ADDR_LEN];

	unsigned int auth;
	unsigned int auth_fails;
	unsigned int assoc;
	unsigned int assoc_fails;
	unsigned int deauth;
	unsigned int diassoc;
};

// Statistics entries
enum {
	MLME_STAT_AUTH = 0,
	MLME_STAT_AUTH_FAILS,
	MLME_STAT_ASSOC,
	MLME_STAT_ASSOC_FAILS,
	MLME_STAT_DEAUTH,
	MLME_STAT_DIASSOC,

	MLME_STAT_MAX,
};

#define MLME_STATS_IOCTL_MAGIC				'q'
#define MLME_STATS_IOC_BASE					0x20
#define MLME_STATS_IOC_GET_MAX_CLIENTS		_IOR(MLME_STATS_IOCTL_MAGIC, MLME_STATS_IOC_BASE, unsigned int)						/* Get the maximum possible number of clients in table */
#define MLME_STATS_IOC_GET_CUR_CLIENTS		_IOR(MLME_STATS_IOCTL_MAGIC, MLME_STATS_IOC_BASE + 1, unsigned int)					/* Get the current number of clients in table */
#define MLME_STATS_IOC_GET_ALL_MACS			_IOR(MLME_STATS_IOCTL_MAGIC, MLME_STATS_IOC_BASE + 2, unsigned char*)				/* Get the list of all macs for the moment */
#define MLME_STATS_IOC_GET_CLIENT_STATS		_IOWR(MLME_STATS_IOCTL_MAGIC, MLME_STATS_IOC_BASE + 3, struct mlme_stats_record)	/* Get stats for specified mac */

/**
 * mlme_stats_update - update stats for the client
 * @mac_addr: mac address of the client
 * @statistics_entry: counter needs to be updated
 * @incrementor: value needs to be added to the specified counter
 *
 * Update existsing record or create new one and
 * move it to the head of LRU list.
 */
extern void mlme_stats_update(unsigned char *mac_addr, unsigned int statistics_entry, unsigned int incrementor);

/**
 * mlme_stats_delayed_update - update stats for the client
 * @mac_addr: mac address of the client
 * @statistics_entry: counter needs to be updated
 * @incrementor: value needs to be added to the specified counter
 *
 * Update existsing record or create new one and
 * move it to the head of LRU list.
 */
extern void mlme_stats_delayed_update(unsigned char *mac_addr, unsigned int statistics_entry, unsigned int incrementor);

/**
 * mlme_stats_init - init statistics facility
 *
 * Init all necessary staff for statistics factory.
 * Need to be called during module init.
 */
extern void mlme_stats_init(void);

/**
 * mlme_stats_exit - clear statitsics factory staff
 *
 * Clears memory and removes proc and dev file.
 * Must be called in module exit routine.
 */
extern void mlme_stats_exit(void);

#endif /* IEEE80211MLMESTATISTICS_H */
