/*-
 * Copyright (c) 2013 Quantenna
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
 * $Id: ieeee80211_tpc.h 5000 2013-01-25 10:22:59Z casper $
 */
#ifndef _NET80211_IEEE80211_TPC_H
#define _NET80211_IEEE80211_TPC_H

#include <net80211/ieee80211_var.h>	/* struct ieee80211com */
#include <net80211/ieee80211_node.h>	/* struct ieee80211_node */
#include <linux/timer.h>		/* struct timer_list */
#include <linux/types.h>		/*struct list_head */

#define TPC_INTERVAL_DEFAULT		30	/* default tpc request interval */
#define TPC_INTERVAL_MIN		1	/* minimum tpc interval */

/* #define USE_IEEE80211_DPRINT */
#ifndef USE_IEEE80211_DPRINT
#define TPC_DBG(vap, fmt, arg...)	printk(KERN_INFO "TPC:" fmt, ##arg)
#else
#define TPC_DBG(vap, fmt, arg...)	IEEE80211_DPRINTF(vap, IEEE80211_MSG_DOTH | IEEE80211_MSG_TPC, fmt, ##arg)
#endif

struct ieee80211_tpc_query_info {
	void	*target;
	int	is_run;
	int32_t	query_interval;
	struct timer_list query_timer;
};

struct pwr_info_per_vap
{
	int8_t			max_in_minpwr;
	struct ieee80211vap	*vap;
};

enum ieee80211_measurement_status {
	MEAS_STATUS_IDLE = 0,
	MEAS_STATUS_RUNNING,
	MEAS_STATUS_DISCRAD
};

struct ieee80211_global_measure_info {
	enum ieee80211_measurement_status status;
	struct ieee80211_node	*ni;
	u_int8_t frame_token;
	u_int8_t type;
	union {
		struct {
			u_int8_t channel;
			u_int64_t tsf;
			u_int16_t duration_tu;
		} basic;
		struct {
			u_int8_t channel;
			u_int64_t tsf;
			u_int16_t duration_tu;
		} cca;
		struct {
			u_int8_t channel;
			u_int64_t tsf;
			u_int16_t duration_tu;
		} rpi;
		struct {
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t upper_interval;
			u_int16_t duration_tu;
		} chan_load;
		struct {
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t upper_interval;
			u_int16_t duration_tu;
		} noise_his;
	} param;
	union {
		uint8_t	basic;
		uint8_t cca;
		uint8_t rpi[MEAS_RPI_HISTOGRAM_SIZE];
		uint8_t chan_load;
		struct {
			u_int8_t anpi;
			u_int8_t ipi[11];
		} noise_his;
	} results;
};

int ieee80211_tpc_query_init(struct ieee80211_tpc_query_info *info, struct ieee80211com *ic, int query_interval);
void ieee80211_tpc_query_deinit(struct ieee80211_tpc_query_info *info);
int ieee80211_tpc_query_config_interval(struct ieee80211_tpc_query_info *info, int interval);
int ieee80211_tpc_query_get_interval(struct ieee80211_tpc_query_info *info);
int ieee80211_tpc_query_start(struct ieee80211_tpc_query_info *info);
int ieee80211_tpc_query_stop(struct ieee80211_tpc_query_info *info);
int ieee80211_tpc_query_state(struct ieee80211_tpc_query_info *info);
int8_t ieee80211_update_tx_power(struct ieee80211com *ic, int8_t txpwr);
int ieee80211_parse_local_max_txpwr(struct ieee80211vap *vap, struct ieee80211_scanparams *scan);
void get_max_in_minpwr(void *arg, struct ieee80211_node *ni);
void ieee80211_doth_measurement_init(struct ieee80211com *ic);
void ieee80211_doth_measurement_deinit(struct ieee80211com *ic);
void ieee80211_action_finish_measurement(struct ieee80211com *ic, u_int8_t result);
int ieee80211_action_trigger_measurement(struct ieee80211com *ic);
int ieee80211_action_measurement_report_fail(struct ieee80211_node *ni,
					u_int8_t type,
					u_int8_t report_mode,
					u_int8_t token,
					u_int8_t meas_token);

#endif
