/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2010-2013 Quantenna Communications, Inc.            **
**                            All Rights Reserved                            **
**                                                                           **
**  File        : ieee80211_tdls.h                                           **
**  Description : Tunnelled Direct-Link Setup                                **
**                                                                           **
**  This module implements portions of the IEEE Std 802.11z specification,   **
** as well as a proprietary discovery mechanism.                             **
**                                                                           **
*******************************************************************************
**                                                                           **
**  Redistribution and use in source and binary forms, with or without       **
**  modification, are permitted provided that the following conditions       **
**  are met:                                                                 **
**  1. Redistributions of source code must retain the above copyright        **
**     notice, this list of conditions and the following disclaimer.         **
**  2. Redistributions in binary form must reproduce the above copyright     **
**     notice, this list of conditions and the following disclaimer in the   **
**     documentation and/or other materials provided with the distribution.  **
**  3. The name of the author may not be used to endorse or promote products **
**     derived from this software without specific prior written permission. **
**                                                                           **
**  Alternatively, this software may be distributed under the terms of the   **
**  GNU General Public License ("GPL") version 2, or (at your option) any    **
**  later version as published by the Free Software Foundation.              **
**                                                                           **
**  In the case this software is distributed under the GPL license,          **
**  you should have received a copy of the GNU General Public License        **
**  along with this software; if not, write to the Free Software             **
**  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA  **
**                                                                           **
**  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR       **
**  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES**
**  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  **
**  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,         **
**  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT **
**  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,**
**  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    **
**  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      **
**  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF **
**  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.        **
**                                                                           **
*******************************************************************************
EH0*/

#ifndef _NET80211_IEEE80211_TDLS_H_
#define _NET80211_IEEE80211_TDLS_H_

#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "net80211/if_llc.h"
#include "net80211/if_ethersubr.h"
#include "net80211/if_media.h"

#include "net80211/ieee80211.h"
#include "net80211/ieee80211_var.h"
#include "net80211/ieee80211_dot11_msg.h"
#include "net80211/ieee80211_linux.h"


struct ieee80211_tdls_params {
	/* Header fields */
	u_int8_t	*sa;
	u_int8_t	*da;
	/* action code */
	u_int8_t	act;

	/* Fixed length parameters */
	u_int16_t	caps;
	u_int16_t	reason;
	u_int16_t	status;
	u_int8_t	diag_token;
	u_int8_t	target_chan;
	u_int8_t	reg_class;

	/* TLVs from 802.11z */
	u_int8_t	*rates;
	u_int8_t	*country;
	u_int8_t	*xrates;
	u_int8_t	*supp_chan;
	u_int8_t	*sec_chan_off;
	u_int8_t	*rsn;
	u_int8_t	*ext_cap;
	u_int8_t	*edca;
	u_int8_t	*qos_cap;
	u_int8_t	*ftie;
	u_int8_t	*tpk_timeout;
	u_int8_t	*sup_reg_class;
	u_int8_t	*htcap;
	u_int8_t	*htinfo;
	u_int8_t	*vhtcap;
	u_int8_t	*vhtop;

	u_int8_t	*bss_2040_coex;
	struct ieee80211_ie_aid			*aid;
	struct ieee80211_tdls_link_id		*link_id;
	struct ieee80211_tdls_wkup_sched	*wkup_sched;
	struct ieee80211_tdls_cs_timing		*cs_timing;
	struct ieee80211_tdls_pti_ctrl		*pti_ctrl;
	struct ieee80211_tdls_pu_buf_stat	*pu_buf_stat;
	struct ieee80211_ie_wbchansw		*wide_bw_cs;
	struct ieee80211_ie_vtxpwren		*vht_tx_pw_env;

	/* Proprietary TLVs */
	u_int8_t	*qtn_info;
	u_int8_t	*qtn_brmacs;
};

#define	DEFAULT_TDLS_TIMEOUT_TIME		30
#define	DEFAULT_TDLS_DISCOVER_INTERVAL		60
#define DEFAULT_TDLS_LIFE_CYCLE			(6 * DEFAULT_TDLS_DISCOVER_INTERVAL)

#define DEFAULT_TDLS_PATH_SEL_MODE		0
#define DEFAULT_TDLS_PATH_SEL_PPS_THRSHLD	20
#define DEFAULT_TDLS_PATH_SEL_RATE_THRSHLD	24
#define DEFAULT_TDLS_RATE_DETECTION_PKT_CNT	16
#define DEFAULT_TDLS_RATE_DETECTION_BURST_CNT	2
#define DEFAULT_TDLS_RATE_DETECTION_WAITING_T	12
#define DEFAULT_TDLS_LINK_WEIGHT		5
#define DEFAULT_TDLS_LINK_DISABLE_SCALE		2
#define DEFAULT_TDLS_LINK_SWITCH_INV		2
#define	DEFAULT_TDLS_PHY_RATE_WEIGHT		6

#define TDLS_INVALID_CHANNEL_NUM		0

#define DEFAULT_TDLS_VERBOSE			1
#define DEFAULT_TDLS_MIN_RSSI			(-900)
#define	DEFAULT_TDLS_CH_SW_NEGO_TIME		10000
#define DEFAULT_TDLS_CH_SW_PROC_TIME		6000
#define DEFAULT_TDLS_CH_SW_MIN_TIME		2000
#define DEFAULT_TDLS_CH_SW_TX_TIME		1500
#define	DEFAULT_TDLS_FIXED_OFF_CHAN		TDLS_INVALID_CHANNEL_NUM

#define	DEFAULT_TDLS_UAPSD_INDICATION_WND	(2)
#define	DEFAULT_TDLS_SETUP_EXPIRE_DURATION	(20)

const char *
ieee80211_tdls_action_name_get(u_int8_t action);

const char *
ieee80211_tdls_status_string_get(uint8_t stats);

int
ieee80211_cfg_tdls_add(struct ieee80211vap *vap, u_int8_t *mac);

int
ieee80211_tdls_cfg_disc_int(struct ieee80211vap *vap, int value);

int
ieee80211_tdls_get_smoothed_rssi(struct ieee80211vap *vap, struct ieee80211_node *ni);

int
ieee80211_tdls_disable_peer_link(struct ieee80211_node *ni);

void
ieee80211_tdls_chan_switch_timeout(unsigned long arg);

int
ieee80211_tdls_send_chan_switch_req(struct ieee80211_node *peer_ni,
		struct ieee80211_tdls_action_data *data);

int
ieee80211_tdls_send_chan_switch_resp(struct ieee80211_node *peer_ni,
		struct ieee80211_tdls_action_data *data);

void
ieee80211_tdls_recv_disc_req(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_disc_resp(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_setup_req(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_setup_resp(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_setup_confirm(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_teardown(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_chan_switch_req(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

void
ieee80211_tdls_recv_chan_switch_resp(struct ieee80211_node *ni, struct sk_buff *skb,
	int rssi, struct ieee80211_tdls_params *tdls);

int
ieee80211_tdls_send_action_frame(struct net_device *ndev,
		struct ieee80211_tdls_action_data *data);

int
ieee80211_tdls_send_event(struct ieee80211_node *peer_ni,
		enum ieee80211_tdls_event event, void *data);

int
ieee80211_tdls_return_to_base_channel(struct ieee80211vap *vap, int ap_disassoc);

int
ieee80211_tdls_del_key(struct ieee80211vap *vap, struct ieee80211_node *ni);

void
ieee80211_tdls_free_peer_ps_info(struct ieee80211vap *vap);

void
ieee80211_tdls_vattach(struct ieee80211vap *vap);

void
ieee80211_tdls_vdetach(struct ieee80211vap *vap);

int
ieee80211_tdls_node_join(struct ieee80211vap *vap, struct ieee80211_node *ni);

int
ieee80211_tdls_node_leave(struct ieee80211vap *vap, struct ieee80211_node *ni);

int
ieee80211_tdls_teardown_all_link(struct ieee80211vap *vap);

int
ieee80211_tdls_free_all_peers(struct ieee80211vap *vap);

int
ieee80211_tdls_free_all_inactive_peers(struct ieee80211vap *vap);

int
ieee80211_tdls_clear_disc_timer(struct ieee80211vap *vap);

int
ieee80211_tdls_start_disc_timer(struct ieee80211vap *vap);

void
ieee80211_tdls_node_expire(unsigned long arg);

int
ieee80211_tdls_start_node_expire_timer(struct ieee80211vap *vap);

int
ieee80211_tdls_init_node_expire_timer(struct ieee80211vap *vap);

int
ieee80211_tdls_clear_node_expire_timer(struct ieee80211vap *vap);

int
ieee80211_tdls_set_link_timeout(struct ieee80211vap *vap,
		struct ieee80211_node *ni);

int
ieee80211_tdls_pend_disassociation(struct ieee80211vap *vap,
	enum ieee80211_state nstate, int arg);

int
ieee80211_tdls_init_chan_switch_timer(struct ieee80211vap *vap);

int
ieee80211_tdls_start_channel_switch(struct ieee80211vap *vap,
		struct ieee80211_node *peer_ni);

void
ieee80211_tdls_update_node_status(struct ieee80211_node *ni,
		enum ni_tdls_status stats);

void
ieee80211_tdls_update_uapsd_indicication_windows(struct ieee80211vap *vap);

int ieee80211_tdls_update_link_timeout(struct ieee80211vap *vap);

void ieee80211_tdls_trigger_rate_detection(unsigned long arg);

#endif
