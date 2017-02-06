/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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
 * $Id: ieee80211_node.h 2607 2007-07-25 15:20:59Z mrenzmann $
 */
#ifndef _NET80211_IEEE80211_NODE_H_
#define _NET80211_IEEE80211_NODE_H_

#include "net80211/ieee80211_ioctl.h"		/* for ieee80211_nodestats */
#include "net80211/ieee80211_proto.h"		/* for proto macros on node */
#include "net80211/ieee80211_var.h"
#include "qtn/muc_phy_stats.h"
#include "qtn/shared_defs.h"
#include "qtn/qdrv_sch_data.h"
#ifdef TOPAZ_PLATFORM
#include "qtn/topaz_shared_params.h"
#endif

/* #define IEEE80211_DEBUG_REFCNT */
#define IEEE80211_NODEREF_INCR	1
#define IEEE80211_NODEREF_DECR	0

#ifdef IEEE80211_DEBUG_REFCNT
#define REFDEBUG_ENTRY_MAX	150
struct node_refdebug_info_entry {
	const char *fname;
	uint32_t line;
	uint32_t count;
};

struct node_refdebug_info {
	int entry_count;
	uint32_t inc_count;
	uint32_t dec_count;
	struct node_refdebug_info_entry entry[REFDEBUG_ENTRY_MAX];
};
#endif
void ieee80211_node_dbgref_history_dump(void);

/*
 * Each ieee80211com instance has a single timer that fires once a
 * second.  This is used to initiate various work depending on the
 * state of the instance: scanning (passive or active), ``transition''
 * (waiting for a response to a management frame when operating
 * as a station), and node inactivity processing (when operating
 * as an AP).  For inactivity processing each node has a timeout
 * set in it's ni_inact field that is decremented on each timeout
 * and the node is reclaimed when the counter goes to zero.  We
 * use different inactivity timeout values depending on whether
 * the node is associated and authorized (either by 802.1x or
 * open/shared key authentication) or associated but yet to be
 * authorized.  The latter timeout is shorter to more aggressively
 * reclaim nodes that leave part way through the 802.1x exchange.
 */
#define	IEEE80211_INACT_WAIT	5		/* inactivity interval (secs) */
#define	IEEE80211_INACT_INIT	(30/IEEE80211_INACT_WAIT)	/* initial */
#define	IEEE80211_INACT_AUTH	(30/IEEE80211_INACT_WAIT)	/* associated but not authorized */
#define	IEEE80211_INACT_RUN	(30 /IEEE80211_INACT_WAIT)	/* authorized */
#define	IEEE80211_INACT_PROBE	(10/IEEE80211_INACT_WAIT)	/* probe */
#define	IEEE80211_INACT_SCAN	(300/IEEE80211_INACT_WAIT)	/* scanned */
#define IEEE80211_INACT_SEND_PKT_THRSH	3			/* The threshold that starts to send detection packet */

#define	IEEE80211_TRANS_WAIT	2		/* mgt frame tx timer (secs) + random32()%HZ */

#define	IEEE80211_NODE_HASHSIZE	32
/* simple hash is enough for variation of macaddr */
#define	IEEE80211_NODE_HASH(addr)	\
	(((const u_int8_t *)(addr))[IEEE80211_ADDR_LEN - 1] % \
		IEEE80211_NODE_HASHSIZE)

#define IEEE80211_NODE_TRAINING_NORMAL_MODE	1
#define IEEE80211_NODE_TRAINING_TDLS_MODE	2

struct ieee80211_rsnparms {
	u_int8_t rsn_mcastcipher;	/* mcast/group cipher */
	u_int8_t rsn_mcastkeylen;	/* mcast key length */
	u_int8_t rsn_ucastcipherset;	/* unicast cipher set */
	u_int8_t rsn_ucastcipher;	/* selected unicast cipher */
	u_int8_t rsn_ucastkeylen;	/* unicast key length */
	u_int8_t rsn_keymgmtset;	/* key management algorithms */
	u_int8_t rsn_keymgmt;		/* selected key mgmt algo */
	u_int16_t rsn_caps;		/* capabilities */
};

#ifdef CONFIG_QVSP
struct ieee80211_ba_throt {
	unsigned long		last_setup_jiffies;
	uint32_t		unthroted_win_size;
	uint32_t		throt_win_size;
	uint32_t		throt_intv;		/* ms */
	uint32_t		throt_dur;		/* ms */
};
#endif

struct ieee80211_ba_tid {
	enum ieee80211_ba_state	state;
	unsigned long		state_deadline;
	seqlock_t		state_lock;
	enum ieee80211_ba_type	type;
	u_int16_t		timeout;
	u_int16_t		buff_size;
	u_int16_t		seq;
	u_int8_t		frag;
	u_int8_t		dlg_out;
	u_int8_t		dlg_in;
	u_int16_t		flags;
	u_int8_t                dtls_present; /* detected DTLS session */
	unsigned long           dtls_jiffies; /* DTLS detect timestamp */
#ifdef CONFIG_QVSP
	struct ieee80211_ba_throt ba_throt;
#endif
};
#if defined(CONFIG_QTN_80211K_SUPPORT)
struct ieee80211_dotk_waitq_state {
	int status;
	int pending;
};

struct ieee80211_dotk_meas_state {
	/* Radio measurement - STA Statistics */
	struct ieee80211_dotk_waitq_state meas_state_sta;
	/* ... */
};
#endif

struct meas_info {
	u_int8_t ni_meas_rep_mode;	/* report mode */
	u_int32_t ni_meas_rep_time;	/* last receive time */
	wait_queue_head_t meas_waitq;	/* wait queue for syncing with user space */
	u_int8_t pending;		/* flag indicating pending request from user space */
	u_int8_t reason;		/* 0 = success; 1 = timeout; 2 = node leave */

	/* measurement result */
	union {
		uint8_t basic;
		uint8_t cca;
		uint8_t rpi[8];
		uint8_t chan_load;
		struct {
			uint8_t antenna_id;
			uint8_t anpi;
			uint8_t ipi[11];
		} noise_his;
		struct {
			uint8_t reported_frame_info;
			uint8_t rcpi;
			uint8_t rsni;
			uint8_t bssid[IEEE80211_ADDR_LEN];
			uint8_t antenna_id;
			uint32_t parent_tsf;
		} beacon;
		struct {
			uint32_t sub_ele_flag;
			uint8_t ta[IEEE80211_ADDR_LEN];
			uint8_t bssid[IEEE80211_ADDR_LEN];
			uint8_t phy_type;
			uint8_t avg_rcpi;
			uint8_t last_rsni;
			uint8_t last_rcpi;
			uint8_t antenna_id;
			uint16_t frame_count;
		} frame_count;
		struct {
			uint8_t reason;
			uint32_t tran_msdu_cnt;
			uint32_t msdu_discard_cnt;
			uint32_t msdu_fail_cnt;
			uint32_t msdu_mul_retry_cnt;
			uint32_t qos_lost_cnt;
			uint32_t avg_queue_delay;
			uint32_t avg_tran_delay;
			uint8_t bin0_range;
			uint32_t bins[6];
		} tran_stream_cat;
		struct {
			uint8_t reason;
			uint32_t mul_rec_msdu_cnt;
			uint16_t first_seq_num;
			uint16_t last_seq_num;
			uint16_t mul_rate;
		} multicast_diag;
	} rep;
};

struct ieee80211_tpc_info {
	struct  {
		int8_t	min_txpow;
		int8_t	max_txpow;
	} tpc_sta_cap;

	struct {
		int8_t	node_txpow;
		int8_t	node_link_margin;
	} tpc_report;

	struct {
		u_int8_t reason;
		u_int8_t tpc_pending;
		wait_queue_head_t tpc_waitq;
	} tpc_wait_info;
};

struct ieee80211_rm_link_measure_report {
	struct {
		int8_t tx_power;
		int8_t link_margin;
	} tpc_report;
	uint8_t recv_antenna_id;
	uint8_t tran_antenna_id;
	uint8_t rcpi;
	uint8_t rsni;
};

struct ieee80211_neighbor_report_item {
	uint8_t bssid[IEEE80211_ADDR_LEN];
	uint32_t bssid_info;
	uint8_t operating_class;
	uint8_t channel;
	uint8_t phy_type;
};

#define IEEE80211_RM_NEIGHBOR_REPORT_ITEM_MAX	32

struct ieee80211_rm_neighbor_report {
	uint8_t report_count;
	struct ieee80211_neighbor_report_item *item_table[IEEE80211_RM_NEIGHBOR_REPORT_ITEM_MAX];
};

struct ieee80211_node_table;
struct ieee80211com;
struct ieee80211vap;

/**
 * Local qdrv node statistics for smoothing
 */
struct qtn_node_phy_stats {
	uint32_t avg_tx_phy_rate;
	uint32_t avg_rx_phy_rate;
};

/*
 * Node specific information.  Note that drivers are expected
 * to derive from this structure to add device-specific per-node
 * state.  This is done by overriding the ic_node_* methods in
 * the ieee80211com structure.
 */
struct ieee80211_node {
	struct ieee80211vap *ni_vap;
	struct ieee80211com *ni_ic;
	struct ieee80211_node_table *ni_table;
	TAILQ_ENTRY(ieee80211_node) ni_list;
	TAILQ_ENTRY(ieee80211_node) ni_addba_list;
	LIST_ENTRY(ieee80211_node) ni_hash;
	atomic_t ni_refcnt;
	u_int ni_scangen;			/* gen# for timeout scan */
	u_int8_t ni_authmode;			/* authentication algorithm */
	u_int16_t ni_flags;			/* special-purpose state */
	u_int8_t ni_ath_flags;			/* Atheros feature flags */
	/* NB: These must have the same values as IEEE80211_ATHC_* */
#define IEEE80211_NODE_TURBOP	0x0001		/* Turbo prime enable */
#define IEEE80211_NODE_COMP	0x0002		/* Compresssion enable */
#define IEEE80211_NODE_FF	0x0004          /* Fast Frame capable */
#define IEEE80211_NODE_XR	0x0008		/* Atheros WME enable */
#define IEEE80211_NODE_AR	0x0010		/* AR capable */
#define IEEE80211_NODE_BOOST	0x0080
#define IEEE80211_NODE_PS_CHANGED	0x0200	/* PS state change */
	u_int16_t ni_ath_defkeyindex;		/* Atheros def key index */
#define IEEE80211_INVAL_DEFKEY	0x7FFF
	u_int8_t ni_brcm_flags;			/* Broadcom feature flags */
	u_int8_t ni_bbf_disallowed;		/* flag to disallow BBF */
	u_int8_t ni_std_bf_disallowed;		/* flag to disallow standard BF */
	uint16_t ni_associd;			/* assoc response */
	uint16_t ni_node_idx;			/* local node index */
	u_int16_t ni_txpower;			/* current transmit power (in 0.5 dBm) */
	u_int16_t ni_vlan;			/* vlan tag */
	u_int32_t *ni_challenge;			/* shared-key challenge */
	u_int8_t *ni_wpa_ie;			/* captured WPA ie */
	u_int8_t *ni_rsn_ie;			/* captured RSN ie */
	u_int8_t *ni_wme_ie;			/* captured WME ie */
	u_int8_t *ni_wsc_ie;			/* captured WSC ie */
	u_int8_t *ni_ath_ie;			/* captured Atheros ie */
	u_int8_t *ni_qtn_assoc_ie;		/* captured Quantenna ie from assoc */
	u_int8_t *ni_qtn_pairing_ie;		/* captured QTN Pairing IE from assoc */
	u_int8_t *ni_qtn_brmacs;		/* captured QTN brmacs IE for TDLS */
	u_int8_t ni_ext_role;			/* extender role of the node */
	u_int8_t *ni_ext_bssid_ie;			/* captured extender bssid IE */
	u_int8_t ni_vendor;
	u_int16_t ni_txseqs[17];		/* tx seq per-tid */
	u_int16_t ni_rxseqs[17];		/* rx seq previous per-tid*/
	unsigned long ni_rxfragstamp;		/* time stamp of last rx frag */
	struct sk_buff *ni_rxfrag;		/* rx frag reassembly */
	struct ieee80211_rsnparms ni_rsn;	/* RSN/WPA parameters */
	struct ieee80211_key ni_ucastkey;	/* unicast key */
	int ni_rxkeyoff;			/* Receive key offset */
	struct ieee80211_htcap ni_htcap;	/* parsed HT capabilities */
	struct ieee80211_ie_htcap ni_ie_htcap;	/* received htcap IE */
	struct ieee80211_htinfo ni_htinfo;	/* parsed HT information */
	struct ieee80211_ie_htinfo ni_ie_htinfo;/* received htinfo IE */
	uint32_t ni_rate_train;
	uint32_t ni_rate_train_hash;
	uint32_t ni_ver_sw;
	uint16_t ni_ver_hw;
	uint16_t ni_ver_platform_id;
	uint32_t ni_ver_timestamp;
	uint32_t ni_ver_flags;
	uint8_t ni_vsp_version;
#ifdef CONFIG_QVSP
	uint32_t ni_vsp_ba_throt_bm;            /* VSP BA throtting bitmap */
#endif

	struct ieee80211_vhtcap		ni_vhtcap;
	struct ieee80211_vhtop		ni_vhtop;
	struct ieee80211_vht_mu_grp	ni_mu_grp;
	struct ieee80211_ie_vhtcap	ni_ie_vhtcap;
	struct ieee80211_ie_vhtop	ni_ie_vhtop;

	struct qtn_node_phy_stats	ni_local_stats;
	struct qtn_node_shared_stats	*ni_shared_stats;
	struct qtn_node_shared_stats	*ni_shared_stats_phys;
#ifdef CONFIG_QVSP
#if TOPAZ_QTM
	/* used to calculate traffic per check interval */
	struct qtn_vsp_per_node_stats	ni_prev_vsp_stats;
#endif
#endif

	struct qdrv_sch_node_data	ni_tx_sch;

	struct ieee80211_action	ni_action;
#if defined(CONFIG_QTN_80211K_SUPPORT)
	/* action token */
	u_int8_t ni_action_token;
#endif
	u_int8_t ni_implicit_ba; /* Implicit block ack flags as passed in by the client */
	u_int8_t ni_implicit_ba_valid;
	u_int16_t ni_implicit_ba_size;

#ifndef WME_NUM_TID
#define WME_NUM_TID 16
#endif
	/* block ack */
	struct ieee80211_ba_tid ni_ba_rx[WME_NUM_TID];
	struct ieee80211_ba_tid ni_ba_tx[WME_NUM_TID];
	struct work_struct ni_tx_addba_task;
#define IEEE80211_WDS_LINK_MAINTAIN_BA_TID 0
	struct timer_list ni_training_timer;	/* timer for running rate training */
#define NI_TRAINING_INIT	0x0
#define NI_TRAINING_RUNNING	0x1
#define NI_TRAINING_END		0x2
	int ni_training_flag;

	int ni_training_count;
	unsigned long ni_training_start;
	int ni_wds_ba_attempts;

	spinlock_t ni_lock;

	/* 11n information */
	enum ieee80211_cwm_width ni_chwidth;	/* recommended tx channel width */
	u_int8_t ni_newchwidth;	/* channel width changed */

	/* hardware */
	u_int32_t ni_rstamp;			/* recv timestamp */
	u_int32_t ni_last_rx;			/* recv jiffies */
	int32_t ni_rssi;			/* recv ssi */
	int32_t ni_smthd_rssi;			/* Smoothed RSSI */
	int32_t ni_atten_smoothed;		/* conservative index for rate ratio in scs channel ranking */
	uint16_t ni_recent_cca_intf;
	uint32_t ni_recent_cca_intf_jiffies;
	uint16_t ni_recent_cca_intf_smthed;
	uint16_t ni_others_rx_time_smthed;
	uint16_t ni_others_tx_time_smthed;
	uint16_t ni_others_time;		/* Total time (Rx + Tx) */
	uint32_t ni_cca_intf_smth_jiffies;
	uint32_t ni_others_time_smth_jiffies;
	int32_t ni_recent_rxglitch_trig_consecut;    /* rxglitch consecutively exceed threshold */
	uint32_t ni_recent_rxglitch;
	uint32_t ni_recent_sp_fail;
	uint32_t ni_recent_lp_fail;
	uint16_t ni_recent_others_time;
	uint16_t ni_recent_others_time_smth;
	uint16_t ni_recent_tdls_tx_time;
	uint16_t ni_recent_tdls_rx_time;
	uint16_t ni_tdls_tx_time_smthed;
	uint16_t ni_tdls_rx_time_smthed;
	uint32_t ni_tdls_time_smth_jiffies;
	int ni_hw_noise;

	/* header */
	u_int8_t ni_macaddr[IEEE80211_ADDR_LEN];
	u_int8_t ni_bssid[IEEE80211_ADDR_LEN];

	/* beacon, probe response */
	union {
		u_int8_t data[8];
		__le64 tsf;
	} ni_tstamp;				/* from last rcv'd beacon */

	u_int16_t ni_intval;			/* beacon interval */
	u_int16_t ni_intval_old;		/* beacon interval before first change */
	u_int16_t ni_intval_cnt;		/* count of ni_intval != ni_intval_old */
	unsigned long ni_intval_end;		/* end of transition interval jiffies */
	uint64_t ni_tbtt;			/* TBTT of AP node */
	uint64_t ni_dtim_tbtt;			/* dtim TBTT of AP node */

	u_int16_t ni_capinfo;			/* capabilities */
	u_int8_t ni_esslen;
	u_int8_t ni_essid[IEEE80211_NWID_LEN];
	struct ieee80211_rateset ni_rates;	/* negotiated rate set */
	struct ieee80211_ht_rateset ni_htrates;	/* negotiated ht rate set */
	struct ieee80211_channel *ni_chan;
	uint8_t ni_supp_chans[IEEE80211_CHAN_BYTES];	/* supported channels bitmap */
	uint32_t ni_chan_num;				/* supported channels number */
	u_int16_t ni_fhdwell;			/* FH only */
	u_int8_t ni_fhindex;			/* FH only */
	u_int8_t ni_erp;				/* ERP from beacon/probe resp */
	u_int16_t ni_timoff;			/* byte offset to TIM ie */
#if defined(CONFIG_QTN_80211K_SUPPORT)
	/* action frame, radio measurement category, sta statistics */
	u_int8_t ni_rm_sta_seq;
#endif
	/* others */
	struct sk_buff_head ni_savedq;		/* packets queued for pspoll */
	short ni_inact;				/* inactivity mark count */
	short ni_inact_reload;			/* inactivity reload value */
	struct work_struct ni_inact_work;	/* inactivity workqueue */
	int ni_txrate;				/* index to ni_rates[] */
	struct ieee80211_nodestats ni_stats;	/* per-node statistics */
	struct ieee80211vap *ni_prev_vap;	/* previously associated vap */
	u_int8_t ni_uapsd;			/* U-APSD per-node flags matching WMM STA Qos Info field */
	u_int16_t ni_uapsd_trigseq[WME_NUM_AC];	/* trigger suppression on retry */
	__le16 ni_pschangeseq;

	u_int16_t ni_linkqual;			/* link quality - provisional - currently TX PHY rate*/
	u_int16_t ni_rx_phy_rate;
	unsigned long ni_blacklist_timeout;	/* MAC filtering timeout in jiffies - 0 if not blacklisted*/
	int ni_lncb_4addr;			/* Support 4-addr encap of LNCB packets (multicast) */
	TAILQ_ENTRY(ieee80211_node) ni_lncb_lst;/* List of STAs supporting 4 address LNCB packets */
	int is_in_lncb_lst;
	TAILQ_ENTRY(ieee80211_node) ni_bridge_lst; /* List of bridge STAs */
	int is_in_bridge_lst;
	u_int64_t ni_start_time_assoc;		/* start time of association (jiffies) */
	int ni_snr;
	int ni_in_auth_state;
	int ni_max_queue;
#if defined(CONFIG_QTN_80211K_SUPPORT)
	/* 802.11k related */
	struct ieee80211_ie_qtn_rm_sta_all ni_qtn_rm_sta_all;
	struct ieee80211_ie_rm_sta_grp221 ni_rm_sta_grp221;
	wait_queue_head_t ni_dotk_waitq;
	struct ieee80211_dotk_meas_state ni_dotk_meas_state;
	unsigned long ni_last_update[RM_QTN_MAX+1];
#endif
	/* TPC related */
	struct ieee80211_tpc_info ni_tpc_info;

	/* Measurement related */
	struct meas_info	ni_meas_info;	/* mainly for measurement report */

	struct ieee80211_rm_link_measure_report ni_lm;
	struct ieee80211_rm_neighbor_report ni_neighbor;

	u_int32_t ni_ip_addr;
#ifdef CONFIG_IPV6
	struct in6_addr ipv6_llocal;		/* IPv6 link local address */
#endif
#define IEEE80211_IP_ADDR_FILTER_NONE 0
#define IEEE80211_IP_ADDR_FILTER_DHCP_RSP 1
#define IEEE80211_IP_ADDR_FILTER_ARP_RSP 2
	uint8_t		ni_ip_addr_filter;
	uint32_t	ni_qtn_flags;
	uint32_t	last_tx_phy_rate;	/* For TDLS path selection */
	uint32_t	rx_pkts;
	uint32_t	tx_acks;
	uint32_t	ni_sa_query_tid;
	unsigned long	ni_sa_query_timeout;

	unsigned long tdls_last_seen;		/* jiffies TDLS peer is last seen */
	uint32_t tdls_last_path_sel;		/* Last path selection result */
	int32_t tdls_path_sel_num;		/* Continuous same path selection number*/
	uint16_t tdls_peer_associd;		/* tdls peer node's AID, allocated by AP, unique value at BSS */
	enum ni_tdls_status tdls_status;	/* TDLS status */
	uint8_t tdls_send_cs_req;		/* TDLS channel switch request has sent out */
	uint8_t tdls_initiator;			/* TDLS peer as initiator */
	uint8_t tdls_no_send_cs_resp;		/* TDLS channel switch response not send yet*/
	unsigned long tdls_setup_start;		/* jiffies TDLS setup start */

/* These values must be kept in sync with ieee80211_node_type_str */
#define IEEE80211_NODE_TYPE_NONE	0
#define IEEE80211_NODE_TYPE_VAP		1
#define IEEE80211_NODE_TYPE_STA		2
#define IEEE80211_NODE_TYPE_WDS		3
#define IEEE80211_NODE_TYPE_TDLS	4
	uint32_t ni_node_type;

	uint8_t ni_used_auth_algo;
	int32_t	rssi_avg_dbm;
	uint8_t ni_wifi_mode;

#ifdef IEEE80211_DEBUG_REFCNT
	struct node_refdebug_info *ni_refdebug_info_p;
#endif
};
MALLOC_DECLARE(M_80211_NODE);

/*
 * Association IDs are managed with a bit vector.
 */
#define	IEEE80211_NODE_AID(ni)			IEEE80211_AID((ni)->ni_associd)
#define IEEE80211_NODE_IS_VHT(_ni)		(((_ni)->ni_flags & IEEE80211_NODE_VHT) != 0)
#define IEEE80211_NODE_IS_HT(_ni)		(((_ni)->ni_flags & IEEE80211_NODE_HT) != 0)

#define	IEEE80211_AID_SET(_ic, _b) \
	((_ic)->ic_aid_bitmap[IEEE80211_AID(_b) / 32] |= \
		(1 << (IEEE80211_AID(_b) % 32)))
#define	IEEE80211_AID_CLR(_ic, _b) \
	((_ic)->ic_aid_bitmap[IEEE80211_AID(_b) / 32] &= \
		~(1 << (IEEE80211_AID(_b) % 32)))
#define	IEEE80211_AID_ISSET(_ic, _b) \
	((_ic)->ic_aid_bitmap[IEEE80211_AID(_b) / 32] & (1 << (IEEE80211_AID(_b) % 32)))

#define	IEEE80211_NODE_STAT(ni,stat)		(ni->ni_stats.ns_##stat++)
#define	IEEE80211_NODE_STAT_ADD(ni,stat,v)	(ni->ni_stats.ns_##stat += v)
#define	IEEE80211_NODE_STAT_SET(ni,stat,v)	(ni->ni_stats.ns_##stat = v)

#define WMM_UAPSD_NODE_IS_PWR_MGT(_ni) ( \
		((_ni)->ni_flags & IEEE80211_NODE_PWR_MGT) && ((_ni)->ni_uapsd))
#define WME_UAPSD_AC_CAN_TRIGGER(_ac, _ni) ( \
		(WMM_UAPSD_NODE_IS_PWR_MGT(_ni)) && WME_UAPSD_AC_ENABLED((_ac), (_ni)->ni_uapsd) )

#define	IEEE80211_NODE_IS_TDLS_ACTIVE(_ni)	((_ni)->tdls_status ==\
			IEEE80211_TDLS_NODE_STATUS_ACTIVE)
#define	IEEE80211_NODE_IS_NONE_TDLS(_ni)	((_ni)->tdls_status ==\
			IEEE80211_TDLS_NODE_STATUS_NONE)
#define	IEEE80211_NODE_IS_TDLS_STARTING(_ni)	((_ni)->tdls_status ==\
			IEEE80211_TDLS_NODE_STATUS_STARTING)
#define	IEEE80211_NODE_IS_TDLS_INACTIVE(_ni)	((_ni)->tdls_status ==\
			IEEE80211_TDLS_NODE_STATUS_INACTIVE)
#define	IEEE80211_NODE_IS_TDLS_IDLE(_ni)	((_ni)->tdls_status ==\
			IEEE80211_TDLS_NODE_STATUS_IDLE)

#define WME_UAPSD_NODE_MAXQDEPTH	8
#define IEEE80211_NODE_UAPSD_USETIM(_ni) (((_ni)->ni_uapsd & 0xF) == 0xF )
#define WME_UAPSD_NODE_INVALIDSEQ	0xffff
#define WME_UAPSD_NODE_TRIGSEQINIT(_ni)	(memset(&(_ni)->ni_uapsd_trigseq[0], 0xff, sizeof((_ni)->ni_uapsd_trigseq)))

#define IEEE80211_NODE_MU_DEL_GRP(_ni, _grp)		IEEE80211_MU_DEL_GRP(_ni->ni_mu_grp, _grp)
#define IEEE80211_NODE_MU_ADD_GRP(_ni, _grp, _pos)	IEEE80211_MU_ADD_GRP(_ni->ni_mu_grp, _grp, _pos)
#define IEEE80211_NODE_MU_IS_GRP_MBR(_ni, _grp)		IEEE80211_MU_IS_GRP_MBR(_ni->ni_mu_grp, _grp)
#define IEEE80211_NODE_MU_GRP_POS(_ni, _grp)		IEEE80211_MU_GRP_POS(_ni->ni_mu_grp, _grp)

static __inline int ieee80211_node_is_running(const struct ieee80211_node *ni)
{
	return (IEEE80211_NODE_IS_NONE_TDLS(ni) || IEEE80211_NODE_IS_TDLS_ACTIVE(ni));
}

static __inline int ieee80211_node_is_qtn(const struct ieee80211_node *ni)
{
	return ni->ni_qtn_assoc_ie != NULL;
}

void ieee80211_node_attach(struct ieee80211com *);
void ieee80211_node_detach(struct ieee80211com *);
void ieee80211_node_vattach(struct ieee80211vap *);
void ieee80211_node_latevattach(struct ieee80211vap *);
void ieee80211_node_vdetach(struct ieee80211vap *);

static __inline int
ieee80211_node_is_authorized(const struct ieee80211_node *ni)
{
	return (ni->ni_flags & IEEE80211_NODE_AUTH);
}

static __inline int
ieee80211_node_power_save_scheme(const struct ieee80211_node *ni)
{
/*Will expand the bitmap to include other power save schemes*/
#define POWER_SAVE_SCHEME_UAPSD		BIT(0)
	uint32_t ret = 0;
	if ((ni->ni_flags & IEEE80211_NODE_UAPSD)) {
		ret |= POWER_SAVE_SCHEME_UAPSD;
	}
	return ret;
}

void _ieee80211_node_authorize(struct ieee80211_node *);
void ieee80211_node_authorize(struct ieee80211_node *);
void ieee80211_node_unauthorize(struct ieee80211_node *);
void ieee80211_tdls_add_rate_detection(struct ieee80211_node *);
void ieee80211_node_training_start(struct ieee80211_node *ni, int immediate);

void ieee80211_create_bss(struct ieee80211vap *, struct ieee80211_channel *);
void ieee80211_reset_bss(struct ieee80211vap *);
int ieee80211_ibss_merge(struct ieee80211_node *);
struct ieee80211_scan_entry;
int ieee80211_sta_join(struct ieee80211vap *, const struct ieee80211_scan_entry *);
void ieee80211_sta_join1(struct ieee80211vap *, struct ieee80211_node *, int reauth);
void ieee80211_sta_join1_tasklet(IEEE80211_TQUEUE_ARG);
void ieee80211_sta_leave(struct ieee80211_node *);
void ieee80211_sta_fast_rejoin(unsigned long arg);
void ieee80211_send_vht_opmode_action(struct ieee80211vap *vap,
                                       struct ieee80211_node *ni,
                                       uint8_t bw, uint8_t rx_nss);


#define WDS_AGING_TIME		600   /* 10 minutes */
#define WDS_AGING_COUNT		2
#define WDS_AGING_STATIC	0xffff
#define WDS_AGING_TIMER_VAL	(WDS_AGING_TIME / 2)

struct ieee80211_wds_addr {
	LIST_ENTRY(ieee80211_wds_addr) wds_hash;
	u_int8_t		wds_macaddr[IEEE80211_ADDR_LEN];
	struct ieee80211_node	*wds_ni;
	u_int16_t		wds_agingcount;
};

/*
 * Table of ieee80211_node instances.  Each ieee80211com
 * has at least one for holding the scan candidates.
 * When operating as an access point or in ibss mode there
 * is a second table for associated stations or neighbors.
 */
struct ieee80211_node_table {
	struct ieee80211com *nt_ic;		/* back reference */
	ieee80211_node_lock_t nt_nodelock;	/* on node table */
	TAILQ_HEAD(, ieee80211_node) nt_node;	/* information of all nodes */
	ATH_LIST_HEAD(, ieee80211_node) nt_hash[IEEE80211_NODE_HASHSIZE];
	ATH_LIST_HEAD(, ieee80211_wds_addr) nt_wds_hash[IEEE80211_NODE_HASHSIZE];
	const char *nt_name;			/* for debugging */
	ieee80211_scan_lock_t nt_scanlock;	/* on nt_scangen */
	u_int nt_scangen;			/* gen# for timeout scan */
	int nt_inact_init;			/* initial node inact setting */
	struct timer_list nt_wds_aging_timer;	/* timer to age out wds entries */
};

struct ieee80211_node *ieee80211_alloc_node(struct ieee80211_node_table *,
	struct ieee80211vap *, const u_int8_t *, const char*);
struct ieee80211_node *ieee80211_tmp_node(struct ieee80211vap *,
	const u_int8_t *);
struct ieee80211_node *_ieee80211_tmp_node(struct ieee80211vap *,
	const u_int8_t *, const u_int8_t *);
struct ieee80211_node *ieee80211_dup_bss(struct ieee80211vap *,
	const u_int8_t *);

#define ieee80211_check_free_node(_held, _ni) do	\
{							\
	if (_held) {					\
		ieee80211_free_node(_ni);		\
	}						\
} while (0)

#ifdef IEEE80211_DEBUG_REFCNT
void ieee80211_node_dbgref(const struct ieee80211_node *ni, const char *filename,
				const int line, int is_increased);
static __inline void
ieee80211_ref_node_debug(struct ieee80211_node *ni, const char * filename, const int line)
{
	ieee80211_node_incref(ni);
	ieee80211_node_dbgref(ni, filename, line, IEEE80211_NODEREF_INCR);
}

void ieee80211_free_node_debug(struct ieee80211_node *,
			const char *filename,
			int line);
struct ieee80211_node *ieee80211_find_node_debug(struct ieee80211_node_table *,
							const u_int8_t *macaddr,
							const char *filename,
							int line);
struct ieee80211_node *ieee80211_find_node_by_node_idx_debug(struct ieee80211vap *vap,
			uint16_t aid,
			const char *filename,
			int line);
struct ieee80211_node *ieee80211_find_rxnode_debug(struct ieee80211com *ic,
							const struct ieee80211_frame_min *wh,
							const char *filename,
							int line);
struct ieee80211_node *ieee80211_find_txnode_debug(struct ieee80211vap *vap,
			const u_int8_t *macaddr,
			const char *filename,
			int line);

struct ieee80211_node *ieee80211_find_node_by_idx_debug(struct ieee80211com *ic,
						  struct ieee80211vap *vap,
						  uint16_t node_idx,
						  const char *filename,
						  int line);

#define ieee80211_ref_node(ni) \
	ieee80211_ref_node_debug(ni, __FILE__, __LINE__)

#define ieee80211_free_node(ni) \
	ieee80211_free_node_debug(ni, __FILE__, __LINE__)

#define ieee80211_find_node(nt, mac) \
	ieee80211_find_node_debug(nt, mac, __FILE__, __LINE__)

#define ieee80211_find_node_by_node_idx(vap, aid) \
	ieee80211_find_node_by_node_idx_debug(vap, aid, __FILE__, __LINE__)

#define ieee80211_find_node_by_idx(ic, vap, aid) \
	ieee80211_find_node_by_idx_debug(ic, vap, aid, __FILE__, __LINE__)

#define ieee80211_find_rxnode(nt, wh) \
	ieee80211_find_rxnode_debug(nt, wh, __FILE__, __LINE__)

#define ieee80211_find_txnode(nt, mac) \
	ieee80211_find_txnode_debug(nt, mac, __FILE__, __LINE__)

#define ieee80211_find_node_by_ip_addr(vap, ip_addr) \
	ieee80211_find_node_by_ip_addr_debug(vap, ip_addr, __FILE__, __LINE__)

#ifdef CONFIG_IPV6
#define ieee80211_find_node_by_ipv6_addr(vap, ipv6_addr) \
	ieee80211_find_node_by_ipv6_addr_debug(vap, ipv6_addr, __FILE__, __LINE__)
#endif

#else /* IEEE80211_DEBUG_REFCNT */

#define ieee80211_node_dbgref(ni, filename, line, is_increased)

static __inline void
ieee80211_ref_node(struct ieee80211_node *ni)
{
	ieee80211_node_incref(ni);
}

void ieee80211_free_node(struct ieee80211_node *);
struct ieee80211_node *ieee80211_find_node(struct ieee80211_node_table *nt,
						const u_int8_t *macaddr);
struct ieee80211_node *ieee80211_find_node_by_idx(struct ieee80211com *ic,
						  struct ieee80211vap *vap,
						  uint16_t node_idx);
struct ieee80211_node *ieee80211_find_node_by_node_idx(struct ieee80211vap *vap, uint16_t node_idx);
struct ieee80211_node *ieee80211_find_rxnode(struct ieee80211com *vap,
						const struct ieee80211_frame_min *macaddr);
struct ieee80211_node *ieee80211_find_txnode(struct ieee80211vap *vap, const u_int8_t *macaddr);
struct ieee80211_node *ieee80211_find_node_by_ip_addr(struct ieee80211vap *vap,
							uint32_t ip_addr);

#ifdef CONFIG_IPV6
struct ieee80211_node *ieee80211_find_node_by_ipv6_addr(struct ieee80211vap *vap,
							struct in6_addr *ipv6_addr);
#endif

#define ieee80211_ref_node_debug(ni, filename, line)	ieee80211_ref_node(ni)

#endif /* IEEE80211_DEBUG_REFCNT */

int ieee80211_add_wds_addr(struct ieee80211_node_table *, struct ieee80211_node *,
	const u_int8_t *, u_int8_t);
void ieee80211_remove_wds_addr(struct ieee80211_node_table *, const u_int8_t *);
void ieee80211_del_wds_node(struct ieee80211_node_table *,
	struct ieee80211_node *);
struct ieee80211_node *ieee80211_find_wds_node(struct ieee80211_node_table *,
	const u_int8_t *);

typedef void ieee80211_iter_func(void *, struct ieee80211_node *);
void ieee80211_iterate_nodes(struct ieee80211_node_table *,
	ieee80211_iter_func *, void *, int ignore_blacklisted);
#ifdef CONFIG_QVSP
void ieee80211_node_vsp_send_action(void *arg, struct ieee80211_node *ni);
#endif
void ieee80211_iterate_dev_nodes(struct net_device *,
	struct ieee80211_node_table *, ieee80211_iter_func *, void *,
	int ignore_blacklisted);

void	ieee80211_dump_node(struct ieee80211_node_table *,
	struct ieee80211_node *);
void	ieee80211_dump_nodes(struct ieee80211_node_table *);

struct ieee80211_node *ieee80211_fakeup_adhoc_node(struct ieee80211vap *,
	const u_int8_t macaddr[]);
struct ieee80211_scanparams;
struct ieee80211_node *ieee80211_add_neighbor(struct ieee80211vap *,
	const struct ieee80211_frame *, const struct ieee80211_scanparams *);
int ieee80211_aid_acquire(struct ieee80211com *ic, struct ieee80211_node *ni);
void ieee80211_node_join(struct ieee80211_node *, int);
void ieee80211_node_leave(struct ieee80211_node *);
void ieee80211_node_set_chan(struct ieee80211com *ic, struct ieee80211_node *ni);
uint8_t ieee80211_getrssi(struct ieee80211com *);
void ieee80211_idx_add(struct ieee80211_node *ni, uint16_t new_idx);
void ieee80211_node_ba_state_clear(struct ieee80211_node *ni);

/*
 * Set up the implicit block ACK for the given node.
 * ni the node to set up the implicit BA agreement with. The bitmap of TIDs is within the
 * ni_implicit_ba field of the ieee80211_node structure.
 */
void ieee80211_node_implicit_ba_setup(struct ieee80211_node *ni);
void ieee80211_node_ba_del(struct ieee80211_node *ni, uint8_t tid, uint8_t is_tx, uint16_t reason);

void ieee80211_node_tx_ba_set_state(struct ieee80211_node *ni, uint8_t tid, enum ieee80211_ba_state state, unsigned delay);

uint16_t ieee80211_find_aid_by_mac_addr(struct ieee80211_node_table *nt,
		const u_int8_t *macaddr);
void ieee80211_disconnect_node(struct ieee80211vap *vap, struct ieee80211_node *ni);
int ieee80211_node_is_intel(struct ieee80211_node *ni);
int ieee80211_node_is_realtek(struct ieee80211_node *ni);
int ieee80211_node_is_no_rxamsdu_no_bf(struct ieee80211_node *ni);
#endif /* _NET80211_IEEE80211_NODE_H_ */
