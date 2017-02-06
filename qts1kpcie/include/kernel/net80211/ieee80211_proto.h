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
 * $Id: ieee80211_proto.h 2029 2007-01-30 04:01:29Z proski $
 */
#ifndef _NET80211_IEEE80211_PROTO_H_
#define _NET80211_IEEE80211_PROTO_H_

/* Current Fix. As mutual inclusion of files prevent to define following. Need fix */
#define	IEEE80211_C_11N		0x00000010	/* CAPABILITY: 11n HT available */
/*
 * 802.11 protocol implementation definitions.
 */

enum ieee80211_state {
	IEEE80211_S_INIT	= 0,	/* default state */
	IEEE80211_S_SCAN	= 1,	/* scanning */
	IEEE80211_S_AUTH	= 2,	/* try to authenticate */
	IEEE80211_S_ASSOC	= 3,	/* try to assoc */
	IEEE80211_S_RUN		= 4,	/* associated */
};
#define	IEEE80211_S_MAX		(IEEE80211_S_RUN + 1)

#define	IEEE80211_SEND_MGMT(_ni,_type,_arg) \
	((*(_ni)->ni_ic->ic_send_mgmt)(_ni, _type, _arg))

extern const char *ieee80211_mgt_subtype_name[];
extern const char *ieee80211_ctl_subtype_name[];
extern const char *ieee80211_state_name[IEEE80211_S_MAX];
extern const char *ieee80211_wme_acnames[];
extern const char *ieee80211_phymode_name[];

void ieee80211_proto_attach(struct ieee80211com *);
void ieee80211_proto_detach(struct ieee80211com *);
void ieee80211_proto_vattach(struct ieee80211vap *);
void ieee80211_proto_vdetach(struct ieee80211vap *);

struct ieee80211_node;
struct ieee80211_channel *ieee80211_doth_findchan(struct ieee80211vap *, u_int8_t);
int ieee80211_input(struct ieee80211_node *, struct sk_buff *, int, u_int32_t);
int ieee80211_input_all(struct ieee80211com *, struct sk_buff *, int, u_int32_t);
int ieee80211_setup_rates(struct ieee80211_node *, const u_int8_t *,
	const u_int8_t *, int);
int ieee80211_parse_htcap(struct ieee80211_node *ni, u_int8_t *ie);
int ieee80211_parse_htinfo(struct ieee80211_node *ni, u_int8_t *ie);
int ieee80211_parse_vhtcap(struct ieee80211_node *ni, u_int8_t *ie);
int ieee80211_parse_vhtop(struct ieee80211_node *ni, u_int8_t *ie);
int ieee80211_parse_rates(struct ieee80211_node *ni,
	const u_int8_t *rates, const u_int8_t *xrates);
int ieee80211_parse_supp_chan(struct ieee80211_node *ni, uint8_t *ie);
void ieee80211_saveie(u_int8_t **, const u_int8_t *);
void ieee80211_saveath(struct ieee80211_node *, u_int8_t *);
int ieee80211_input_tdls_qtnie(struct ieee80211_node *ni, struct ieee80211vap *vap,
				struct ieee80211_ie_qtn *qtnie);
#if TOPAZ_RX_ACCELERATE
int ieee80211_tdls_tqe_path_check(struct ieee80211_node *ni, struct sk_buff *skb, int rssi, uint16_t ether_type);
#endif
void ieee80211_recv_mgmt(struct ieee80211_node *, struct sk_buff *,
	int, int, u_int32_t);
void ieee80211_sta_pwrsave(struct ieee80211vap *, int);
void ieee80211_parent_queue_xmit(struct sk_buff *);
int ieee80211_send_nulldata(struct ieee80211_node *);
int ieee80211_send_qosnulldata(struct ieee80211_node *, int);
int ieee80211_send_qosnulldata_ext(struct ieee80211com *ic, uint8_t *mac, int pwr);
void ieee80211_send_csa_frame(struct ieee80211vap *vap, u_int8_t csa_mode,
	u_int8_t csa_chan, u_int8_t csa_count, u_int64_t tsf);
int ieee80211_send_mgmt(struct ieee80211_node *, int, int);
void ieee80211_mgmt_output(struct ieee80211_node *ni, struct sk_buff *skb, int type,
	const u_int8_t da[IEEE80211_ADDR_LEN]);
void ieee80211_tdls_mgmt_output(struct ieee80211_node *ni,
	struct sk_buff *skb, const uint8_t type, const uint8_t subtype,
	const uint8_t *da, const uint8_t *bssid);
void ieee80211_send_pspoll(struct ieee80211_node *ni);
void ieee80211_initiate_scan(struct ieee80211vap *vap);
struct sk_buff * ieee80211_get_qosnulldata(struct ieee80211_node *ni, int ac);
struct sk_buff *
ieee80211_get_probereq(struct ieee80211_node *ni,
	const u_int8_t sa[IEEE80211_ADDR_LEN],
	const u_int8_t da[IEEE80211_ADDR_LEN],
	const u_int8_t bssid[IEEE80211_ADDR_LEN],
	const u_int8_t *ssid, size_t ssidlen,
	const void *optie, size_t optielen);
int ieee80211_send_probereq(struct ieee80211_node *,
	const u_int8_t sa[IEEE80211_ADDR_LEN],
	const u_int8_t da[IEEE80211_ADDR_LEN],
	const u_int8_t bssid[IEEE80211_ADDR_LEN],
	const u_int8_t *, size_t, const void *, size_t);
struct sk_buff *ieee80211_encap(struct ieee80211_node *, struct sk_buff *, int *);
void ieee80211_pwrsave(struct ieee80211_node *, struct sk_buff *);
void ieee80211_send_delba(struct ieee80211_node *ni, int tid, int tx, int reason);

void ieee80211_reset_erp(struct ieee80211com *, enum ieee80211_phymode);
void ieee80211_set_shortslottime(struct ieee80211com *, int);
int ieee80211_iserp_rateset(struct ieee80211com *, struct ieee80211_rateset *);
void ieee80211_set11gbasicrates(struct ieee80211_rateset *, enum ieee80211_phymode);
enum ieee80211_phymode ieee80211_get11gbasicrates(struct ieee80211_rateset *);
void ieee80211_send_pspoll(struct ieee80211_node *);
void ieee80211_tkip_mic_failure(struct ieee80211vap *, int count);
void ieee80211_send_rm_req_stastats(struct ieee80211_node *ni, u_int32_t flags);
int32_t ieee80211_send_rm_rep_stastats(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t group_id,
		u_int16_t duration_tu,
		void *sub_item);
void ieee80211_send_rm_req_cca(struct ieee80211_node *ni);
void ieee80211_send_action_cca_report(struct ieee80211_node *ni, uint8_t token,
		uint16_t cca_intf, uint64_t tsf, uint16_t duration, uint32_t sp_fail,
		uint32_t lp_fail, uint16_t others_time, uint8_t *extra_ie, uint16_t ie_len);
void ieee80211_send_rm_req_stastats_all(struct ieee80211com *ic);
void ieee80211_send_rm_req_chan_load(struct ieee80211_node *ni,
				u_int8_t channel,
				u_int16_t duration_ms,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_rm_req_noise_his(struct ieee80211_node *ni,
				u_int8_t channel,
				u_int16_t duration_ms,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_rm_req_beacon(struct ieee80211_node *ni,
				u_int8_t op_class,
				u_int8_t channel,
				u_int16_t duration_ms,
				u_int8_t mode,
				u_int8_t *bssid,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_rm_req_frame(struct ieee80211_node *ni,
				u_int8_t op_class,
				u_int8_t channel,
				u_int16_t duration_ms,
				u_int8_t type,
				u_int8_t *mac_address,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_rm_req_tran_stream_cat(struct ieee80211_node *ni,
				u_int16_t duration_ms,
				u_int8_t *peer_sta,
				u_int8_t tid,
				u_int8_t bin0,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_rm_req_multicast_diag(struct ieee80211_node *ni,
				u_int16_t duration_ms,
				u_int8_t *group_mac,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_link_measure_request(struct ieee80211_node *ni,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_neighbor_report_request(struct ieee80211_node *ni,
				unsigned long expire,
				void *fn_success,
				void *fn_fail);
void ieee80211_send_rm_rep_chan_load(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t op_class,
		u_int8_t channel,
		u_int16_t duration_tu,
		u_int8_t channel_load);
void ieee80211_send_rm_rep_noise_his(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t op_class,
		u_int8_t channel,
		u_int16_t duration_tu,
		u_int8_t antenna_id,
		u_int8_t anpi,
		u_int8_t *ipi);
void ieee80211_send_rm_rep_beacon(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t op_class,
		u_int8_t channel,
		u_int16_t duration_tu,
		u_int8_t reported_frame_info,
		u_int8_t rcpi,
		u_int8_t rsni,
		u_int8_t *bssid,
		u_int8_t antenna_id,
		u_int8_t *parent_tsf);
void ieee80211_send_rm_rep_frame(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t op_class,
		u_int8_t channel,
		u_int16_t duration_tu,
		void *sub_ele);
void ieee80211_send_rm_rep_multicast_diag(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int16_t duration_tu,
		u_int8_t *group_mac,
		u_int8_t reason,
		u_int32_t mul_rec_msdu_cnt,
		u_int16_t first_seq_num,
		u_int16_t last_seq_num,
		u_int16_t mul_rate);
int32_t ieee80211_send_meas_request_basic(struct ieee80211_node *ni,
		u_int8_t channel,
		u_int16_t tsf_offset,
		u_int16_t duration,
		unsigned long expire,
		void *fn_success,
		void *fn_fail);
int32_t ieee80211_send_meas_request_cca(struct ieee80211_node *ni,
		u_int8_t channel,
		u_int16_t tsf_offset,
		u_int16_t duration,
		unsigned long expire,
		void *fn_success,
		void *fn_fail);
int32_t ieee80211_send_meas_request_rpi(struct ieee80211_node *ni,
		u_int8_t channel,
		u_int16_t tsf_offset,
		u_int16_t duration,
		unsigned long expire,
		void *fn_success,
		void *fn_fail);
int32_t ieee80211_send_meas_report_basic(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t channel,
		u_int64_t start_tsf,
		u_int16_t duration,
		u_int8_t basic_report);
int32_t ieee80211_send_meas_report_cca(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t channel,
		u_int64_t start_tsf,
		u_int16_t duration,
		u_int8_t cca_report);
int32_t ieee80211_send_meas_report_rpi(struct ieee80211_node *ni,
		u_int8_t report_mode,
		u_int8_t token,
		u_int8_t meas_token,
		u_int8_t channel,
		u_int64_t start_tsf,
		u_int16_t duration,
		u_int8_t *rpi_report);
void ieee80211_send_neighbor_report_response(struct ieee80211_node *ni,
					u_int8_t token,
					u_int8_t bss_num,
					void *table);
void ieee80211_send_notify_chan_width_action(struct ieee80211vap *vap,
					     struct ieee80211_node *ni,
					     u_int32_t width);
void ieee80211_send_sa_query (struct ieee80211_node *ni, u_int8_t action,
					u_int16_t tid);
/*
 * Return the size of the 802.11 header for a management or data frame.
 */
static __inline int
ieee80211_hdrsize(u_int32_t ht_capable, const void *data)
{
	const struct ieee80211_frame *wh = data;
	int size = sizeof(struct ieee80211_frame);

	/* NB: we don't handle control frames */
	KASSERT((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL,
		("%s: control frame", __func__));
	if ((wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_DSTODS)
		size += IEEE80211_ADDR_LEN;
	if (IEEE80211_QOS_HAS_SEQ(wh))
		size += sizeof(u_int16_t);
	if (ht_capable) {
		if ((wh->i_fc[1] & IEEE80211_FC1_ORDER) == IEEE80211_FC1_ORDER) {
			/* Frame has HT control field in the header */
			size += sizeof(u_int32_t);
		}
	}
	return size;
}

/*
 * Like ieee80211_hdrsize, but handles any type of frame.
 */
static __inline int
ieee80211_anyhdrsize(u_int32_t ht_capable, const void *data)
{
	const struct ieee80211_frame *wh = data;

	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_CTL) {
		switch (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) {
		case IEEE80211_FC0_SUBTYPE_CTS:
		case IEEE80211_FC0_SUBTYPE_ACK:
			return sizeof(struct ieee80211_frame_ack);
		}
		return sizeof(struct ieee80211_frame_min);
	} else {
		if (ht_capable)
			return ieee80211_hdrsize(IEEE80211_HT_CAPABLE, data);
		else
			return ieee80211_hdrsize(IEEE80211_NON_HT_CAPABLE, data);
	}
}

/*
 * Template for an in-kernel authenticator.  Authenticators
 * register with the protocol code and are typically loaded
 * as separate modules as needed.
 */
struct ieee80211_authenticator {
	const char *ia_name;		/* printable name */
	int (*ia_attach)(struct ieee80211vap *);
	void (*ia_detach)(struct ieee80211vap *);
	void (*ia_node_join)(struct ieee80211_node *);
	void (*ia_node_leave)(struct ieee80211_node *);
};
void ieee80211_authenticator_register(int, const struct ieee80211_authenticator *);
void ieee80211_authenticator_unregister(int);
const struct ieee80211_authenticator *ieee80211_authenticator_get(int);

struct eapolcom;
/*
 * Template for an in-kernel authenticator backend.  Backends
 * register with the protocol code and are typically loaded
 * as separate modules as needed.
 */
struct ieee80211_authenticator_backend {
	const char *iab_name;		/* printable name */
	int (*iab_attach)(struct eapolcom *);
	void (*iab_detach)(struct eapolcom *);
};
void ieee80211_authenticator_backend_register(
	const struct ieee80211_authenticator_backend *);
void ieee80211_authenticator_backend_unregister(
	const struct ieee80211_authenticator_backend *);
const struct ieee80211_authenticator_backend *
	ieee80211_authenticator_backend_get(const char *);

/*
 * Template for an MAC ACL policy module.  Such modules
 * register with the protocol code and are passed the sender's
 * address of each received frame for validation.
 */
struct ieee80211_aclator {
	const char *iac_name;		/* printable name */
	int (*iac_attach)(struct ieee80211vap *);
	void (*iac_detach)(struct ieee80211vap *);
	int (*iac_check)(struct ieee80211vap *,
		const u_int8_t mac[IEEE80211_ADDR_LEN]);
	int (*iac_add)(struct ieee80211vap *,
		const u_int8_t mac[IEEE80211_ADDR_LEN]);
	int (*iac_remove)(struct ieee80211vap *,
		const u_int8_t mac[IEEE80211_ADDR_LEN]);
	int (*iac_flush)(struct ieee80211vap *);
	int (*iac_setpolicy)(struct ieee80211vap *, int);
	int (*iac_getpolicy)(struct ieee80211vap *);
};
void ieee80211_aclator_register(const struct ieee80211_aclator *);
void ieee80211_aclator_unregister(const struct ieee80211_aclator *);
const struct ieee80211_aclator *ieee80211_aclator_get(const char *name);

/* flags for ieee80211_fix_rate() */
#define	IEEE80211_F_DOSORT	0x00000001	/* sort rate list */
#define	IEEE80211_F_DOFRATE	0x00000002	/* use fixed rate */
#define	IEEE80211_F_DONEGO	0x00000004	/* calc negotiated rate */
#define	IEEE80211_F_DODEL	0x00000008	/* delete ignore rate */
#define	IEEE80211_F_DOXSECT	0x00000010	/* intersection of rates */
#define	IEEE80211_F_DOBRS	0x00000020	/* check for basic rates */
int	ieee80211_fix_rate(struct ieee80211_node *, int);
int	ieee80211_fix_ht_rate(struct ieee80211_node *, int);

#define IEEE80211_EXPONENT_TO_VALUE(_exp) (1 << (u_int32_t)(_exp)) - 1
#define IEEE80211_TXOP_TO_US(_txop)	(u_int32_t)(_txop) << 5
#define IEEE80211_US_TO_TXOP(_us)	(u_int16_t)((u_int32_t)(_us)) >> 5

struct chanAccParams{
	/* XXX: is there any reason to have multiple instances of cap_info_count??? */
	u_int8_t cap_info_count;		 		/* ver. of the current param set */
	struct wmm_params cap_wmeParams[WME_NUM_AC];	/*WME params for each access class */
};

struct ieee80211_wme_state {
	u_int32_t wme_flags;
#define	WME_F_AGGRMODE	0x00000001	/* STATUS: WME aggressive mode */

	u_int wme_hipri_traffic;			/* VI/VO frames in beacon interval */
	u_int wme_hipri_switch_thresh;		/* aggressive mode switch threshold */
	u_int wme_hipri_switch_hysteresis;	/* aggressive mode switch hysteresis */

	struct chanAccParams wme_wmeChanParams;	/* configured WME parameters applied to itself*/
	struct chanAccParams wme_wmeBssChanParams; /* configured WME parameters broadcasted to STAs*/
	struct chanAccParams wme_chanParams;	/* channel parameters applied to itself*/
	struct chanAccParams wme_bssChanParams;	/* channel parameters broadcasted to STAs*/
	u_int8_t wme_nonAggressiveMode;   	/* don't use aggressive params and use WME params */

#ifdef CONFIG_QVSP
	uint32_t wme_throt_bm;			/* VSP WME throt bitmap */
	struct chanAccParams wme_throt_bssChanParams;
	uint32_t wme_throt_add_qwme_ie;
#endif
	/* update hardware tx params after wme state change */
	int (*wme_update)(struct ieee80211com *);
};

void ieee80211_wme_initparams(struct ieee80211vap *);
void ieee80211_wme_initparams_locked(struct ieee80211vap *);
void ieee80211_wme_updateparams(struct ieee80211vap *vap, int sync_chan_param);
void ieee80211_wme_updateparams_locked(struct ieee80211vap *);
void ieee80211_wme_updateparams_delta(struct ieee80211vap *vap, uint8_t apply_delta);
struct ieee80211_wme_state *ieee80211_vap_get_wmestate(struct ieee80211vap *vap);
void ieee80211_vap_sync_chan_wmestate(struct ieee80211vap *vap);
void ieee80211_adjust_wme_by_vappri(struct ieee80211com *ic);

int ieee80211_open(struct net_device *);
int ieee80211_init(struct net_device *, int);
void ieee80211_start_running(struct ieee80211com *);
int ieee80211_stop(struct net_device *);
void ieee80211_stop_running(struct ieee80211com *);
void ieee80211_beacon_miss(struct ieee80211com *);
int ieee80211_new_state(struct ieee80211vap *, enum ieee80211_state, int);
void ieee80211_print_essid(const u_int8_t *, int);
void ieee80211_dump_pkt(struct ieee80211com *, const u_int8_t *, int, int, int);
struct sk_buff *ieee80211_getcfframe(struct ieee80211vap *, int);
void ieee80211_swbmiss(unsigned long arg);
void ieee80211_swberp(unsigned long arg);


/* used for send formatted string custom event IWEVCUSTOM */
int ieee80211_eventf(struct net_device *dev, const char *fmt, ...);

/*
 * Beacon frames constructed by ieee80211_beacon_alloc
 * have the following structure filled in so drivers
 * can update the frame later w/ minimal overhead.
 */
struct ieee80211_beacon_offsets {
	__le16 *bo_caps;		/* capabilities */
	u_int8_t *bo_tim;		/* start of atim/dtim */
	u_int8_t *bo_wme;		/* start of WME parameters */
	u_int8_t *bo_tim_trailer;	/* start of fixed-size tim trailer */
	u_int16_t bo_tim_len;		/* atim/dtim length in bytes */
	u_int16_t bo_tim_trailerlen;	/* trailer length in bytes */
	u_int8_t *bo_bss_load;          /* start of bss load */
	u_int8_t *bo_chanswitch;	/* where channel switch IE will go */
	u_int8_t *bo_ath_caps;		/* where ath caps is */
	u_int8_t *bo_xr;		/* start of xr element */
	u_int8_t *bo_cca;		/* start of clear channel assessment element. */
	u_int8_t *bo_htcap;		/* start of HT Capability element */
	u_int8_t *bo_htinfo;		/* start of HT Info element */
	u_int8_t *bo_tpc_rep;		/* start of TPC Report IE*/
	u_int8_t *bo_erp;		/* start of ERP element */
	u_int8_t *bo_appie_buf;		/* start of APP IE buf */
	u_int16_t bo_appie_buf_len;	/* APP IE buf length in bytes */
	u_int16_t bo_chanswitch_trailerlen;
};
struct sk_buff *ieee80211_beacon_alloc(struct ieee80211_node *,
	struct ieee80211_beacon_offsets *);
int ieee80211_beacon_update(struct ieee80211_node *,
	struct ieee80211_beacon_offsets *, struct sk_buff *, int);

/* XXX exposed due to of beacon code botch */
u_int8_t *ieee80211_add_rates(u_int8_t *, const struct ieee80211_rateset *);
u_int8_t *ieee80211_add_xrates(u_int8_t *, const struct ieee80211_rateset *);
u_int8_t *ieee80211_add_bss_load(u_int8_t *, struct ieee80211vap *);
u_int8_t *ieee80211_add_extcap(u_int8_t *);
u_int8_t *ieee80211_add_wpa(u_int8_t *, struct ieee80211vap *);
u_int8_t *ieee80211_add_csa(u_int8_t *, u_int8_t, u_int8_t, u_int8_t);
u_int8_t *ieee80211_add_erp(u_int8_t *, struct ieee80211com *);
u_int8_t *ieee80211_add_athAdvCap(u_int8_t *, u_int8_t, u_int16_t);
u_int8_t *ieee80211_add_xr_param(u_int8_t *, struct ieee80211vap *);
u_int8_t *ieee80211_add_xr_param(u_int8_t *, struct ieee80211vap *);
u_int8_t *ieee80211_add_wme_param(u_int8_t *, struct ieee80211_wme_state *, int, int);
u_int8_t *ieee80211_add_country(u_int8_t *, struct ieee80211com *);
u_int8_t *ieee80211_add_country(u_int8_t *, struct ieee80211com *);
u_int8_t *ieee80211_add_athAdvCap(u_int8_t *, u_int8_t, u_int16_t);
uint8_t *ieee80211_add_qtn_ie(uint8_t *frm, struct ieee80211com *ic, uint8_t flags,
				uint8_t my_flags, uint8_t implicit_ba, uint16_t implicit_ba_size,
				uint32_t rate_train);
u_int8_t *ieee80211_add_chansw_wrap(u_int8_t *, struct ieee80211com *);
u_int8_t *ieee80211_add_wband_chanswitch(u_int8_t *, struct ieee80211com *);
u_int8_t *ieee80211_add_vhttxpwr_envelope(u_int8_t *, struct ieee80211com *);
#ifdef CONFIG_QVSP
uint8_t *ieee80211_add_qtn_wme_param(struct ieee80211vap *, u_int8_t *);
#endif
u_int8_t *ieee80211_add_htcap(struct ieee80211_node *, u_int8_t *, struct ieee80211_htcap *, int subtype);
u_int8_t *ieee80211_add_htinfo(struct ieee80211_node *, u_int8_t *, struct ieee80211_htinfo *);

u_int8_t *ieee80211_add_vhtcap(struct ieee80211_node *, u_int8_t *, struct ieee80211_vhtcap *);
u_int8_t *ieee80211_add_vhtop(struct ieee80211_node *, u_int8_t *, struct ieee80211_vhtop *);

/* MU MIMO */
void
ieee80211_send_vht_grp_id_mgmt_action(struct ieee80211vap *vap,
				      struct ieee80211_node *ni);
void ieee80211_node_mu_grp_update(struct ieee80211_node *ni, uint8_t grp,
				  uint8_t pos, uint8_t delete);
struct ieee80211_node *ieee80211_find_node_by_aid(struct ieee80211com *ic, uint8_t aid);

u_int8_t *ieee80211_add_qtn_extender_role_ie(uint8_t *frm, uint8_t role);
u_int8_t *ieee80211_add_qtn_extender_bssid_ie(struct ieee80211vap *vap, uint8_t *frm);
int ieee80211_extender_send_event(struct ieee80211vap *vap,
	const struct qtn_wds_ext_event_data *p_wds_event_data, uint8_t *ie);
struct ieee80211_extender_wds_info *ieee80211_extender_find_peer_wds_info(struct ieee80211vap *,
	uint8_t *);
int ieee80211_extender_remove_peer_wds_info(struct ieee80211vap *vap, uint8_t *mac_addr);
void ieee80211_extender_notify_ext_role(struct ieee80211_node *ni);
void ieee80211_extender_sta_update_info(struct ieee80211_node *ni,
		const struct ieee80211_qtn_ext_role *ie_role,
		const struct ieee80211_qtn_ext_bssid *ie_bssid);
struct ieee80211_scanparams;
void extender_event_data_prepare(struct ieee80211com *ic,
			struct ieee80211_scanparams *p_scan,
			struct qtn_wds_ext_event_data *data,
			uint8_t cmd,
			uint8_t *peer_mac);
void ieee80211_extender_cleanup_wds_link(struct ieee80211vap *vap);
void ieee80211_extender_vdetach(struct ieee80211vap *vap);
uint8_t *ieee80211_add_qtn_tdls_sta_info(uint8_t *frm, void *sta_info);
/*
 * Notification methods called from the 802.11 state machine.
 * Note that while these are defined here, their implementation
 * is OS-specific.
 */
void ieee80211_notify_node_join(struct ieee80211_node *, int);
void ieee80211_notify_node_leave(struct ieee80211_node *);
void ieee80211_notify_scan_done(struct ieee80211vap *);
void ieee80211_notify_sta_stats(struct ieee80211_node *ni);
void ieee80211_nofity_sta_require_leave(struct ieee80211_node *ni);

#endif /* _NET80211_IEEE80211_PROTO_H_ */
