/*-
 * Copyright (c) 2005 John Bicket
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
 * $Id: ieee80211_monitor.h 2602 2007-07-24 14:15:34Z kelmo $
 */
#ifndef _NET80211_IEEE80211_MONITOR_H_
#define _NET80211_IEEE80211_MONITOR_H_


#ifndef ARPHRD_IEEE80211_RADIOTAP
#define ARPHRD_IEEE80211_RADIOTAP	803 /* IEEE 802.11 + radiotap header */
#endif /* ARPHRD_IEEE80211_RADIOTAP */

#ifndef ARPHRD_IEEE80211_ATHDESC
#define ARPHRD_IEEE80211_ATHDESC	804 /* IEEE 802.11 + atheros descriptor */
#endif /* ARPHRD_IEEE80211_RADIOTAP */

#define ATHDESC_HEADER_SIZE	32
#include <compat.h>
#include "net80211/ieee80211_radiotap.h"
struct ieee80211_phy_params {
	u_int8_t rate0;
	u_int8_t rate1;
	u_int8_t rate2;
	u_int8_t rate3;

	u_int8_t try0;
	u_int8_t try1;
	u_int8_t try2;
	u_int8_t try3;

	u_int8_t power;
	u_int32_t flags;
};



enum {
	DIDmsg_lnxind_wlansniffrm		= 0x00000044,
	DIDmsg_lnxind_wlansniffrm_hosttime	= 0x00010044,
	DIDmsg_lnxind_wlansniffrm_mactime	= 0x00020044,
	DIDmsg_lnxind_wlansniffrm_channel	= 0x00030044,
	DIDmsg_lnxind_wlansniffrm_rssi		= 0x00040044,
	DIDmsg_lnxind_wlansniffrm_sq		= 0x00050044,
	DIDmsg_lnxind_wlansniffrm_signal	= 0x00060044,
	DIDmsg_lnxind_wlansniffrm_noise		= 0x00070044,
	DIDmsg_lnxind_wlansniffrm_rate		= 0x00080044,
	DIDmsg_lnxind_wlansniffrm_istx		= 0x00090044,
	DIDmsg_lnxind_wlansniffrm_frmlen	= 0x000A0044
};
enum {
	P80211ENUM_msgitem_status_no_value	= 0x00
};
enum {
	P80211ENUM_truth_false			= 0x00,
	P80211ENUM_truth_true			= 0x01
};

/*
 * Transmit descriptor status.  This structure is filled
 * in only after the tx descriptor process method finds a
 * ``done'' descriptor; at which point it returns something
 * other than HAL_EINPROGRESS.
 *
 * Note that ts_antenna may not be valid for all h/w.  It
 * should be used only if non-zero.
 */
struct ath_tx_status {
        u_int16_t       ts_seqnum;      /* h/w assigned sequence number */
        u_int16_t       ts_tstamp;      /* h/w assigned timestamp */
        u_int8_t        ts_status;      /* frame status, 0 => xmit ok */
        u_int8_t        ts_rate;        /* h/w transmit rate index */
        int8_t          ts_rssi;        /* tx ack RSSI */
        u_int8_t        ts_shortretry;  /* # short retries */
        u_int8_t        ts_longretry;   /* # long retries */
        u_int8_t        ts_virtcol;     /* virtual collision count */
        u_int8_t        ts_antenna;     /* antenna information */
};

/*
 * Receive descriptor status.  This structure is filled
 * in only after the rx descriptor process method finds a
 * ``done'' descriptor; at which point it returns something
 * other than HAL_EINPROGRESS.
 *
 * If rx_status is zero, then the frame was received ok;
 * otherwise the error information is indicated and rs_phyerr
 * contains a phy error code if HAL_RXERR_PHY is set.  In general
 * the frame contents is undefined when an error occurred thought
 * for some errors (e.g. a decryption error), it may be meaningful.
 *
 * Note that the receive timestamp is expanded using the TSF to
 * 15 bits (regardless of what the h/w provides directly).
 *
 * rx_rssi is in units of dbm above the noise floor.  This value
 * is measured during the preamble and PLCP; i.e. with the initial
 * 4us of detection.  The noise floor is typically a consistent
 * -96dBm absolute power in a 20MHz channel.
 */
struct ath_rx_status {
        u_int16_t       rs_datalen;     /* rx frame length */
        u_int16_t       rs_tstamp;      /* h/w assigned timestamp */
        u_int8_t        rs_status;      /* rx status, 0 => recv ok */
        u_int8_t        rs_phyerr;      /* phy error code */
        int8_t          rs_rssi;        /* rx frame RSSI */
        u_int8_t        rs_keyix;       /* key cache index */
        u_int8_t        rs_rate;        /* h/w receive rate index */
        u_int8_t        rs_antenna;     /* antenna information */
        u_int8_t        rs_more;        /* more descriptors follow */
};

/*
 * Definitions for the software frame/packet descriptors used by
 * the Quantenna HAL.  This definition obscures hardware-specific
 * details from the driver.  Drivers are expected to fillin the
 * portions of a descriptor that are not opaque then use HAL calls
 * to complete the work.  Status for completed frames is returned
 * in a device-independent format.
 */
#define ds_txstat       ds_us.tx
#define ds_rxstat       ds_us.rx

#define HAL_RXERR_CRC           0x01    /* CRC error on frame */
#define HAL_RXERR_PHY           0x02    /* PHY error, rs_phyerr is valid */
#define HAL_RXERR_FIFO          0x04    /* fifo overrun */
#define HAL_RXERR_DECRYPT       0x08    /* non-Michael decrypt error */
#define HAL_RXERR_MIC   

struct qnt_desc {
        /*
         * The following definitions are passed directly
         * the hardware and managed by the HAL.  Drivers
         * should not touch those elements marked opaque.
         */
        u_int32_t       ds_link;        /* phys address of next descriptor */
        u_int32_t       ds_data;        /* phys address of data buffer */
        u_int32_t       ds_ctl0;        /* opaque DMA control 0 */
        u_int32_t       ds_ctl1;        /* opaque DMA control 1 */
        u_int32_t       ds_hw[4];       /* opaque h/w region */
        /*
         * The remaining definitions are managed by software;
         * these are valid only after the rx/tx process descriptor
         * methods return a non-EINPROGRESS  code.
         */
        union {
                struct ath_tx_status tx;/* xmit status */
                struct ath_rx_status rx;/* recv status */
        } ds_us;
        void            *ds_vdata;      /* virtual addr of data buffer */
} __packed;


typedef struct {
        u_int32_t did;
        u_int16_t status;
        u_int16_t len;
        u_int32_t data;
} p80211item_uint32_t;

typedef struct {
        u_int32_t msgcode;
        u_int32_t msglen;
#define WLAN_DEVNAMELEN_MAX 16
        u_int8_t devname[WLAN_DEVNAMELEN_MAX];
        p80211item_uint32_t hosttime;
        p80211item_uint32_t mactime;
        p80211item_uint32_t channel;
        p80211item_uint32_t rssi;
        p80211item_uint32_t sq;
        p80211item_uint32_t signal;
        p80211item_uint32_t noise;
        p80211item_uint32_t rate;
        p80211item_uint32_t istx;
        p80211item_uint32_t frmlen;
} wlan_ng_prism2_header;



#define ATH_RX_RADIOTAP_PRESENT (               \
	(1 << IEEE80211_RADIOTAP_TSFT)		| \
        (1 << IEEE80211_RADIOTAP_FLAGS)         | \
        (1 << IEEE80211_RADIOTAP_RATE)          | \
        (1 << IEEE80211_RADIOTAP_CHANNEL)       | \
	(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL)	| \
	(1 << IEEE80211_RADIOTAP_DBM_ANTNOISE)	| \
        (1 << IEEE80211_RADIOTAP_ANTENNA)       | \
        (1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL)  | \
        0)

struct ath_rx_radiotap_header {
        struct ieee80211_radiotap_header wr_ihdr;
	__le64 wr_tsft;
        u_int8_t wr_flags;
        u_int8_t wr_rate;
        __le16 wr_chan_freq;
        __le16 wr_chan_flags;
	int8_t  wr_dbm_antsignal;
	int8_t  wr_dbm_antnoise;
        u_int8_t wr_antenna;
        u_int8_t wr_antsignal;
}__attribute__((__packed__));

#define ATH_TX_RADIOTAP_PRESENT (               \
	(1 << IEEE80211_RADIOTAP_TSFT)		| \
        (1 << IEEE80211_RADIOTAP_FLAGS)         | \
        (1 << IEEE80211_RADIOTAP_RATE)          | \
        (1 << IEEE80211_RADIOTAP_DBM_TX_POWER)  | \
        (1 << IEEE80211_RADIOTAP_ANTENNA)       | \
        0)

struct ath_tx_radiotap_header {
        struct ieee80211_radiotap_header wt_ihdr;
	__le64 wt_tsft;
        u_int8_t wt_flags;	
        u_int8_t wt_rate;
        u_int8_t wt_txpower;
        u_int8_t wt_antenna;
};


void ieee80211_monitor_encap(struct ieee80211vap *, struct sk_buff *);


#endif /* _NET80211_IEEE80211_MONITOR_H_ */
