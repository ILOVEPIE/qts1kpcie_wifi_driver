/*-
 * Copyright (c) 2010 Quantenna Systems
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
 */
#ifndef _NET80211_IEEE80211_DOT11_MSG_H_
#define _NET80211_IEEE80211_DOT11_MSG_H_

/* Enumeration for dot11 messages */
enum ieee80211_dot11_msg_message {
	IEEE80211_DOT11_MSG_CLIENT_CONNECTED = 0,
	IEEE80211_DOT11_MSG_CLIENT_DISCONNECTED,
	IEEE80211_DOT11_MSG_CLIENT_AUTH_FAILED,
	IEEE80211_DOT11_MSG_CLIENT_REMOVED,
	IEEE80211_DOT11_MSG_AP_CONNECTED,
	IEEE80211_DOT11_MSG_AP_CONNECTION_FAILED,
	IEEE80211_DOT11_MSG_AP_DISCONNECTED,
};

/* Enumeration for message codes */
enum ieee80211_dot11_msg_reason {
	IEEE80211_DOT11_MSG_REASON_DISASSOCIATED = 0,
	IEEE80211_DOT11_MSG_REASON_DEAUTHENTICATED,
	IEEE80211_DOT11_MSG_REASON_TKIP_CMEASURES,
	IEEE80211_DOT11_MSG_REASON_CLIENT_TIMEOUT,
	IEEE80211_DOT11_MSG_REASON_WPA_PASSWORD_FAIL,
	IEEE80211_DOT11_MSG_REASON_WPA_TIMEOUT,
	IEEE80211_DOT11_MSG_REASON_BEACON_LOSS,
	IEEE80211_DOT11_MSG_REASON_CLIENT_SENT_DEAUTH,
	IEEE80211_DOT11_MSG_REASON_CLIENT_SENT_DISASSOC,
};

/* FIXME: this value must correspond to the d11_r array as defined in ieee80211_wireless.c */
#define DOT11_MAX_REASON_CODE 45

#endif /* _NET80211_IEEE80211_DOT11_MSG_H_ */
