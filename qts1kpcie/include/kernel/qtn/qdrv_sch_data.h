/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2012 Quantenna Communications, Inc.                 **
**                            All Rights Reserved                            **
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
EH1*/

#ifndef _QDRV_SCH_DATA_H
#define _QDRV_SCH_DATA_H

#include <qtn/qtn_global.h>
#include <qdrv_sch_const.h>
#include <common/queue.h>

#define QDRV_SCH_SHARED_AC_DATA_DEQUEUE_LIMIT	4

struct qdrv_sch_shared_data;

struct qdrv_sch_node_band_data {
	TAILQ_ENTRY(qdrv_sch_node_band_data) nbd_next;
	struct sk_buff_head queue;
	uint32_t sent;
	uint32_t dropped;
	uint32_t dropped_victim;
};

struct qdrv_sch_node_data {
	struct qdrv_sch_node_band_data bands[QDRV_SCH_BANDS];
	struct qdrv_sch_shared_data *shared_data;
	struct Qdisc *qdisc;
	uint16_t used_tokens;
	uint16_t muc_queued;
	uint8_t over_thresh;
	uint32_t over_thresh_cnt;
	uint32_t low_rate;
	struct device_attribute sysfs_attr;
};

static inline struct qdrv_sch_node_data *
qdrv_sch_get_node_data(struct qdrv_sch_node_band_data *nbd, uint8_t band)
{
	return container_of(nbd, struct qdrv_sch_node_data, bands[band]);
}

struct qdrv_sch_shared_band_data {
	int consec_dequeues;
	TAILQ_HEAD(, qdrv_sch_node_band_data) active_nodes;
};

struct qdrv_sch_shared_data {
	struct list_head entry;
	spinlock_t lock;
	char dev_name[IFNAMSIZ];
	uint8_t queuing_alg;
	int16_t total_tokens;
	int16_t users;
	struct sk_buff *held_skb;
	struct Qdisc *held_skb_sch;
	int16_t available_tokens;
	uint16_t reserved_tokens_per_user;
	uint16_t random_drop_threshold;
	void (*drop_callback)(struct sk_buff *);
	struct qdrv_sch_shared_band_data bands[QDRV_SCH_BANDS];
};

#define qdrv_sch_shared_data_lock(qsh, flags)	spin_lock_irqsave(&qsh->lock, flags)
#define qdrv_sch_shared_data_unlock(qsh, flags)	spin_unlock_irqrestore(&qsh->lock, flags)

#endif

