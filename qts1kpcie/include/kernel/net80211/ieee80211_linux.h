/*-
 * Copyright (c) 2003-2005 Sam Leffler, Errno Consulting
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
 * $Id: ieee80211_linux.h 2696 2007-08-30 01:59:13Z kelmo $
 */
#ifndef _NET80211_IEEE80211_LINUX_H_
#define _NET80211_IEEE80211_LINUX_H_

#include <linux/version.h>
#include <linux/wireless.h>
#include <linux/fs.h>
#include "compat.h"

/*
 * Compatibility definition of statistics flags
 * (bitmask in (struct iw_quality *)->updated)
 */
#ifndef IW_QUAL_QUAL_UPDATED
#define IW_QUAL_QUAL_UPDATED	0x01	/* Value was updated since last read */
#define IW_QUAL_LEVEL_UPDATED	0x02
#define IW_QUAL_NOISE_UPDATED	0x04
#define IW_QUAL_QUAL_INVALID	0x10	/* Driver doesn't provide value */
#define IW_QUAL_LEVEL_INVALID	0x20
#define IW_QUAL_NOISE_INVALID	0x40
#endif /* IW_QUAL_QUAL_UPDATED */

#ifndef IW_QUAL_ALL_UPDATED
#define IW_QUAL_ALL_UPDATED \
	(IW_QUAL_QUAL_UPDATED | IW_QUAL_LEVEL_UPDATED | IW_QUAL_NOISE_UPDATED)
#endif
#ifndef IW_QUAL_ALL_INVALID
#define IW_QUAL_ALL_INVALID \
	(IW_QUAL_QUAL_INVALID | IW_QUAL_LEVEL_INVALID | IW_QUAL_NOISE_INVALID)
#endif

void set_quality(struct iw_quality *iq, u_int rssi, int noise);

/*
 * Task deferral
 *
 * Deduce if tasklets are available.  If not then
 * fall back to using the immediate work queue.
 */
#include <linux/interrupt.h>
#ifdef DECLARE_TASKLET			/* native tasklets */
#define IEEE80211_TQ_STRUCT tasklet_struct
#define IEEE80211_INIT_TQUEUE(a,b,c)	tasklet_init((a),(b),(unsigned long)(c))
#define IEEE80211_SCHEDULE_TQUEUE(a)	tasklet_schedule((a))
#define IEEE80211_CANCEL_TQUEUE(a)	if (!in_interrupt()) tasklet_kill((a))
typedef unsigned long IEEE80211_TQUEUE_ARG;
#define mark_bh(a) do {} while (0)
#else					/* immediate work queue */
#define IEEE80211_TQ_STRUCT tq_struct
#define IEEE80211_INIT_TQUEUE(a,b,c)		INIT_TQUEUE(a,b,c)
#define IEEE80211_SCHEDULE_TQUEUE(a) do { \
	int __macro_needmark;		\
	__macro_needmark |= queue_task((a), &tq_immediate);	\
	if (__macro_needmark)		\
		mark_bh(IMMEDIATE_BH);	\
} while(0)
typedef void *IEEE80211_TQUEUE_ARG;
#define	tasklet_disable(t)	do { (void) t; local_bh_disable(); } while (0)
#define	tasklet_enable(t)	do { (void) t; local_bh_enable(); } while (0)
/* XXX: not supporting cancel in old kernels! */
#define IEEE80211_CANCEL_TQUEUE(a)	((a),0)
#endif /* !DECLARE_TASKLET */

#define	IEEE80211_RESCHEDULE	schedule

/*
 * Beacon handler locking definitions.
 * Beacon locking 
 * UAPSD locking 
 */
typedef spinlock_t ieee80211com_lock_t;
#define	IEEE80211_LOCK_INIT(_ic, _name)				\
	spin_lock_init(&(_ic)->ic_comlock)
#define	IEEE80211_LOCK_DESTROY(_ic)
#define	IEEE80211_LOCK_IRQ(_ic) do {				\
	unsigned long __ilockflags;				\
	spin_lock_irqsave(&(_ic)->ic_comlock, __ilockflags);
#define	IEEE80211_UNLOCK_IRQ(_ic)					\
	spin_unlock_irqrestore(&(_ic)->ic_comlock, __ilockflags);	\
} while (0)
#define	IEEE80211_UNLOCK_IRQ_EARLY(_ic)					\
	spin_unlock_irqrestore(&(_ic)->ic_comlock, __ilockflags);
#define IEEE80211_LOCK_BH(_ic)	spin_lock_bh(&(_ic)->ic_comlock)
#define IEEE80211_UNLOCK_BH(_ic) spin_unlock_bh(&(_ic)->ic_comlock)
#define IEEE80211_LOCK(_ic)	spin_lock(&(_ic)->ic_comlock)
#define IEEE80211_UNLOCK(_ic)	spin_unlock(&(_ic)->ic_comlock)

/* NB: beware, spin_is_locked() is unusable for !SMP */
#if defined(CONFIG_SMP)
#define	IEEE80211_LOCK_ASSERT(_ic) \
	KASSERT(spin_is_locked(&(_ic)->ic_comlock),("ieee80211com not locked!"))
#else
#define	IEEE80211_LOCK_ASSERT(_ic)
#endif

#define IEEE80211_VAPS_LOCK_INIT(_ic, _name)		\
	spin_lock_init(&(_ic)->ic_vapslock)
#define IEEE80211_VAPS_LOCK_DESTROY(_ic)
#define IEEE80211_VAPS_LOCK(_ic)	spin_lock(&(_ic)->ic_vapslock);
#define IEEE80211_VAPS_UNLOCK(_ic)	spin_unlock(&(_ic)->ic_vapslock);
#define IEEE80211_VAPS_LOCK_BH(_ic)	spin_lock_bh(&(_ic)->ic_vapslock);
#define IEEE80211_VAPS_UNLOCK_BH(_ic)	spin_unlock_bh(&(_ic)->ic_vapslock);
#define IEEE80211_VAPS_LOCK_IRQ(_ic)	do {	\
	int _vaps_lockflags;			\
	spin_lock_irqsave(&(_ic)->ic_vapslock, _vaps_lockflags);
#define IEEE80211_VAPS_UNLOCK_IRQ(_ic)	\
	spin_unlock_irqrestore(&(_ic)->ic_vapslock, _vaps_lockflags); \
} while (0)
#define IEEE80211_VAPS_UNLOCK_IRQ_EARLY(_ic)	spin_unlock_irqrestore(&(_ic)->ic_vapslock, _vaps_lockflags)


/* NB: beware, spin_is_locked() is unusable for !SMP */
#if defined(CONFIG_SMP)
#define IEEE80211_VAPS_LOCK_ASSERT(_ic) \
	KASSERT(spin_is_locked(&(_ic)->ic_vapslock),("ieee80211com_vaps not locked!"))
#else
#define IEEE80211_VAPS_LOCK_ASSERT(_ic)
#endif


/*
 * Node locking definitions.
 */
typedef spinlock_t ieee80211_node_lock_t;
#define	IEEE80211_NODE_LOCK_INIT(_nt, _name)	spin_lock_init(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_LOCK_DESTROY(_nt)
#define	IEEE80211_NODE_LOCK(_nt)	spin_lock(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_UNLOCK(_nt)	spin_unlock(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_LOCK_BH(_nt)	spin_lock_bh(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_UNLOCK_BH(_nt)	spin_unlock_bh(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_LOCK_IRQ(_nt)	do {	\
	unsigned long __node_lockflags;		\
	spin_lock_irqsave(&(_nt)->nt_nodelock, __node_lockflags);
#define	IEEE80211_NODE_UNLOCK_IRQ(_nt)		\
	spin_unlock_irqrestore(&(_nt)->nt_nodelock, __node_lockflags); \
} while(0)
#define	IEEE80211_NODE_UNLOCK_IRQ_EARLY(_nt)		\
	spin_unlock_irqrestore(&(_nt)->nt_nodelock, __node_lockflags);

/* NB: beware, *_is_locked() are bogusly defined for UP+!PREEMPT */
#if (defined(CONFIG_SMP) || defined(CONFIG_PREEMPT)) && defined(spinlock_is_locked)
#define	IEEE80211_NODE_LOCK_ASSERT(_nt) \
	KASSERT(spinlock_is_locked(&(_nt)->nt_nodelock), \
		("802.11 node table not locked!"))
#else
#define	IEEE80211_NODE_LOCK_ASSERT(_nt)
#endif

/*
 * Node table scangen locking definitions.
 */
typedef spinlock_t ieee80211_scan_lock_t;
#define	IEEE80211_SCAN_LOCK_INIT(_nt, _name) spin_lock_init(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_LOCK_DESTROY(_nt)
#define	IEEE80211_SCAN_LOCK_BH(_nt)	spin_lock_bh(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_UNLOCK_BH(_nt)	spin_unlock_bh(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_LOCK_IRQ(_nt)	do {	\
	unsigned long __scan_lockflags;		\
	spin_lock_irqsave(&(_nt)->nt_scanlock, __scan_lockflags);
#define	IEEE80211_SCAN_UNLOCK_IRQ(_nt)		\
	spin_unlock_irqrestore(&(_nt)->nt_scanlock, __scan_lockflags); \
} while (0)
#define	IEEE80211_SCAN_UNLOCK_IRQ_EARLY(_nt)		\
	spin_unlock_irqrestore(&(_nt)->nt_scanlock, __scan_lockflags);

/* NB: beware, spin_is_locked() is unusable for !SMP */
#if defined(CONFIG_SMP)
#define	IEEE80211_SCAN_LOCK_ASSERT(_nt) \
	KASSERT(spin_is_locked(&(_nt)->nt_scanlock), ("scangen not locked!"))
#else
#define	IEEE80211_SCAN_LOCK_ASSERT(_nt)
#endif

/*
 * 802.1x MAC ACL database locking definitions.
 */
typedef spinlock_t acl_lock_t;
#define	ACL_LOCK_INIT(_as, _name)	spin_lock_init(&(_as)->as_lock)
#define	ACL_LOCK_DESTROY(_as)
#define	ACL_LOCK(_as)			spin_lock(&(_as)->as_lock)
#define	ACL_UNLOCK(_as)			spin_unlock(&(_as)->as_lock)
#define	ACL_LOCK_BH(_as)		spin_lock_bh(&(_as)->as_lock)
#define	ACL_UNLOCK_BH(_as)		spin_unlock_bh(&(_as)->as_lock)

/* NB: beware, spin_is_locked() is unusable for !SMP */
#if defined(CONFIG_SMP)
#define	ACL_LOCK_ASSERT(_as) \
	KASSERT(spin_is_locked(&(_as)->as_lock), ("ACL not locked!"))
#else
#define	ACL_LOCK_ASSERT(_as)
#endif

/*
 * Per-node power-save queue definitions.  Beware of control
 * flow with IEEE80211_NODE_SAVEQ_LOCK/IEEE80211_NODE_SAVEQ_UNLOCK.
 */
#define	IEEE80211_NODE_SAVEQ_INIT(_ni, _name) do {		\
	skb_queue_head_init(&(_ni)->ni_savedq);			\
} while (0)
#define	IEEE80211_NODE_SAVEQ_DESTROY(_ni)
#define	IEEE80211_NODE_SAVEQ_QLEN(_ni)	skb_queue_len(&(_ni)->ni_savedq)
#define	IEEE80211_NODE_SAVEQ_LOCK(_ni)				\
	spin_lock(&(_ni)->ni_savedq.lock)
#define	IEEE80211_NODE_SAVEQ_UNLOCK(_ni)			\
	spin_unlock(&(_ni)->ni_savedq.lock)
#define	IEEE80211_NODE_SAVEQ_LOCK_IRQ(_ni) do {			\
	unsigned long __sqlockflags;				\
	spin_lock_irqsave(&(_ni)->ni_savedq.lock, __sqlockflags);
#define	IEEE80211_NODE_SAVEQ_UNLOCK_IRQ(_ni)			\
	spin_unlock_irqrestore(&(_ni)->ni_savedq.lock, __sqlockflags);\
} while (0)

/* caller MUST lock IEEE80211_NODE_SAVEQ */
#define	IEEE80211_NODE_SAVEQ_DEQUEUE(_ni, _skb, _qlen) do {	\
	_skb = __skb_dequeue(&(_ni)->ni_savedq);		\
	(_qlen) = skb_queue_len(&(_ni)->ni_savedq);		\
} while (0)
#define	_IEEE80211_NODE_SAVEQ_ENQUEUE(_ni, _skb, _qlen, _age) do {\
	struct sk_buff *tail = skb_peek_tail(&(_ni)->ni_savedq);\
	if (tail != NULL) {					\
		_age -= M_AGE_GET(tail);			\
		__skb_append(tail, _skb, &(_ni)->ni_savedq);	\
	} else {						\
		__skb_queue_head(&(_ni)->ni_savedq, _skb);	\
	}							\
	M_AGE_SET(_skb, _age);					\
	(_qlen) = skb_queue_len(&(_ni)->ni_savedq);		\
} while (0)

struct ieee80211com;
struct ieee80211vap;

int ieee80211_load_module(const char *);

/*
 * Node reference counting definitions.
 *
 * ieee80211_node_initref	initialize the reference count to 1
 * ieee80211_node_incref	add a reference
 * ieee80211_node_dectestref	remove a reference and return 1 if this
 *				is the last reference, otherwise 0
 * ieee80211_node_refcnt	reference count for printing (only)
 */
#define ieee80211_node_initref(_ni)	atomic_set(&(_ni)->ni_refcnt, 1)
#define ieee80211_node_incref(_ni)	atomic_inc(&(_ni)->ni_refcnt)
#define ieee80211_node_decref(_ni)	atomic_dec(&(_ni)->ni_refcnt)
#define	ieee80211_node_dectestref(_ni)	atomic_dec_and_test(&(_ni)->ni_refcnt)
#define	ieee80211_node_refcnt(_ni)	(_ni)->ni_refcnt.counter

#define	le16toh(_x)	le16_to_cpu(_x)
#define	htole16(_x)	cpu_to_le16(_x)
#define	le32toh(_x)	le32_to_cpu(_x)
#define	htole32(_x)	cpu_to_le32(_x)
#define	be16toh(_x)	be16_to_cpu(_x)
#define	htobe16(_x)	cpu_to_be16(_x)
#define	be32toh(_x)	be32_to_cpu(_x)
#define	htobe32(_x)	cpu_to_be32(_x)

/*
 * Linux has no equivalents to malloc types so null these out.
 */
#define	MALLOC_DEFINE(type, shortdesc, longdesc)
#define	MALLOC_DECLARE(type)

/*
 * flags to malloc.
 */
#define	M_NOWAIT	0x0001		/* do not block */
#define	M_WAITOK	0x0002		/* ok to block */
#define	M_ZERO		0x0100		/* bzero the allocation */

/* Debug memory alloc/free. Keeps a running total of allocated/freed
 * dynamic memory on the WLAN driver.
 */
#define WLAN_MALLOC_FREE_TOT_DEBUG

#ifdef WLAN_MALLOC_FREE_TOT_DEBUG
/* Total number of bytes allocated using the MALLOC macro */
extern int g_wlan_tot_alloc;
extern int g_wlan_tot_alloc_cnt;
/* Total number of bytes freed using the FREE macro */
extern int g_wlan_tot_free;
extern int g_wlan_tot_free_cnt;
/* g_wlan_tot_alloc - g_wlan_tot_free - convenience value */
extern int g_wlan_balance;
#endif

static __inline void
ieee80211_safe_wait_ms(uint32_t wait_ms, int is_proc)
{
#define	IEEE80211_SAFE_WAIT_MS_MAX	1000
	unsigned long start_time = jiffies;
	uint32_t ret = 0;

	KASSERT(wait_ms <= IEEE80211_SAFE_WAIT_MS_MAX,
		("%s: safe wait limit exceeded\n", __func__));

	while (!ret) {
		if (is_proc) {
			msleep(wait_ms);
			ret = 1;
		} else {
			ret = time_after(jiffies,
				start_time + msecs_to_jiffies(wait_ms));
		}
	}
}

/* WLAN driver malloc - chain into kmalloc while supporting extra
 * features such as zeroing the memory. 
 */
static __inline void *
ieee80211_malloc(size_t size, int flags)
{
#ifndef WLAN_MALLOC_FREE_TOT_DEBUG
	void *p = kmalloc(size, flags & M_NOWAIT ? GFP_ATOMIC : GFP_KERNEL);
	if (p && (flags & M_ZERO))
		memset(p, 0, size);
	return p;
#else
	/* Debug version of the MALLOC - add in extra fields for accounting */
	/* The memory looks like: | Size | Magic | Block ..... | 
	 * Where magic is 0xDEADBEEF and is used to ensure we're only
	 * doing accounting/pointer juggling on blocks allocated using
	 * this MALLOC
	 */
	void *p = kmalloc(size + (2 * sizeof(int)), flags & M_NOWAIT ? GFP_ATOMIC : GFP_KERNEL);
	if (p)
	{
		int *p_size = (int *)p;
		if (flags & M_ZERO)
		{
			memset(p, 0, size + (2 * sizeof(int)));
		}
		/* First extra word is the length of the allocation */
		*p_size = size;
		p_size++;
		/* Magic value for making free work properly */
		*p_size = 0xDEADBEEF;
		p_size++;
		p = p_size;
		/* Accounting */
		g_wlan_tot_alloc += size;
		g_wlan_tot_alloc_cnt++;
		/* This is NOT thread safe */
		g_wlan_balance += size;
	}
	return p;
#endif
}

#ifdef WLAN_MALLOC_FREE_TOT_DEBUG
static __inline void
ieee80211_free(void *addr)
{
	int *p_size = (int *)(addr);
	p_size--;
	/* Only do pointer juggling for blocks allocated using MALLOC */
	if (*p_size == 0xDEADBEEF)
	{
		p_size--;
		g_wlan_tot_free += *p_size;
		g_wlan_tot_free_cnt++;
		/* This is NOT thread safe */
		g_wlan_balance -= *p_size;
	}
	else
	{
		/* Invalid - block not allocated using 'MALLOC' passed in */
		p_size++;
	}
	//printk("Free %p\n", p_size);
	kfree((void *)p_size);
}
#endif

#define	MALLOC(_ptr, cast, _size, _type, _flags) \
	((_ptr) = (cast)ieee80211_malloc(_size, _flags))
#ifdef WLAN_MALLOC_FREE_TOT_DEBUG
# define	FREE(addr, type)	ieee80211_free((addr))
#else
# define	FREE(addr, type)	kfree((addr))
#endif

/*
 * This unlikely to be popular but it dramatically reduces diffs.
 */
#define printf(...) printk(__VA_ARGS__)
struct ieee80211com;
extern void if_printf(struct net_device *, const char *, ...);
extern const char *ether_sprintf(const u_int8_t *);

/*
 * Queue write-arounds and support routines.
 */
struct sk_buff *ieee80211_getdataframe(struct ieee80211vap *vap, uint8_t **frm, uint8_t qos,
					uint32_t payload_len);
struct sk_buff *ieee80211_getmgtframe(uint8_t **frm, uint32_t payload_len);
#define	IF_ENQUEUE(_q,_skb)	skb_queue_tail(_q,_skb)
#define	IF_DEQUEUE(_q,_skb)	(_skb = skb_dequeue(_q))
#define	_IF_QLEN(_q)		skb_queue_len(_q)
#define	IF_DRAIN(_q)		skb_queue_drain(_q)
void skb_queue_drain(struct sk_buff_head *q);

#ifndef __MOD_INC_USE_COUNT
#define	_MOD_INC_USE(_m, _err)						\
	if (!try_module_get(_m)) {					\
		printk(KERN_WARNING "%s: try_module_get failed\n",	\
			__func__); \
		_err;							\
	}
#define	_MOD_DEC_USE(_m)		module_put(_m)
#else
#define	_MOD_INC_USE(_m, _err)	MOD_INC_USE_COUNT
#define	_MOD_DEC_USE(_m)	MOD_DEC_USE_COUNT
#endif

#include <linux/delay.h>
#include <linux/jiffies.h>

#ifndef CLONE_KERNEL
/*
 * List of flags we want to share for kernel threads,
 * if only because they are not used by them anyway.
 */
#define CLONE_KERNEL	(CLONE_FS | CLONE_FILES | CLONE_SIGHAND)
#endif

#include <linux/mm.h>
#ifndef offset_in_page
#define	offset_in_page(p) ((unsigned long) (p) & ~PAGE_MASK)
#endif

#ifndef module_put_and_exit
#define module_put_and_exit(code) do {	\
	_MOD_DEC_USE(THIS_MODULE);	\
	do_exit(code);			\
} while (0)
#endif

/*
 * Linux uses __BIG_ENDIAN and __LITTLE_ENDIAN while BSD uses _foo
 * and an explicit _BYTE_ORDER.  Sorry, BSD got there first--define
 * things in the BSD way...
 */
#undef _LITTLE_ENDIAN
#define	_LITTLE_ENDIAN	1234	/* LSB first: i386, vax */
#undef _BIG_ENDIAN
#define	_BIG_ENDIAN	4321	/* MSB first: 68000, ibm, net */
#include <asm/byteorder.h>
#if defined(__LITTLE_ENDIAN)
#define	_BYTE_ORDER	_LITTLE_ENDIAN
#elif defined(__BIG_ENDIAN)
#define	_BYTE_ORDER	_BIG_ENDIAN
#else
#error "Please fix asm/byteorder.h"
#endif


/*
 * Deal with the sysctl handler api changing.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
#define	IEEE80211_SYSCTL_DECL(f, ctl, write, filp, buffer, lenp, ppos) \
	f(ctl_table *ctl, int write, \
	  void __user *buffer, size_t *lenp, loff_t *ppos)
#define	IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos) \
	proc_dointvec(ctl, write, buffer, lenp, ppos)
#define BIN_ATTR_ACCESS_DECL(f, filp, kobj, bin_attr, buf, offset, size) \
	ssize_t f(struct file *filp, struct kobject * kobj, struct bin_attribute * bin_attr, char * buf, loff_t offset, size_t size)
#else
#define	IEEE80211_SYSCTL_DECL(f, ctl, write, filp, buffer, lenp, ppos) \
	f(ctl_table *ctl, int write, struct file *filp, \
	  void __user *buffer, size_t *lenp, loff_t *ppos)
#define	IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos) \
	proc_dointvec(ctl, write, filp, buffer, lenp, ppos)
#define BIN_ATTR_ACCESS_DECL(f, filp, kobj, bin_attr, buf, offset, size) \
	ssize_t f(struct kobject * kobj, struct bin_attribute * bin_attr, char * buf, loff_t offset, size_t size)
#endif

void ieee80211_sysctl_vattach(struct ieee80211vap *);
void ieee80211_sysctl_vdetach(struct ieee80211vap *);
int ieee80211_proc_vcreate(struct ieee80211vap *, struct file_operations *,
	       char *);
void ieee80211_proc_cleanup(struct ieee80211vap *);

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define IEEE80211_VLAN_TAG_USED 1

#ifndef VLAN_GROUP_ARRAY_PART_LEN
#define vlan_group_set_device(group, vid, dev) do { \
	group->vlan_devices[vid] = dev; \
} while (0);
#endif

#else
#define IEEE80211_VLAN_TAG_USED 0
#endif
void ieee80211_vlan_vattach(struct ieee80211vap *);
void ieee80211_vlan_vdetach(struct ieee80211vap *);

void ieee80211_ioctl_vattach(struct ieee80211vap *);
void ieee80211_ioctl_vdetach(struct ieee80211vap *);
struct ifreq;
int ieee80211_ioctl_create_vap(struct ieee80211com *, struct ifreq *,
	struct net_device *);
int ieee80211_create_vap(struct ieee80211com *, char *, struct net_device *,
	int, int);

/* For explicitly logging messages */
void ieee80211_dot11_msg_send(struct ieee80211vap *vap,
		const char *mac_bssid,
		const char *message,
		const char *message_code,
		int message_reason,
		const char *message_description,
		const char *auth,
		const char *crypto);

extern char *d11_m[];
extern char *d11_c[];
extern char *d11_r[];

#endif /* _NET80211_IEEE80211_LINUX_H_ */
