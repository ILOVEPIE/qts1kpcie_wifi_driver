/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2012-2013 Quantenna Communications, Inc.            **
**                            All Rights Reserved                            **
**                                                                           **
**  File        : qvsp_ioctl.h                                               **
**  Description : Video Stream Protection                                    **
**                                                                           **
*******************************************************************************
EH0*/

#ifndef __QTN_QVSP_IOCTL_H__
#define __QTN_QVSP_IOCTL_H__

enum qvsp_ioctl {
	QVSP_IOCTL_MIN = 0x2000,
	QVSP_IOCTL_STATE_GET,
	QVSP_IOCTL_STATE_SET,
	QVSP_IOCTL_CFG_GET,
	QVSP_IOCTL_CFG_SET,
	QVSP_IOCTL_WL_ADD,
	QVSP_IOCTL_WL_DEL,
	QVSP_IOCTL_WL_DEL_INDEX,
	QVSP_IOCTL_WL_GETLIST,
	QVSP_IOCTL_RULE_ADD,
	QVSP_IOCTL_RULE_DEL,
	QVSP_IOCTL_RULE_DEL_INDEX,
	QVSP_IOCTL_RULE_GETLIST,
	QVSP_IOCTL_STRM_GETLIST,
	QVSP_IOCTL_STRM_GETLIST_ALL,
	QVSP_IOCTL_STATS_GET,
	QVSP_IOCTL_INACTIVE_FLAGS_GET,
	QVSP_IOCTL_MAX
};

struct qvsp_ioctl_get {
	unsigned int	index;
	void		*param;
	unsigned int	count;
};

struct qvsp_ioctl_set_cfg {
	unsigned int index;
	unsigned int value;
};

union qvsp_ioctl_set {
	unsigned int index;
	struct qvsp_ioctl_set_cfg	cfg;
	struct qvsp_wl_flds		wl;
	struct qvsp_rule_flds		rule;
};

#endif	/* __QTN_QVSP_IOCTL_H__ */

