/*
 * Copyright (c) 2014 Quantenna Communications, Inc.
 */

#ifndef _MUC_SHARE_DEF_H_
#define _MUUC_SHARE_DEF_H_

#include "../common/ruby_mem.h"

#define QTN_FW_WMAC_RX_Q_MGMT		0
#define QTN_FW_WMAC_RX_Q_CTRL		1
#define QTN_FW_WMAC_RX_Q_DATA		2
#define QTN_FW_WMAC_RX_QNUM		3
#define QTN_FW_WMAC_RX_QDEEP_MGMT	8
#define QTN_FW_WMAC_RX_QDEEP_CTRL	8
#define QTN_FW_WMAC_RX_QDEEP_DATA	280
#define QTN_FW_WMAC_RX_DESC_NUM	(QTN_FW_WMAC_RX_QDEEP_MGMT + \
	QTN_FW_WMAC_RX_QDEEP_CTRL + QTN_FW_WMAC_RX_QDEEP_DATA)

#endif // #ifndef _MUC_SHARE_DEF_H_

