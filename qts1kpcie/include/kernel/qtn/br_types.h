/*
 * Linux ethernet bridge; shared types with Quantenna FWT
 *
 * (C) Copyright 2013 Quantenna Communications Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#ifndef _BR_TYPES_H
#define _BR_TYPES_H

#include <linux/netdevice.h>
#include <linux/if_bridge.h>

struct br_ip
{
	union {
		__be32	ip4;
#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
		struct in6_addr ip6;
#endif
	} u;
	__be16		proto;
};

#endif
