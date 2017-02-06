/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2013 Quantenna Communications, Inc                  **
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

#ifndef __TOPAZ_BUSMON_H
#define __TOPAZ_BUSMON_H

#define TOPAZ_BUSMON_MAX_RANGES	4

struct topaz_busmon_range {
	uintptr_t start;
	uintptr_t end;
};

void topaz_busmon_range_check(uint8_t bus,
                              const struct topaz_busmon_range *range,
                              size_t nranges, bool outside);

static inline void topaz_busmon_range_check_disable(uint8_t bus)
{
	topaz_busmon_range_check(bus, NULL, 0, 0);
}

void topaz_busmon_timeout(uint8_t bus, uint16_t timeout, bool enable);

static inline void topaz_busmon_timeout_en(uint8_t bus, uint16_t timeout)
{
	topaz_busmon_timeout(bus, timeout, 1);
}

static inline void topaz_busmon_timeout_dis(uint8_t bus)
{
	topaz_busmon_timeout(bus, 0, 0);
}

#endif /* __TOPAZ_BUSMON_H */

