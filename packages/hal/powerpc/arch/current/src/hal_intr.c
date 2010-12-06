//==========================================================================
//
//      hal_intr.c
//
//      PowerPC interrupt handlers
//
//==========================================================================
///// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003 Free Software Foundation, Inc.
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later
// version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License
// along with eCos; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// As a special exception, if other files instantiate templates or use
// macros or inline functions from this file, or you compile this file
// and link it with other works to produce a work based on this file,
// this file does not by itself cause the resulting work to be covered by
// the GNU General Public License. However the source code for this file
// must still be made available in accordance with section (3) of the GNU
// General Public License v2.
//
// This exception does not invalidate any other reasons why a work based
// on this file might be covered by the GNU General Public License.
// -------------------------------------------
// ####ECOSGPLCOPYRIGHTEND####
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    jskov
// Contributors: jskov
// Date:         1999-02-20
// Purpose:      PowerPC interrupt handlers
// Description:  This file contains code to handle interrupt related issues
//               on the PowerPC.
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>

#include <cyg/hal/hal_intr.h>

static unsigned long ticks_per_us;

externC void
hal_IRQ_init(void)
{
    // No architecture general initialization, but the variant may have
    // provided some.
    hal_variant_IRQ_init();

    // Initialize real-time clock (for delays, etc, even if kernel doesn't use it)
    HAL_CLOCK_INITIALIZE(CYGNUM_HAL_RTC_PERIOD);

    // Pre-calculate this factor to avoid the extra calculations on each delay
    ticks_per_us = ((long long)1 * (CYGNUM_HAL_RTC_PERIOD )) / 10000;
}

// Delay for some number of useconds.
externC void
hal_delay_us(int us)
{
	cyg_uint32 val_u, val_l;
    volatile cyg_uint64 old_dec, new_dec;
    long long ticks;
    long long diff;

    // Scale the desired number of microseconds to be a number of decrementer ticks
	ticks = us * ticks_per_us;

	asm volatile("mftbl %0;" : "=r"(val_l) : );
	asm volatile("mftbu %0;" : "=r"(val_u) : );
	old_dec = (((cyg_uint64)val_u) << 32) | (cyg_uint64)(val_l);

    do
    {
		asm volatile("mftbl %0;" : "=r"(val_l) : );
		asm volatile("mftbu %0;" : "=r"(val_u) : );
		new_dec = (((cyg_uint64)val_u) << 32) | (cyg_uint64)(val_l);

        diff = (new_dec - old_dec);
    }while(diff < ticks);
}

// -------------------------------------------------------------------------
// EOF hal_intr.c
