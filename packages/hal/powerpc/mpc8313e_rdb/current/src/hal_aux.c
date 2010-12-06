//=============================================================================
//
//      hal_aux.c
//
//      HAL auxiliary objects and code; per platform
//
//=============================================================================
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   hmt
// Contributors:hmt, gthomas
// Date:        1999-06-08
// Purpose:     HAL aux objects: startup tables.
// Description: Tables for per-platform initialization
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <pkgconf/hal.h>
#include <cyg/hal/hal_mem.h>            // HAL memory definitions
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/mpc83xx.h>            // For IMM structures
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/kernel/kapi.h>


unsigned long long sec_tick;

// FIXME

static __inline__ unsigned long
_le32(unsigned long val)
{
    return (((val & 0x000000FF) << 24) |
            ((val & 0x0000FF00) <<  8) |
            ((val & 0x00FF0000) >>  8) |
            ((val & 0xFF000000) >> 24));
}

static __inline__ unsigned short
_le16(unsigned short val)
{
    return (((val & 0x000000FF) << 8) |
            ((val & 0x0000FF00) >> 8));
}

#define HAL_WRITE_UINT32LE(_addr_, _val_) \
  HAL_WRITE_UINT32(_addr_, _le32(_val_))
#define HAL_WRITE_UINT16LE(_addr_, _val_) \
  HAL_WRITE_UINT16(_addr_, _le16(_val_))
#define HAL_WRITE_UINT8LE(_addr_, _val_) \
  HAL_WRITE_UINT8(_addr_, _val_)
#define HAL_READ_UINT32LE(_addr_, _val_)        \
  {                                             \
      HAL_READ_UINT32(_addr_, _val_);           \
      _val_ = _le32(_val_);                     \
  }
#define HAL_READ_UINT16LE(_addr_, _val_)        \
  {                                             \
      HAL_READ_UINT16(_addr_, _val_);           \
      _val_ = _le16(_val_);                     \
  }
#define HAL_READ_UINT8LE(_addr_, _val_)        \
  HAL_READ_UINT8(_addr_, _val_)

// FIXME

// The memory map is weakly defined, allowing the application to redefine
// it if necessary. The regions defined below are the minimum requirements.
CYGARC_MEMDESC_TABLE CYGBLD_ATTRIB_WEAK = {
	CYGARC_MEMDESC_CACHE(   0x00000000, 0x00800000),  // SDRAM
	CYGARC_MEMDESC_CACHE(   0x80000000, 0x00800000 ), // FLASH
    CYGARC_MEMDESC_NOCACHE( CYGARC_IMM_BASE, 0x00100000 ), // IMMR registers

    CYGARC_MEMDESC_TABLE_END
};

//--------------------------------------------------------------------------
// Platform init code.
void
hal_platform_init(void)
{
#ifndef CYGSEM_HAL_USE_ROM_MONITOR

	//configure the I/O pins
	cyg_uint32 state = 0;
	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_GP1DIR, state);
	HAL_WRITE_UINT32(CYGARC_IMM_BASE + CYGARC_REG_IMM_GP1DIR, state | 0x03);

    // Start up system I/O
    hal_if_init();
#endif // CYGSEM_HAL_USE_ROM_MONITOR
}

//
// Cause the platform to reset
//
void
mpc8313e_rdb_reset(void)
{
	volatile cyg_uint32 *swcrr = (volatile cyg_uint32 *)(CYGARC_IMM_BASE + CYGARC_REG_IMM_SWCRR);
	*swcrr &= 0x10006;
}

// ms to ticks.
#define CALC_TICKS(ms)  (((cyg_tick_count_t)(ms)) * (CYGNUM_HAL_RTC_PERIOD * 100)) / 1000000


/******************************* functions for RTC ***************************/
unsigned long rtc_zero(void)
{
	   sec_tick=0;
	   return (sec_tick);  /* read timer tick */
}
/*****************************************************************************/
unsigned long rtc_read(void)
{
	cyg_uint32 retVal = 0;
	HAL_READ_UINT32(CYGARC_IMM_BASE + CYGARC_REG_IMM_RTCNR, retVal);

	return retVal;  /* read rtc counter */
}
/*****************************************************************************/
/* rtc_read_sec : Read counter & convert to second                           */
/* the real time clock have no external crystal so we are using 25 MHZ clock */
/* (average is ca 24.75Mhz)as input.then this clock is divided by 512 * 9600 */
/* that is 5.0354 tick/sec                                                   */
/* RETURNS : second since last boot                                          */
/*****************************************************************************/
unsigned long rtc_read_sec(void)
{
	unsigned long long sec,ticks;

	ticks = cyg_current_time();
	sec_tick = ticks / 500;
    sec = sec_tick;

	return sec;
}

// EOF hal_aux.c
