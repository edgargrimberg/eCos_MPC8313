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
//
//      var_misc.c
//
//      HAL implementation miscellaneous functions
//
//==========================================================================

#include <pkgconf/hal.h>
#include <cyg/hal/hal_io.h>
#include <cyg/infra/cyg_ass.h>          // CYG_ASSERT
#define CYGARC_HAL_COMMON_EXPORT_CPU_MACROS
#include <cyg/hal/ppc_regs.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_mem.h>
#include <cyg/hal/mpc83xx.h>

// Newlib provides support for building the run-time elements of C++
// within the toolchain. Modern newlib stuff looks for a _impure_ptr
// entry.
void           *_impure_ptr;

//--------------------------------------------------------------------------
void hal_variant_init(void)
{
#ifndef CYGSEM_HAL_USE_ROM_MONITOR
    // Reset CPM
    _mpc83xx_reset_cpm();
#endif
}


//--------------------------------------------------------------------------
// Variant specific idle thread action.
bool
hal_variant_idle_thread_action( cyg_uint32 count )
{
    // Let architecture idle thread action run
    return true;
}

//---------------------------------------------------------------------------
// Use MMU resources to map memory regions.
// Takes and returns an int used to ID the MMU resource to use. This ID
// is increased as resources are used and should be used for subsequent
// invocations.
int
cyg_hal_map_memory (int id,CYG_ADDRESS virt, CYG_ADDRESS phys,
                    cyg_int32 size, cyg_uint8 flags)
{
    // Use BATs to map the memory.
    cyg_uint32 ubat, lbat;

    ubat = (virt & UBAT_BEPIMASK) | UBAT_VS | UBAT_VP;
    lbat = (phys & LBAT_BRPNMASK);
    if (flags & CYGARC_MEMDESC_CI)
        lbat |= LBAT_I;
    if (flags & CYGARC_MEMDESC_GUARDED)
        lbat |= LBAT_G;
    if (flags & CYGARC_MEMDESC_WRITE_THROUGH)
    	lbat |= LBAT_W;
    if (flags & CYGARC_MEMDESC_MEMORY_COHERENCE)
        	lbat |= LBAT_M;

//    if (flags & CYGARC_MEMDESC_RO) // Memory is Read Only
//        lbat |= LBAT_PP_RO;
//    if (flags & CYGARC_MEMDESC_RW) // Memory is RW
        lbat |= LBAT_PP_RW;

    // There are 8 BATs, size is programmable.
    // 8 BATs are specific to e300c3 core
    while (id < 8 && size > 0)
    {
        cyg_uint32 blk_size = 128*1024;
        cyg_uint32 bl = 0;
        while (blk_size < 256*1024*1024 && blk_size < size)
        {
            blk_size *= 2;
            bl = (bl << 1) | 1;
        }
        ubat = (ubat & ~UBAT_BLMASK) | (bl << 2);

        switch (id) {
        case 0:
            CYGARC_MTSPR (IBAT0U, ubat);
            CYGARC_MTSPR (IBAT0L, lbat);
            CYGARC_MTSPR (DBAT0U, ubat);
            CYGARC_MTSPR (DBAT0L, lbat);
            break;
        case 1:
            CYGARC_MTSPR (IBAT1U, ubat);
            CYGARC_MTSPR (IBAT1L, lbat);
            CYGARC_MTSPR (DBAT1U, ubat);
            CYGARC_MTSPR (DBAT1L, lbat);
            break;
        case 2:
            CYGARC_MTSPR (IBAT2U, ubat);
            CYGARC_MTSPR (IBAT2L, lbat);
            CYGARC_MTSPR (DBAT2U, ubat);
            CYGARC_MTSPR (DBAT2L, lbat);
            break;
        case 3:
            CYGARC_MTSPR (IBAT3U, ubat);
            CYGARC_MTSPR (IBAT3L, lbat);
            CYGARC_MTSPR (DBAT3U, ubat);
            CYGARC_MTSPR (DBAT3L, lbat);
            break;
        case 4:
        	CYGARC_MTSPR (HID2, 0x40000);
            CYGARC_MTSPR (IBAT4U, ubat);
            CYGARC_MTSPR (IBAT4L, lbat);
            CYGARC_MTSPR (DBAT4U, ubat);
            CYGARC_MTSPR (DBAT4L, lbat);
            break;
        case 5:
        	CYGARC_MTSPR (HID2, 0x40000);
            CYGARC_MTSPR (IBAT5U, ubat);
            CYGARC_MTSPR (IBAT5L, lbat);
            CYGARC_MTSPR (DBAT5U, ubat);
            CYGARC_MTSPR (DBAT5L, lbat);
            break;
        case 6:
        	CYGARC_MTSPR (HID2, 0x40000);
            CYGARC_MTSPR (IBAT6U, ubat);
            CYGARC_MTSPR (IBAT6L, lbat);
            CYGARC_MTSPR (DBAT6U, ubat);
            CYGARC_MTSPR (DBAT6L, lbat);
            break;
        case 7:
        	CYGARC_MTSPR (HID2, 0x40000);
            CYGARC_MTSPR (IBAT7U, ubat);
            CYGARC_MTSPR (IBAT7L, lbat);
            CYGARC_MTSPR (DBAT7U, ubat);
            CYGARC_MTSPR (DBAT7L, lbat);
            break;

        }

        size -= blk_size;
        id++;
    }

    return id;
}


// Initialize MMU to a sane (NOP) state.
void
cyg_hal_clear_MMU (void)
{
    cyg_uint32 ubat, lbat;

    // Initialize BATs with 0 -- VS&VP are unset, making all matches fail
    ubat = 0;
    lbat = 0;

    CYGARC_MTSPR (IBAT0U, ubat);
    CYGARC_MTSPR (IBAT0L, lbat);
    CYGARC_MTSPR (DBAT0U, ubat);
    CYGARC_MTSPR (DBAT0L, lbat);

    CYGARC_MTSPR (IBAT1U, ubat);
    CYGARC_MTSPR (IBAT1L, lbat);
    CYGARC_MTSPR (DBAT1U, ubat);
    CYGARC_MTSPR (DBAT1L, lbat);

    CYGARC_MTSPR (IBAT2U, ubat);
    CYGARC_MTSPR (IBAT2L, lbat);
    CYGARC_MTSPR (DBAT2U, ubat);
    CYGARC_MTSPR (DBAT2L, lbat);

    CYGARC_MTSPR (IBAT3U, ubat);
    CYGARC_MTSPR (IBAT3L, lbat);
    CYGARC_MTSPR (DBAT3U, ubat);
    CYGARC_MTSPR (DBAT3L, lbat);

    CYGARC_MTSPR (IBAT4U, ubat);
    CYGARC_MTSPR (IBAT4L, lbat);
    CYGARC_MTSPR (DBAT4U, ubat);
    CYGARC_MTSPR (DBAT4L, lbat);

    CYGARC_MTSPR (IBAT5U, ubat);
    CYGARC_MTSPR (IBAT5L, lbat);
    CYGARC_MTSPR (DBAT5U, ubat);
    CYGARC_MTSPR (DBAT5L, lbat);

    CYGARC_MTSPR (IBAT6U, ubat);
    CYGARC_MTSPR (IBAT6L, lbat);
    CYGARC_MTSPR (DBAT6U, ubat);
    CYGARC_MTSPR (DBAT6L, lbat);

    CYGARC_MTSPR (IBAT7U, ubat);
    CYGARC_MTSPR (IBAT7L, lbat);
    CYGARC_MTSPR (DBAT7U, ubat);
    CYGARC_MTSPR (DBAT7L, lbat);

}

// -------------------------------------------------------------------------
// Clock support


void hal_clock_initialize1(cyg_uint32 period)
{
//    CYG_ASSERT(period < 0x10000, "Invalid clock period");

    // Disable counter
    HAL_WRITE_UINT32(CYGARC_IMM_BASE+CYGARC_REG_IMM_PTCNR, 0x0);

    // Clear events
    HAL_WRITE_UINT32(CYGARC_IMM_BASE+CYGARC_REG_IMM_PTEVR, PTEVR_PIF);

    // Set prescaler to divide the clock by 1
    HAL_WRITE_UINT32(CYGARC_IMM_BASE+CYGARC_REG_IMM_PTPSR, 0x0);

    // Load the period into the PIT counter
    HAL_WRITE_UINT32(CYGARC_IMM_BASE+CYGARC_REG_IMM_PTLDR, period);

    // Start timer and enable interrupts,
    HAL_WRITE_UINT32(CYGARC_IMM_BASE+CYGARC_REG_IMM_PTCNR, PTCNR_CLEN | PTCNR_PIM);

}

void hal_clock_reset1(cyg_uint32 vector, cyg_uint32 period)
{

	hal_clock_initialize1(period);

}

void hal_clock_read1(cyg_uint32 *pvalue)
{
	CYG_ADDRESS timer = CYGARC_IMM_BASE + CYGARC_REG_IMM_PTCNR;
    cyg_uint32 val;

    HAL_READ_UINT32(timer+CYGARC_REG_IMM_PTCTR, val);
    *pvalue = val;
}



#ifdef CYGPKG_PROFILE_GPROF
//--------------------------------------------------------------------------
//
// Profiling support - uses a separate high-speed timer
//

#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/profile/profile.h>

// Can't rely on Cyg_Interrupt class being defined.
#define Cyg_InterruptHANDLED 1


static cyg_uint32  profile_period  = 0;

// Profiling timer ISR
static cyg_uint32
profile_isr(CYG_ADDRWORD vector, CYG_ADDRWORD data, HAL_SavedRegisters *regs)
{
	hal_clock_reset1(CYGNUM_HAL_INTERRUPT_PIT, profile_period);
    HAL_INTERRUPT_ACKNOWLEDGE(CYGNUM_HAL_INTERRUPT_PIT);

	__profile_hit(regs->pc);

    return Cyg_InterruptHANDLED;
}

int
hal_enable_profile_timer(int resolution)
{
    // Run periodic timer interrupt for profile
    // The resolution is specified in us


    // calculate # of ticks for the resolution which is given in us.
    profile_period =  ((cyg_uint32)resolution * CYGNUM_HAL_RTC_PERIOD) / 10000;

    // Attach ISR.
    HAL_INTERRUPT_ATTACH(CYGNUM_HAL_INTERRUPT_PIT, &profile_isr, 0x1111, 0);
    HAL_INTERRUPT_UNMASK(CYGNUM_HAL_INTERRUPT_PIT);

    // Set period and enable timer interrupts
    hal_clock_initialize1(profile_period);

    return resolution;
}
#endif



//--------------------------------------------------------------------------
// End of var_misc.c
