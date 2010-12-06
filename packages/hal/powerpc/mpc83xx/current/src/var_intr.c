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
//      var_intr.c
//
//      PowerPC variant interrupt handlers
//
//==========================================================================

#define CYGARC_HAL_COMMON_EXPORT_CPU_MACROS

#include <pkgconf/hal.h>
#include <cyg/infra/cyg_type.h>

#include <cyg/hal/hal_mem.h>            // HAL memory definitions
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/mpc83xx.h>            // For IMM structures
#include <cyg/hal/hal_if.h>
#include <cyg/hal/var_intr.h>

//#===========================================================================
//# MSR initialization value
//# zero all bits except:
//# FP = floating point available
//# ME = machine check enabled
//# IP = vectors at 0xFFFxxxxx (ROM startup only)
//# IR = instruction address translation
//# DR = data address translation
//# RI = recoverable interrupt

#define CYG_MSR_COMMON (MSR_FP | MSR_ME | MSR_RI)

#if (CYGHWR_HAL_POWERPC_VECTOR_BASE == 0xfff00000)
# define IP_BIT MSR_IP
#else
# define IP_BIT 0
#endif

#ifdef CYGHWR_HAL_POWERPC_ENABLE_MMU
# define IR_DR_BITS (MSR_IR | MSR_DR)
#else
# define IR_DR_BITS 0
#endif

#define CYG_MSR (CYG_MSR_COMMON | IP_BIT | IR_DR_BITS)

extern  void
hal_variant_IRQ_init(void)
{
    // Clear any pending interrupts & reset masks

	HAL_WRITE_UINT32(CYGARC_IMM_BASE + CYGARC_REG_IMM_INTR_SIMSR_H, 0);
	HAL_WRITE_UINT32(CYGARC_IMM_BASE + CYGARC_REG_IMM_INTR_SIMSR_L, 0);
	HAL_WRITE_UINT32(CYGARC_IMM_BASE + CYGARC_REG_IMM_INTR_SIPNR_H, 0xFFFFFFFF);
	HAL_WRITE_UINT32(CYGARC_IMM_BASE + CYGARC_REG_IMM_INTR_SIPNR_L, 0xFFFFFFFF);
}

// -------------------------------------------------------------------------
// EOF var_intr.c
