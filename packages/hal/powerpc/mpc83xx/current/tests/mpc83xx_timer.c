//==========================================================================
//
//        mpc83xx_timer.c
//
//        PowerPC MPC83xx timer tests
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
// Author(s):     gthomas
// Contributors:  gthomas
// Date:          2003-11-19
//####DESCRIPTIONEND####

#include <pkgconf/hal.h>
#include <cyg/hal/hal_io.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/testcase.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/mpc83xx.h>

#ifdef CYGPKG_KERNEL
#include <cyg/kernel/kapi.h>

#define STACK_SIZE CYGNUM_HAL_STACK_SIZE_MINIMUM
static char main_thread_stack[STACK_SIZE];
static cyg_thread main_thread_thread_data;
static cyg_handle_t main_thread_thread_handle;

static cyg_vector_t  interrupt;             // Interrupt vector used by controller
static cyg_handle_t  interrupt_handle;
static cyg_interrupt interrupt_object;

static volatile int intr_count;

static void
main_thread(cyg_addrword_t param)
{
    int tries = 0;
    int old_intr_count;
    int hits = 0;

    while (++tries <= 10000) {
        old_intr_count = intr_count;
        cyg_thread_delay(100);
        diag_printf("tick - count = %d\n", intr_count);
        if (intr_count != old_intr_count) hits++;
    }
    if (hits == (tries-1)) {
        CYG_TEST_PASS("mpc83xx_timer OK");
    } else {
        diag_printf("tries = %d, hits = %d\n", tries, hits);
        CYG_TEST_FAIL("mpc83xx_timer unreliable");
    }
    CYG_TEST_EXIT("mpc83xx_timer");
}

// This ISR is called when the timer interrupt occurs
static int
timer_isr(cyg_vector_t vector, cyg_addrword_t data, HAL_SavedRegisters *regs)
{
    cyg_interrupt_mask(interrupt);
    return (CYG_ISR_HANDLED|CYG_ISR_CALL_DSR);  // Run the DSR
}

// This DSR is called when the timer interrupt occurs
static void
timer_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    intr_count++;
    switch( *((char *)data) )
    {
    case 1:
    	HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTEVR1, 0x0003);
    	break;
    case 2:
    	HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTEVR2, 0x0003);
    	break;
    case 3:
    	HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTEVR3, 0x0003);
    	break;
    case 4:
    	HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTEVR4, 0x0003);
    	break;
    default:
    	//invalid timer no
    	break;
    }
    cyg_interrupt_acknowledge(interrupt);
    cyg_interrupt_unmask(interrupt);
}

#ifdef CYGSEM_HAL_STOP_CONSTRUCTORS_ON_FLAG
externC void cyg_hal_invoke_constructors();
#endif

externC void
cyg_user_start( void )
{
	unsigned char timer_no = 1;
#ifdef CYGSEM_HAL_STOP_CONSTRUCTORS_ON_FLAG
    cyg_hal_invoke_constructors();
#endif
    CYG_TEST_INIT();
    CYG_TEST_INFO("cyg_user_start()");
    cyg_thread_create(1,                             // Priority
                      main_thread     ,              // entry
                      (cyg_addrword_t)0,             // entry parameter
                      "CS8900 int",                  // Name
                      &main_thread_stack[0],         // Stack
                      STACK_SIZE,                    // Size
                      &main_thread_thread_handle,    // Handle
                      &main_thread_thread_data       // Thread data structure
            );
    cyg_thread_resume(main_thread_thread_handle);    // Start it

    interrupt = CYGNUM_HAL_INTERRUPT_GTM1;
    cyg_interrupt_create(interrupt,
                             0,								// Priority - what goes here?
                             (cyg_addrword_t)(&timer_no),	//  Data item passed to interrupt handler
                             (cyg_ISR_t *)timer_isr,
                             (cyg_DSR_t *)timer_dsr,
                             &interrupt_handle,
                             &interrupt_object);
    cyg_interrupt_attach(interrupt_handle);
    cyg_interrupt_acknowledge(interrupt);
    cyg_interrupt_unmask(interrupt);
    cyg_interrupt_enable();



    // Set up timer1, module 1
    //reset
    HAL_WRITE_UINT8( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTCFR1, 0);
    //prescale = 1
    HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTPSR1, 0);
    HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTMDR1, GTMDR_ICLK_INT_CLK16 | GTMDR_ORI);
    HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTEVR1, 0x0003);
    HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTRFR1, 0x2000);  // Reference value
    HAL_WRITE_UINT16( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTCNR1, 0x0000);
    //enable
    HAL_WRITE_UINT8( CYGARC_IMM_BASE + CYGARC_REG_IMM_GTM1_GTCFR1 , GTCFR_RST1);  // Reset & enable timer1
    cyg_scheduler_start();
    CYG_TEST_PASS("mpc83xx_timer");
}

// -------------------------------------------------------------------------

#else  // ! CYGPKG_KERNEL
#define N_A_MSG "no kernel"
#endif // CYGPKG_KERNEL

#ifdef N_A_MSG
externC void
cyg_start( void )
{
    CYG_TEST_INIT();
    CYG_TEST_NA( N_A_MSG );
}
#endif // N_A_MSG defined ie. we are N/A.
