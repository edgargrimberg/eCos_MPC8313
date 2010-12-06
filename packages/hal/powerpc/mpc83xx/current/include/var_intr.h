#ifndef CYGONCE_VAR_INTR_H
#define CYGONCE_VAR_INTR_H


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
//
//      var_intr.h
//
//      Variant HAL interrupt and clock support
//
//=============================================================================

#include <pkgconf/hal.h>
#include <cyg/hal/plf_intr.h>
#include <cyg/hal/mpc83xx.h>            // Memory map
#include <cyg/infra/cyg_type.h>         // types
#include <cyg/hal/hal_io.h>             // io macros
#include <cyg/infra/cyg_ass.h>          // CYG_FAIL
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_intr.h>

// Interrupts
#define CYGNUM_HAL_INTERRUPT_ERROR          0

#define CYGNUM_HAL_INTERRUPT_UART1			9   //0b000_1001
#define CYGNUM_HAL_INTERRUPT_UART2 			10  //0b000_1010
#define CYGNUM_HAL_INTERRUPT_SEC 			11  //0b000_1011
#define CYGNUM_HAL_INTERRUPT_eTSEC1_TIMER	12  //0b000_1100
#define CYGNUM_HAL_INTERRUPT_eTSEC2_TIMER	13  //0b000_1101
#define CYGNUM_HAL_INTERRUPT_I2C1 			14  //0b000_1110
#define CYGNUM_HAL_INTERRUPT_I2C2 			15  //0b000_1111
#define CYGNUM_HAL_INTERRUPT_SPI 			16  //0b001_0000
#define CYGNUM_HAL_INTERRUPT_IRQ1 			17  //0b001_0001
#define CYGNUM_HAL_INTERRUPT_IRQ2 			18  //0b001_0010
#define CYGNUM_HAL_INTERRUPT_IRQ3 			19  //0b001_0011
#define CYGNUM_HAL_INTERRUPT_IRQ4 			20  //0b001_0100


#ifdef CYGPKG_HAL_POWERPC_MPC8313_SILICON_VERSION_1
/* Swapped TSEC1 with TSEC2 interrupt vectors
 * read errata IPIC1 from http://www.freescale.com/files/32bit/doc/errata/MPC8313ECE.pdf
 */
#	define CYGNUM_HAL_INTERRUPT_TSEC2_Err 		32  //0b010_0000
#	define CYGNUM_HAL_INTERRUPT_TSEC2_Rx 		33  //0b010_0001
#	define CYGNUM_HAL_INTERRUPT_TSEC2_TX 		34  //0b010_0010
#	define CYGNUM_HAL_INTERRUPT_TSEC1_Err 		35  //0b010_0011
#	define CYGNUM_HAL_INTERRUPT_TSEC1_Rx 		36  //0b010_0100
#	define CYGNUM_HAL_INTERRUPT_TSEC1_Tx 		37  //0b010_0101
#else
#	define CYGNUM_HAL_INTERRUPT_TSEC1_Tx 		32  //0b010_0000
#	define CYGNUM_HAL_INTERRUPT_TSEC1_Rx 		33  //0b010_0001
#	define CYGNUM_HAL_INTERRUPT_TSEC1_Err 		34  //0b010_0010
#	define CYGNUM_HAL_INTERRUPT_TSEC2_TX 		35  //0b010_0011
#	define CYGNUM_HAL_INTERRUPT_TSEC2_Rx 		36  //0b010_0100
#	define CYGNUM_HAL_INTERRUPT_TSEC2_Err 		37  //0b010_0101
#endif

#define CYGNUM_HAL_INTERRUPT_USB_DR 		38  //0b010_0110
#define CYGNUM_HAL_INTERRUPT_IRQ0 			48  //0b011_0000
#define CYGNUM_HAL_INTERRUPT_RTC_SEC 		64  //0b100_0000
#define CYGNUM_HAL_INTERRUPT_PIT 			65  //0b100_0001
#define CYGNUM_HAL_INTERRUPT_PCI 			66  //0b100_0010
#define CYGNUM_HAL_INTERRUPT_RTC_ALR 		68  //0b100_0100
#define CYGNUM_HAL_INTERRUPT_MU 			69  //0b100_0101
#define CYGNUM_HAL_INTERRUPT_SBA 			70  //0b100_0110
#define CYGNUM_HAL_INTERRUPT_DMA 			71  //0b100_0111
#define CYGNUM_HAL_INTERRUPT_GTM4 			72  //0b100_1000
#define CYGNUM_HAL_INTERRUPT_GTM8 			73  //0b100_1001
#define CYGNUM_HAL_INTERRUPT_GPIO 			74  //0b100_1010
#define CYGNUM_HAL_INTERRUPT_DDR 			76  //0b100_1100
#define CYGNUM_HAL_INTERRUPT_LBC 			77  //0b100_1101
#define CYGNUM_HAL_INTERRUPT_GTM2 			78  //0b100_1110
#define CYGNUM_HAL_INTERRUPT_GTM6 			79  //0b100_1111
#define CYGNUM_HAL_INTERRUPT_PMC 			80  //0b101_0000
#define CYGNUM_HAL_INTERRUPT_GTM3 			84  //0b101_0100
#define CYGNUM_HAL_INTERRUPT_GTM7 			85  //0b101_0101
#define CYGNUM_HAL_INTERRUPT_GTM1 			90  //0b101_1010
#define CYGNUM_HAL_INTERRUPT_GTM5 			91  //0b101_1011

//use RTC instead of decrementer
#define CYGNUM_HAL_INTERRUPT_RTC 0

#define CYGNUM_HAL_ISR_MAX                   CYGNUM_HAL_INTERRUPT_GTM5

//--------------------------------------------------------------------------
// Interrupt controller access

#ifndef CYGHWR_HAL_INTERRUPT_CONTROLLER_ACCESS_DEFINED
#ifdef CYGPKG_HAL_POWERPC_MPC83XX
static __inline__ void cyg_hal_interrupt_mask(cyg_uint32 vector)
{
	volatile cyg_uint32 *reg_simr_h = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SIMSR_H);
	volatile cyg_uint32 *reg_simr_l = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SIMSR_L);
	volatile cyg_uint32 *reg_semsr = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SEMSR);

	switch (vector)
	{
#ifdef CYGPKG_HAL_POWERPC_MPC8313_SILICON_VERSION_1
	case CYGNUM_HAL_INTERRUPT_TSEC2_Err ... CYGNUM_HAL_INTERRUPT_USB_DR:
	*reg_simr_h &= ~((0x80000000) >> (vector
					- CYGNUM_HAL_INTERRUPT_TSEC2_Err));
			break;
#else
	case CYGNUM_HAL_INTERRUPT_TSEC1_Tx ... CYGNUM_HAL_INTERRUPT_USB_DR:
	*reg_simr_h &= ~((0x80000000) >> (vector
					- CYGNUM_HAL_INTERRUPT_TSEC1_Tx));
			break;
#endif


	case CYGNUM_HAL_INTERRUPT_UART1 ... CYGNUM_HAL_INTERRUPT_SPI:
		*reg_simr_h &= ~((0x00000001) << (CYGNUM_HAL_INTERRUPT_SPI - vector));
		break;

	case CYGNUM_HAL_INTERRUPT_RTC_SEC ... CYGNUM_HAL_INTERRUPT_PCI:
		*reg_simr_l &= ~((0x80000000)
				>> (vector - CYGNUM_HAL_INTERRUPT_RTC_SEC));
		break;

	case CYGNUM_HAL_INTERRUPT_RTC_ALR ... CYGNUM_HAL_INTERRUPT_GPIO:
		*reg_simr_l &= ~((0x08000000)
				>> (vector - CYGNUM_HAL_INTERRUPT_RTC_ALR));
		break;

	case CYGNUM_HAL_INTERRUPT_DDR ... CYGNUM_HAL_INTERRUPT_PMC:
		*reg_simr_l &= ~((0x00080000) >> (vector - CYGNUM_HAL_INTERRUPT_DDR));
		break;

	case CYGNUM_HAL_INTERRUPT_GTM3 ... CYGNUM_HAL_INTERRUPT_GTM7:
		*reg_simr_l &= ~((0x00000800) >> (vector - CYGNUM_HAL_INTERRUPT_GTM3));
		break;

	case CYGNUM_HAL_INTERRUPT_GTM1 ... CYGNUM_HAL_INTERRUPT_GTM5:
		*reg_simr_l &= ~((0x00000020) >> (vector - CYGNUM_HAL_INTERRUPT_GTM1));
		break;
	case 0:
		//This is the unmaskable decrementer
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ0:
		*reg_semsr &= ~(0x80000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ1:
		*reg_semsr &= ~(0x40000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ2:
		*reg_semsr &= ~(0x20000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ3:
		*reg_semsr &= ~(0x10000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ4:
		*reg_semsr &= ~(0x08000000);
		break;
	default:
		CYG_FAIL("Unknown Interrupt in mask !!!");
		break;
	}

}

static __inline__ void cyg_hal_interrupt_unmask(cyg_uint32 vector)
{
	volatile cyg_uint32 *reg_simr_h = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SIMSR_H);
	volatile cyg_uint32 *reg_simr_l = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SIMSR_L);
	volatile cyg_uint32 *reg_semsr = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SEMSR);

	switch (vector)
	{

#ifdef CYGPKG_HAL_POWERPC_MPC8313_SILICON_VERSION_1
	case CYGNUM_HAL_INTERRUPT_TSEC2_Err ... CYGNUM_HAL_INTERRUPT_USB_DR:
	*reg_simr_h |= ((0x80000000) >> (vector
			- CYGNUM_HAL_INTERRUPT_TSEC2_Err));
#else
	case CYGNUM_HAL_INTERRUPT_TSEC1_Tx ... CYGNUM_HAL_INTERRUPT_USB_DR:
	*reg_simr_h |= ((0x80000000) >> (vector
			- CYGNUM_HAL_INTERRUPT_TSEC1_Tx));

#endif
		break;

	case CYGNUM_HAL_INTERRUPT_UART1 ... CYGNUM_HAL_INTERRUPT_SPI:
		*reg_simr_h |= ((0x00000001) << (CYGNUM_HAL_INTERRUPT_SPI - vector));
		break;

	case CYGNUM_HAL_INTERRUPT_RTC_SEC ... CYGNUM_HAL_INTERRUPT_PCI:
		*reg_simr_l
				|= ((0x80000000) >> (vector - CYGNUM_HAL_INTERRUPT_RTC_SEC));
		break;

	case CYGNUM_HAL_INTERRUPT_RTC_ALR ... CYGNUM_HAL_INTERRUPT_GPIO:
		*reg_simr_l
				|= ((0x08000000) >> (vector - CYGNUM_HAL_INTERRUPT_RTC_ALR));
		break;

	case CYGNUM_HAL_INTERRUPT_DDR ... CYGNUM_HAL_INTERRUPT_PMC:
		*reg_simr_l |= ((0x00080000) >> (vector - CYGNUM_HAL_INTERRUPT_DDR));
		break;

	case CYGNUM_HAL_INTERRUPT_GTM3 ... CYGNUM_HAL_INTERRUPT_GTM7:
		*reg_simr_l |= ((0x00000800) >> (vector - CYGNUM_HAL_INTERRUPT_GTM3));
		break;

	case CYGNUM_HAL_INTERRUPT_GTM1 ... CYGNUM_HAL_INTERRUPT_GTM5:
		*reg_simr_l |= ((0x00000020) >> (vector - CYGNUM_HAL_INTERRUPT_GTM1));
		break;
	case 0:
		//This is the unmaskable decrementer
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ0:
		*reg_semsr |= (0x80000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ1:
		*reg_semsr |= (0x40000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ2:
		*reg_semsr |= (0x20000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ3:
		*reg_semsr |= (0x10000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ4:
		*reg_semsr |= (0x08000000);
		break;

	default:
		CYG_FAIL("Unknown Interrupt in unmask !!!");
		break;
	}

}

static __inline__ void cyg_hal_interrupt_acknowledge(cyg_uint32 vector)
{

	volatile cyg_uint32 *reg_sipnr_h = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SIPNR_H);
	volatile cyg_uint32 *reg_sipnr_l = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SIPNR_L);
	volatile cyg_uint32 *reg_sepnr = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SEPNR);

	switch (vector)
	{

#ifdef CYGPKG_HAL_POWERPC_MPC8313_SILICON_VERSION_1
	case CYGNUM_HAL_INTERRUPT_TSEC2_Err ... CYGNUM_HAL_INTERRUPT_USB_DR:
	*reg_sipnr_h |= ((0x80000000) >> (vector
			- CYGNUM_HAL_INTERRUPT_TSEC2_Err));
#else
	case CYGNUM_HAL_INTERRUPT_TSEC1_Tx ... CYGNUM_HAL_INTERRUPT_USB_DR:
	*reg_sipnr_h |= ((0x80000000) >> (vector
			- CYGNUM_HAL_INTERRUPT_TSEC1_Tx));
#endif
		break;

	case CYGNUM_HAL_INTERRUPT_UART1 ... CYGNUM_HAL_INTERRUPT_SPI:
		*reg_sipnr_h |= ((0x00000001) << (CYGNUM_HAL_INTERRUPT_SPI - vector));
		break;

	case CYGNUM_HAL_INTERRUPT_RTC_SEC ... CYGNUM_HAL_INTERRUPT_PCI:
		*reg_sipnr_l |= ((0x80000000)
				>> (vector - CYGNUM_HAL_INTERRUPT_RTC_SEC));
		break;

	case CYGNUM_HAL_INTERRUPT_RTC_ALR ... CYGNUM_HAL_INTERRUPT_GPIO:
		*reg_sipnr_l |= ((0x08000000)
				>> (vector - CYGNUM_HAL_INTERRUPT_RTC_ALR));
		break;

	case CYGNUM_HAL_INTERRUPT_DDR ... CYGNUM_HAL_INTERRUPT_PMC:
		*reg_sipnr_l |= ((0x00080000) >> (vector - CYGNUM_HAL_INTERRUPT_DDR));
		break;

	case CYGNUM_HAL_INTERRUPT_GTM3 ... CYGNUM_HAL_INTERRUPT_GTM7:
		*reg_sipnr_l |= ((0x00000800) >> (vector - CYGNUM_HAL_INTERRUPT_GTM3));
		break;

	case CYGNUM_HAL_INTERRUPT_GTM1 ... CYGNUM_HAL_INTERRUPT_GTM5:
		*reg_sipnr_l |= ((0x00000020) >> (vector - CYGNUM_HAL_INTERRUPT_GTM1));
		break;
	case 0:
		//This is the unmaskable decrementer
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ0:
		*reg_sepnr &= ~(0x80000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ1:
		*reg_sepnr &= ~(0x40000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ2:
		*reg_sepnr &= ~(0x20000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ3:
		*reg_sepnr &= ~(0x10000000);
		break;
	case CYGNUM_HAL_INTERRUPT_IRQ4:
		*reg_sepnr &= ~(0x08000000);
		break;

	default:
		CYG_FAIL("Unknown Interrupt in ack !!!");
		break;
	}

}

static __inline__ void cyg_hal_interrupt_configure(cyg_uint32 vector,
		cyg_bool level, cyg_bool up)
{
	cyg_uint32 siel, bit;
	volatile cyg_uint32 *reg_secnr = (volatile cyg_uint32 *) (CYGARC_IMM_BASE
			+ CYGARC_REG_IMM_INTR_SECNR);

    switch (vector) {
    case CYGNUM_HAL_INTERRUPT_IRQ0:
		CYG_ASSERT( level || !up, "Only falling edge is supported");

		bit = 0x00008000;

		siel = *reg_secnr;
		siel &= ~bit;
		if (!level) {
			// Set edge detect bit.
			siel |= bit;
		}
		*reg_secnr = siel;
		break;

	case CYGNUM_HAL_INTERRUPT_IRQ1:
	case CYGNUM_HAL_INTERRUPT_IRQ2:
	case CYGNUM_HAL_INTERRUPT_IRQ3:
	case CYGNUM_HAL_INTERRUPT_IRQ4:

		CYG_ASSERT( level || !up, "Only falling edge is supported");

		bit = (cyg_uint32) (0x80000000 >> (vector));
		siel = *reg_secnr;
		siel &= ~bit;
		if (!level) {
			// Set edge detect bit.
			siel |= bit;
		}
		*reg_secnr = siel;
		break;

	default:
		CYG_FAIL("Interrupt # not implemented in cyg_hal_interrupt_configure !!!");
	        break;
	}

}

static __inline__ void cyg_hal_interrupt_set_level(cyg_uint32 vector,
		cyg_uint32 level)
{

	// NOT IMPLEMENTED  ....
	// FACT : USER should not program the same interrupt to more than
	// one priority position.
	// FACT : Every interrupt has an assigned default priority.

	// PROBLEM : One has to find the previous priority of the given vector
	// and swap(?) it with the requested priority owner (Not nice because
	// it changes another interrupt's priority inadvertently)

}

#define HAL_INTERRUPT_MASK( _vector_ )                    \
    CYG_MACRO_START                                       \
        cyg_hal_interrupt_mask ( (_vector_) );            \
    CYG_MACRO_END

#define HAL_INTERRUPT_UNMASK( _vector_ )                  \
    CYG_MACRO_START                                       \
        cyg_hal_interrupt_unmask ( (_vector_) );          \
    CYG_MACRO_END

#define HAL_INTERRUPT_ACKNOWLEDGE( _vector_ )             \
    CYG_MACRO_START                                       \
        cyg_hal_interrupt_acknowledge ( (_vector_) );     \
    CYG_MACRO_END

#define HAL_INTERRUPT_CONFIGURE( _vector_, _level_, _up_ )              \
    CYG_MACRO_START                                                     \
        cyg_hal_interrupt_configure ( (_vector_), (_level_), (_up_) );  \
    CYG_MACRO_END

#define HAL_INTERRUPT_SET_LEVEL( _vector_, _level_ )            \
    CYG_MACRO_START                                             \
        cyg_hal_interrupt_set_level ( (_vector_) , (_level_) ); \
    CYG_MACRO_END

#define CYGHWR_HAL_INTERRUPT_CONTROLLER_ACCESS_DEFINED

#endif
#endif

//--------------------------------------------------------------------------
// Clock control

#define CYGHWR_HAL_CLOCK_DEFINED 1
// Note: variant or platform allowed to override these definitions

#define HAL_CLOCK_INITIALIZE( _period_ )        \
    CYG_MACRO_START                             \
    asm volatile (                              \
        "mtdec %0;"                             \
        :                                       \
        : "r"(_period_)                         \
        );                                      \
    CYG_MACRO_END

#define HAL_CLOCK_RESET( _vector_, _period_ )   HAL_CLOCK_INITIALIZE( _period_ )

#define HAL_CLOCK_READ( _pvalue_ )                              \
    CYG_MACRO_START                                             \
    register cyg_uint32 result;                                 \
    asm volatile(                                               \
        "mfdec  %0;"                                            \
        : "=r"(result)                                          \
        );                                                      \
    *(_pvalue_) = CYGNUM_KERNEL_COUNTERS_RTC_PERIOD-result;     \
    CYG_MACRO_END

#ifdef CYGVAR_KERNEL_COUNTERS_CLOCK_LATENCY
#define HAL_CLOCK_LATENCY( _pvalue_ )                           \
    CYG_MACRO_START                                             \
    register cyg_int32 result;                                  \
    asm volatile(                                               \
        "mfdec  %0;"                                            \
        : "=r"(result)                                          \
        );                                                      \
    /* Pending DEC interrupts cannot be discarded. If dec is */ \
    /* positive it''s because a DEC interrupt occured while  */ \
    /* eCos was getting ready to run. Just return 0 in that  */ \
    /* case.                                                 */ \
    if (result > 0)                                             \
        result = 0;                                             \
    *(_pvalue_) = -result;                                      \
    CYG_MACRO_END
#endif

#ifndef HAL_DELAY_US
externC void hal_delay_us(int);
#define HAL_DELAY_US(n) hal_delay_us(n)
#endif

// The vector used by the Real time clock
#ifndef CYGNUM_HAL_INTERRUPT_RTC
#define CYGNUM_HAL_INTERRUPT_RTC             CYGNUM_HAL_INTERRUPT_DECREMENTER
#endif // CYGNUM_HAL_INTERRUPT_RTC
//-----------------------------------------------------------------------------
#endif // ifndef CYGONCE_VAR_INTR_H
// End of var_intr.h
