#ifndef CYGONCE_HAL_VAR_REGS_H
#define CYGONCE_HAL_VAR_REGS_H

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
//      var_regs.h
//
//      PowerPC MPC83xx CPU definitions
//
//==========================================================================


//--------------------------------------------------------------------------
#define CYGARC_REG_LR   8              // Link Register
#define CYGARC_REG_CTR   9              // Counter Register

#define CYGARC_REG_DSISR  18
#define CYGARC_REG_DAR    19
#define CYGARC_REG_DEC    22
#define CYGARC_REG_SDR1   25

#define CYGARC_REG_TBL  268
#define CYGARC_REG_TBU  269

#define CYGARC_REG_SPRG0  272
#define CYGARC_REG_SPRG1  273
#define CYGARC_REG_SPRG2  274
#define CYGARC_REG_SPRG3  275
#define CYGARC_REG_EAR    282
#define CYGARC_REG_PVR    287

#define CYGARC_REG_IBAT0U          528
#define CYGARC_REG_IBAT0L          529
#define CYGARC_REG_IBAT1U          530
#define CYGARC_REG_IBAT1L          531
#define CYGARC_REG_IBAT2U          532
#define CYGARC_REG_IBAT2L          533
#define CYGARC_REG_IBAT3U          534
#define CYGARC_REG_IBAT3L          535
#define CYGARC_REG_IBAT4U          560
#define CYGARC_REG_IBAT4L          561
#define CYGARC_REG_IBAT5U          562
#define CYGARC_REG_IBAT5L          563
#define CYGARC_REG_IBAT6U          564
#define CYGARC_REG_IBAT6L          565
#define CYGARC_REG_IBAT7U          566
#define CYGARC_REG_IBAT7L          567



#define CYGARC_REG_DBAT0U          536
#define CYGARC_REG_DBAT0L          537
#define CYGARC_REG_DBAT1U          538
#define CYGARC_REG_DBAT1L          539
#define CYGARC_REG_DBAT2U          540
#define CYGARC_REG_DBAT2L          541
#define CYGARC_REG_DBAT3U          542
#define CYGARC_REG_DBAT3L          543
#define CYGARC_REG_DBAT4U          568
#define CYGARC_REG_DBAT4L          569
#define CYGARC_REG_DBAT5U          570
#define CYGARC_REG_DBAT5L          571
#define CYGARC_REG_DBAT6U          572
#define CYGARC_REG_DBAT6L          573
#define CYGARC_REG_DBAT7U          574
#define CYGARC_REG_DBAT7L          575


#define CYGARC_REG_DMISS   976
#define CYGARC_REG_DCMP    977
#define CYGARC_REG_HASH1   978
#define CYGARC_REG_HASH2   979
#define CYGARC_REG_IMISS   980
#define CYGARC_REG_ICMP    981
#define CYGARC_REG_RPA     982


// Hardware Implementation Defined Special Purpose Registers
#define CYGARC_REG_HID0   1008
#define CYGARC_REG_HID1   1009
#define CYGARC_REG_IABR   1010
#define CYGARC_REG_HID2   1011
#define CYGARC_REG_DABR   1013


//memory mapped registers

#define CYGARC_IMM_BASE						0xFF400000	//Internal Memory Mapped Registers
#define CYGARC_REG_IMM_IMMRBAR				0x0000		//Internal memory map base address register
#define CYGARC_REG_IMM_SWCRR				0x0204		//System Watchdog Control Register
#define CYGARC_REG_IMM_SPCR					0x0110		//System priority configuration register
#define CYGARC_REG_IMM_LCRR					0x50d4		//Clock Ratio Register
#define CYGARC_REG_IMM_LBLAWBAR0			0x0020		//LBLAWBAR0—LBC local access window 0 base address register
#define CYGARC_REG_IMM_LBLAWAR0				0x0024		//LBLAWAR0—LBC local access window 0 attribute register
#define CYGARC_REG_IMM_LBLAWBAR1			0x0028		//LBLAWBAR1—LBC local access window 1 base address register
#define CYGARC_REG_IMM_LBLAWAR1				0x002c		//LBLAWAR1—LBC local access window 1 attribute register
#define CYGARC_REG_IMM_LBLAWBAR2			0x0030		//LBLAWBAR2—LBC local access window 2 base address register
#define CYGARC_REG_IMM_LBLAWAR2				0x0034		//LBLAWAR2—LBC local access window 2 attribute register
#define CYGARC_REG_IMM_LBLAWBAR3			0x0038		//LBLAWBAR3—LBC local access window 2 base address register
#define CYGARC_REG_IMM_LBLAWAR3				0x003c		//LBLAWAR3—LBC local access window 2 attribute register

#define CYGARC_REG_IMM_DDRLAWBAR0			0x00A0		//DDRLAWBAR0—DDR local access window0 base address register
#define CYGARC_REG_IMM_DDRLAWAR0			0x00A4		//DDRLAWAR0—DDR local access window0 attribute register
#define CYGARC_REG_IMM_DDRLAWBAR1			0x00A8		//DDRLAWBAR1—DDR local access window1 base address register
#define CYGARC_REG_IMM_DDRLAWAR1			0x00AC		//DDRLAWAR1—DDR local access window1 attribute register
#define CYGARC_REG_IMM_DDRCDR				0x0128		//DDRCDR DDR control driver register

#define CYGARC_REG_IMM_SICRL				0x0114		//System I/O Configuration Register Low
#define CYGARC_REG_IMM_SICRH				0x0118		//System I/O Configuration Register High

#define CYGARC_REG_IMM_CS0_BNDS				0x2000		//CS0_BNDS—Chip select 0 memory bounds
#define CYGARC_REG_IMM_CS0_CONFIG			0x2080		//CS0_CONFIG—Chip select 0 configuration

#define CYGARC_REG_IMM_TIMING_CFG_3			0x2100		//TIMING_CFG_0—DDR SDRAM timing configuration 3
#define CYGARC_REG_IMM_TIMING_CFG_0			0x2104		//TIMING_CFG_0—DDR SDRAM timing configuration 0
#define CYGARC_REG_IMM_TIMING_CFG_1			0x2108		//TIMING_CFG_0—DDR SDRAM timing configuration 1
#define CYGARC_REG_IMM_TIMING_CFG_2			0x210c		//TIMING_CFG_0—DDR SDRAM timing configuration 2

#define CYGARC_REG_IMM_DDR_SDRAM_CFG		0x2110		//DDR_SDRAM_CFG—DDR SDRAM control configuration
#define CYGARC_REG_IMM_DDR_SDRAM_CFG_2		0x2114		//DDR_SDRAM_CFG_2—DDR SDRAM control configuration

#define CYGARC_REG_IMM_DDR_SDRAM_MODE		0x2118		//DDR_SDRAM_MODE—DDR SDRAM mode configuration
#define CYGARC_REG_IMM_DDR_SDRAM_MODE_2		0x211c		//DDR_SDRAM_MODE—DDR SDRAM mode configuration

#define CYGARC_REG_IMM_DDR_SDRAM_INTERVAL	0x2124		//DDR_SDRAM_INTERVAL—DDR SDRAM interval configuration

#define CYGARC_REG_IMM_DDR_SDRAM_CLK_CNTL	0x2130		//DDR_SDRAM_CLK_CNTL—DDR SDRAM clock control

#define CYGARC_PPC83XX_REG_IMM_BR0 			0x5000 		//BR0—Options register 0
#define CYGARC_PPC83XX_REG_IMM_OR0 			0x5004 		//OR0—Options register 0
#define CYGARC_PPC83XX_REG_IMM_BR1 			0x5008 		//BR1—Options register 1
#define CYGARC_PPC83XX_REG_IMM_OR1 			0x500C 		//OR1—Options register 1
#define CYGARC_PPC83XX_REG_IMM_BR2 			0x5010 		//BR2—Options register 2
#define CYGARC_PPC83XX_REG_IMM_OR2 			0x5014 		//OR2—Options register 2
#define CYGARC_PPC83XX_REG_IMM_BR3 			0x5018 		//BR3—Options register 3
#define CYGARC_PPC83XX_REG_IMM_OR3 			0x501c 		//OR3—Options register 3

#define CYGARC_REG_IMM_SWCRR 				0x0204 		//SWCRR—System watchdog control register R/W 0x0000_0007 5.4.4.1/5-30
#define CYGARC_REG_IMM_SWCNR 				0x0208 		//SWCNR—System watchdog count register R 0x0000_FFFF 5.4.4.2/5-31
#define CYGARC_REG_IMM_SWSRR 				0x020E 		//SWSRR—System watchdog service register R/W 0x0000 5.4.4.3/5-32

#define CYGARC_REG_IMM_RTCNR 				0x0300	//—Real time counter control register R/W 0x0000_0000 5.5.5.1/5-38
#define CYGARC_REG_IMM_RTLDR 				0x0304	//—Real time counter load register R/W 0x0000_0000 5.5.5.2/5-38
#define CYGARC_REG_IMM_RTPSR 				0x0308	//—Real time counter prescale register R/W 0x0000_0000 5.5.5.3/5-39
#define CYGARC_REG_IMM_RTCTR 				0x030C	//—Real time counter register R 0x0000_0000 5.5.5.4/5-39
#define CYGARC_REG_IMM_RTEVR 				0x0310	//—Real time counter event register R/W 0x0000_0000 5.5.5.5/5-40
#define CYGARC_REG_IMM_RTALR 				0x0314	//—Real time counter alarm register R/W 0xFFFF_FFFF 5.5.5.6/5-40

#define CYGARC_REG_IMM_INTR_SICFR 			0x0700 		//System global interrupt configuration register (SICFR) R/W 0x0000_0000 8.5.1/8-8
#define CYGARC_REG_IMM_INTR_SIVCR 			0x0704 		//System regular interrupt vector register (SIVCR) R 0x0000_0000 8.5.2/8-9
#define CYGARC_REG_IMM_INTR_SIPNR_H 		0x0708 		//System internal interrupt pending register (SIPNR_H) R 0x0000_0000 8.5.3/8-11
#define CYGARC_REG_IMM_INTR_SIPNR_L 		0x070C 		//System internal interrupt pending register (SIPNR_L) R 0x0000_0000 8.5.3/8-11
#define CYGARC_REG_IMM_INTR_SIPRR_A 		0x0710 		//System internal interrupt group A priority register (SIPRR_A) R/W 0x0530_9770 8.5.4/8-14
#define CYGARC_REG_IMM_INTR_SIPRR_D 		0x071C 		//System internal interrupt group D priority register (SIPRR_D) R/W 0x0530_9770 8.5.5/8-15
#define CYGARC_REG_IMM_INTR_SIMSR_H 		0x0720 		//System internal interrupt mask register (SIMSR_H) R/W 0x0000_0000 8.5.6/8-15
#define CYGARC_REG_IMM_INTR_SIMSR_L 		0x0724 		//System internal interrupt mask register (SIMSR_L) R/W 0x0000_0000 8.5.6/8-15
#define CYGARC_REG_IMM_INTR_SICNR 			0x0728 		//System internal interrupt control register (SICNR) R/W 0x0000_0000 8.5.7/8-17
#define CYGARC_REG_IMM_INTR_SEPNR 			0x072C 		//System external interrupt pending register (SEPNR) R/W Special 8.5.8/8-18
#define CYGARC_REG_IMM_INTR_SMPRR_A 		0x0730 		//System mixed interrupt group A priority register (SMPRR_A) R/W 0x0530_9770 8.5.9/8-19
#define CYGARC_REG_IMM_INTR_SMPRR_B 		0x0734 		//System mixed interrupt group B priority register (SMPRR_B) R/W 0x0530_9770 8.5.10/8-19
#define CYGARC_REG_IMM_INTR_SEMSR 			0x0738 		//System external interrupt mask register (SEMSR) R/W 0x0000_0000 8.5.11/8-20
#define CYGARC_REG_IMM_INTR_SECNR 			0x073C 		//System external interrupt control register (SECNR) R/W 0x0000_0000 8.5.12/8-21
#define CYGARC_REG_IMM_INTR_SERSR 			0x0740 		//System error status register (SERSR) R/W 0x0000_0000 8.5.13/8-22
#define CYGARC_REG_IMM_INTR_SERMR 			0x0744 		//System error mask register (SERMR) R/W 8.5.14/8-23
#define CYGARC_REG_IMM_INTR_SERCR 			0x0748 		//System error control register (SERCR) R/W 0x0000_0000 8.5.15/8-24
#define CYGARC_REG_IMM_INTR_SIFCR_H 		0x0750 		//System internal interrupt force register (SIFCR_H) R/W 0x0000_0000 8.5.16/8-25
#define CYGARC_REG_IMM_INTR_SIFCR_L 		0x0754 		//System internal interrupt force register (SIFCR_L) R/W 0x0000_0000 8.5.16/8-25
#define CYGARC_REG_IMM_INTR_SEFCR 			0x0758 		//System external interrupt force register (SEFCR) R/W 0x0000_0000 8.5.17/8-26
#define CYGARC_REG_IMM_INTR_SERFR 			0x075C 		//System error force register (SERFR) R/W 0x0000_0000 8.5.18/8-26
#define CYGARC_REG_IMM_INTR_SCVCR 			0x0760 		//System critical interrupt vector register (SCVCR) R 0x0000_0000 8.5.19/8-27
#define CYGARC_REG_IMM_INTR_SMVCR 			0x0764 		//System management interrupt vector register (SMVCR) R 0x0000_0000 8.5.20/8-27

// Reset module
#define CYGARC_REG_IMM_RCWLR 				0x0900
#define CYGARC_REG_IMM_RCWHR 				0x0904
#define CYGARC_REG_IMM_RSR 		    		0x0910
#define CYGARC_REG_IMM_RMR 		    		0x0914
#define CYGARC_REG_IMM_RPR 		    		0x0918
#define CYGARC_REG_IMM_RCR 		    		0x091C
#define CYGARC_REG_IMM_RCER 		    	0x0920

#define CYGARC_MSR_EE		                (1<<15)		/* External Interrupt Enable */
#define CYGARC_MSR_IR		                (1<<5)		/* Instruction Relocate */
#define CYGARC_MSR_DR		                (1<<4)		/* Data Relocate */

#define CYGARC_RCER_CRE			             0x00000001	/* software hard reset */
#define CYGARC_RCR_SWHR			             0x00000002	/* software hard reset */

//GPIO Registers
#define CYGARC_REG_IMM_GP1DIR 				0x0C00 		//—GPIO direction register R/W 0x0000_0000 21.3.1/21-3
#define CYGARC_REG_IMM_GP1DR 				0x0C04 		//—GPIO open drain register R/W 0x0000_0000 21.3.2/21-3
#define CYGARC_REG_IMM_GP1DAT 				0x0C08 		//—GPIO data register R/W 0x0000_0000 21.3.3/21-4
#define CYGARC_REG_IMM_GP1IER 				0x0C0C 		//—GPIO interrupt event register R/W Undefined 21.3.4/21-4
#define CYGARC_REG_IMM_GP1IMR 				0x0C10 		//—GPIO interrupt mask register R/W 0x0000_0000 21.3.5/21-4
#define CYGARC_REG_IMM_GP1ICR 				0x0C14 		//—GPIO external interrupt control register R/W 0x0000_0000 21.3.6/21-5

//Periodic Interval Timer (PIT) Registers
#define CYGARC_REG_IMM_PTCNR 0x0400 //—Periodic interval timer control register R/W 0x0000_0000 5.6.5.1/5-45
#define CYGARC_REG_IMM_PTLDR 0x0404 //—Periodic interval timer load register R/W 0x0000_0000 5.6.5.2/5-45
#define CYGARC_REG_IMM_PTPSR 0x0408 //—Periodic interval timer prescale register R/W 0x0000_0000 5.6.5.3/5-46
#define CYGARC_REG_IMM_PTCTR 0x040C //—Periodic interval timer counter register R 0x0000_0000 5.6.5.4/5-46
#define CYGARC_REG_IMM_PTEVR 0x0410 //—Periodic interval timer event register R/W 0x0000_0000 5.6.5.5/5-47

#define PTCNR_CLEN	0x00000080 //Clock enable control bit.
#define PTCNR_CLIN	0x00000040 //Input clock control bit.
#define PTCNR_PIM	0x00000001 //Periodic interrupt mask bit.
#define PTEVR_PIF	0x00000001 //Periodic interrupt flag bit.

//Global Timers Module 1
#define CYGARC_REG_IMM_GTM1_GTCFR1 0x00500 //Timer 1 and 2 global timers configuration register R/W 0x00 5.7.5.1/5-54
#define CYGARC_REG_IMM_GTM1_GTCFR2 0x00504 //Timer 3 and 4 global timers configuration register R/W 0x00 5.7.5.1/5-54
#define CYGARC_REG_IMM_GTM1_GTMDR1 0x00510 //Timer 1 global timers mode register R/W 0x0000 5.7.5.2/5-57
#define CYGARC_REG_IMM_GTM1_GTMDR2 0x00512 //Timer 2 global timers mode register
#define CYGARC_REG_IMM_GTM1_GTRFR1 0x00514 //Timer 1 global timers reference register R/W 0xFFFF 5.7.5.3/5-59
#define CYGARC_REG_IMM_GTM1_GTRFR2 0x00516 //Timer 2 global timers reference register
#define CYGARC_REG_IMM_GTM1_GTCPR1 0x00518 //Timer 1 global timers capture register R/W 0x0000 5.7.5.4/5-59
#define CYGARC_REG_IMM_GTM1_GTCPR2 0x0051A //Timer 2 global timers capture register
#define CYGARC_REG_IMM_GTM1_GTCNR1 0x0051C //Timer 1 global timers counter register R/W 0x0000 5.7.5.5/5-60
#define CYGARC_REG_IMM_GTM1_GTCNR2 0x0051E //Timer 2 global timers counter register
#define CYGARC_REG_IMM_GTM1_GTMDR3 0x00520 //Timer 3 global timers mode register R/W 0x0000 5.7.5.2/5-57
#define CYGARC_REG_IMM_GTM1_GTMDR4 0x00522 //Timer 4 global timers mode register
#define CYGARC_REG_IMM_GTM1_GTRFR3 0x00524 //Timer 3 global timers reference register R/W 0xFFFF 5.7.5.3/5-59
#define CYGARC_REG_IMM_GTM1_GTRFR4 0x00526 //Timer 4 global timers reference register
#define CYGARC_REG_IMM_GTM1_GTCPR3 0x00528 //Timer 3 global timers capture register R 0x0000 5.7.5.4/5-59
#define CYGARC_REG_IMM_GTM1_GTCPR4 0x0052A //Timer 4 global timers capture register
#define CYGARC_REG_IMM_GTM1_GTCNR3 0x0052C //Timer 3 global timers counter register R/W 0x0000 5.7.5.5/5-60
#define CYGARC_REG_IMM_GTM1_GTCNR4 0x0052E //Timer 4 global timers counter register
#define CYGARC_REG_IMM_GTM1_GTEVR1 0x00530 //Timer 1 global timers event register Special 0x0000 5.7.5.6/5-60
#define CYGARC_REG_IMM_GTM1_GTEVR2 0x00532 //Timer 2 global timers event register
#define CYGARC_REG_IMM_GTM1_GTEVR3 0x00534 //Timer 3 global timers event register
#define CYGARC_REG_IMM_GTM1_GTEVR4 0x00536 //Timer 4 global timers event register
#define CYGARC_REG_IMM_GTM1_GTPSR1 0x00538 //Timer 1 global timers prescale register R/W 0x0003 5.7.5.7/5-61
#define CYGARC_REG_IMM_GTM1_GTPSR2 0x0053A //Timer 2 global timers prescale register
#define CYGARC_REG_IMM_GTM1_GTPSR3 0x0053C //Timer 3 global timers prescale register
#define CYGARC_REG_IMM_GTM1_GTPSR4 0x0053E //Timer 4 global timers prescale register

//Global Timers Module 2
#define CYGARC_REG_IMM_GTM2_GTCFR1 0x00500 //Timer 1 and 2 global timers configuration register R/W 0x00 5.7.5.1/5-54
#define CYGARC_REG_IMM_GTM2_GTCFR2 0x00504 //Timer 3 and 4 global timers configuration register R/W 0x00 5.7.5.1/5-54
#define CYGARC_REG_IMM_GTM2_GTMDR1 0x00510 //Timer 1 global timers mode register R/W 0x0000 5.7.5.2/5-57
#define CYGARC_REG_IMM_GTM2_GTMDR2 0x00512 //Timer 2 global timers mode register
#define CYGARC_REG_IMM_GTM2_GTRFR1 0x00514 //Timer 1 global timers reference register R/W 0xFFFF 5.7.5.3/5-59
#define CYGARC_REG_IMM_GTM2_GTRFR2 0x00516 //Timer 2 global timers reference register
#define CYGARC_REG_IMM_GTM2_GTCPR1 0x00518 //Timer 1 global timers capture register R/W 0x0000 5.7.5.4/5-59
#define CYGARC_REG_IMM_GTM2_GTCPR2 0x0051A //Timer 2 global timers capture register
#define CYGARC_REG_IMM_GTM2_GTCNR1 0x0051C //Timer 1 global timers counter register R/W 0x0000 5.7.5.5/5-60
#define CYGARC_REG_IMM_GTM2_GTCNR2 0x0051E //Timer 2 global timers counter register
#define CYGARC_REG_IMM_GTM2_GTMDR3 0x00520 //Timer 3 global timers mode register R/W 0x0000 5.7.5.2/5-57
#define CYGARC_REG_IMM_GTM2_GTMDR4 0x00522 //Timer 4 global timers mode register
#define CYGARC_REG_IMM_GTM2_GTRFR3 0x00524 //Timer 3 global timers reference register R/W 0xFFFF 5.7.5.3/5-59
#define CYGARC_REG_IMM_GTM2_GTRFR4 0x00526 //Timer 4 global timers reference register
#define CYGARC_REG_IMM_GTM2_GTCPR3 0x00528 //Timer 3 global timers capture register R 0x0000 5.7.5.4/5-59
#define CYGARC_REG_IMM_GTM2_GTCPR4 0x0052A //Timer 4 global timers capture register
#define CYGARC_REG_IMM_GTM2_GTCNR3 0x0052C //Timer 3 global timers counter register R/W 0x0000 5.7.5.5/5-60
#define CYGARC_REG_IMM_GTM2_GTCNR4 0x0052E //Timer 4 global timers counter register
#define CYGARC_REG_IMM_GTM2_GTEVR1 0x00530 //Timer 1 global timers event register Special 0x0000 5.7.5.6/5-60
#define CYGARC_REG_IMM_GTM2_GTEVR2 0x00532 //Timer 2 global timers event register
#define CYGARC_REG_IMM_GTM2_GTEVR3 0x00534 //Timer 3 global timers event register
#define CYGARC_REG_IMM_GTM2_GTEVR4 0x00536 //Timer 4 global timers event register
#define CYGARC_REG_IMM_GTM2_GTPSR1 0x00538 //Timer 1 global timers prescale register R/W 0x0003 5.7.5.7/5-61
#define CYGARC_REG_IMM_GTM2_GTPSR2 0x0053A //Timer 2 global timers prescale register
#define CYGARC_REG_IMM_GTM2_GTPSR3 0x0053C //Timer 3 global timers prescale register
#define CYGARC_REG_IMM_GTM2_GTPSR4 0x0053E //Timer 4 global timers prescale register

#define CYGARC_REG_IMM_TSEC1 0x24000	//Base address for TSEC1
#define CYGARC_REG_IMM_TSEC2 0x25000	//Base address for TSEC2

#define CYGARC_REG_IMM_TSEC1_MIIMCFG	0x24520	//—MII management configuration R/W 0x0000_0007 15.5.3.5.6/15-71
#define CYGARC_REG_IMM_TSEC1_MIIMCOM	0x24524	//—MII management command R/W 0x0000_0000 15.5.3.5.7/15-72
#define CYGARC_REG_IMM_TSEC1_MIIMADD	0x24528	//—MII management address R/W 0x0000_0000 15.5.3.5.8/15-73
#define CYGARC_REG_IMM_TSEC1_MIIMCON	0x2452C	//—MII management control WO 0x0000_0000 15.5.3.5.9/15-74
#define CYGARC_REG_IMM_TSEC1_MIIMSTAT	0x24530	//—MII management status R 0x0000_0000 15.5.3.5.10/15-74
#define CYGARC_REG_IMM_TSEC1_MIIMIND	0x24534	//—MII management indicator

#define CYGARC_REG_IMM_TSEC2_MIIMCFG	0x25520	//—MII management configuration R/W 0x0000_0007 15.5.3.5.6/15-71
#define CYGARC_REG_IMM_TSEC2_MIIMCOM	0x25524	//—MII management command R/W 0x0000_0000 15.5.3.5.7/15-72
#define CYGARC_REG_IMM_TSEC2_MIIMADD	0x25528	//—MII management address R/W 0x0000_0000 15.5.3.5.8/15-73
#define CYGARC_REG_IMM_TSEC2_MIIMCON	0x2552C	//—MII management control WO 0x0000_0000 15.5.3.5.9/15-74
#define CYGARC_REG_IMM_TSEC2_MIIMSTAT	0x25530	//—MII management status R 0x0000_0000 15.5.3.5.10/15-74
#define CYGARC_REG_IMM_TSEC2_MIIMIND	0x25534	//—MII management indicator




#define GTCFR_RST1				0x01

#define GTMDR_ORI				0x0010
#define GTMDR_FRR				0x0008
#define GTMDR_ICLK_INT_CASCADE	0x0000
#define GTMDR_ICLK_INT_CLK		0x0002
#define GTMDR_ICLK_INT_CLK16	0x0004
#define GTMDR_ICLK_INT_TIN		0x0006



#ifdef CYGARC_HAL_COMMON_EXPORT_CPU_MACROS
#define HID0       CYGARC_REG_HID0
#define HID1       CYGARC_REG_HID1
#define HID2       CYGARC_REG_HID2
#endif // ifdef CYGARC_HAL_COMMON_EXPORT_CPU_MACROS

//--------------------------------------------------------------------------
#ifdef CYGARC_HAL_COMMON_EXPORT_CPU_MACROS

// BATs
#define IBAT0U          528
#define IBAT0L          529
#define IBAT1U          530
#define IBAT1L          531
#define IBAT2U          532
#define IBAT2L          533
#define IBAT3U          534
#define IBAT3L          535
#define IBAT4U          560
#define IBAT4L          561
#define IBAT5U          562
#define IBAT5L          563
#define IBAT6U          564
#define IBAT6L          565
#define IBAT7U          566
#define IBAT7L          567

#define DBAT0U          536
#define DBAT0L          537
#define DBAT1U          538
#define DBAT1L          539
#define DBAT2U          540
#define DBAT2L          541
#define DBAT3U          542
#define DBAT3L          543
#define DBAT4U          568
#define DBAT4L          569
#define DBAT5U          570
#define DBAT5L          571
#define DBAT6U          572
#define DBAT6L          573
#define DBAT7U          574
#define DBAT7L          575

#define UBAT_BEPIMASK   0xfffe0000      // effective address mask
#define UBAT_BLMASK     0x00001ffc      // block length mask
#define UBAT_VS         0x00000002      // supervisor mode valid bit
#define UBAT_VP         0x00000001      // problem mode valid bit

#define LBAT_BRPNMASK   0xfffe0000      // real address mask
#define LBAT_W          0x00000040      // write-through
#define LBAT_I          0x00000020      // caching-inhibited
#define LBAT_M          0x00000010      // memory coherence
#define LBAT_G          0x00000008      // guarded

#define LBAT_PP_NA      0x00000000      // no access
#define LBAT_PP_RO      0x00000001      // read-only
#define LBAT_PP_RW      0x00000002      // read/write


#endif // ifdef CYGARC_HAL_COMMON_EXPORT_CPU_MACROS

//-----------------------------------------------------------------------------
#endif // ifdef CYGONCE_HAL_VAR_REGS_H
// End of var_regs.h
