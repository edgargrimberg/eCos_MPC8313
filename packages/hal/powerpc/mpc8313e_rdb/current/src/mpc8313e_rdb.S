### ####ECOSGPLCOPYRIGHTBEGIN####
## -------------------------------------------
## This file is part of eCos, the Embedded Configurable Operating System.
## Copyright (C) 1998, 1999, 2000, 2001, 2002, 2009 Free Software Foundation, Inc.
##
## eCos is free software; you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free
## Software Foundation; either version 2 or (at your option) any later
## version.
##
## eCos is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License
## along with eCos; if not, write to the Free Software Foundation, Inc.,
## 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
##
## As a special exception, if other files instantiate templates or use
## macros or inline functions from this file, or you compile this file
## and link it with other works to produce a work based on this file,
## this file does not by itself cause the resulting work to be covered by
## the GNU General Public License. However the source code for this file
## must still be made available in accordance with section (3) of the GNU
## General Public License v2.
##
## This exception does not invalidate any other reasons why a work based
## on this file might be covered by the GNU General Public License.
## -------------------------------------------
## ####ECOSGPLCOPYRIGHTEND####


#include <pkgconf/hal.h>
#include <cyg/hal/arch.inc>
#include <cyg/hal/ppc_regs.h>		/* on-chip resource layout, special */
#------------------------------------------------------------------------------

//.globl pc_trick
//pc_trick:
//#this will  be executed from RAM
//
//lwi r3, 0x80000016  //LBLAWAR0 - LBLAWAR0有BC local access window 0 attribute register
//stw r3, CYGARC_REG_IMM_LBLAWAR1(r30)
//
//lwi r3, 0x80000000  //LBLAWBAR0 : Flash - LBLAWBAR0有BC local access window 0 base address register
//stw r3, CYGARC_REG_IMM_LBLAWBAR1(r30)
//
//lwi r3, 0xff8016a0      //OR0 : Flash - Ref: 5
//stw r3, CYGARC_PPC83XX_REG_IMM_OR0(r30)
//
//lwi r3, 0x80001001      //BR0 : Flash - Ref: 4
//stw r3, CYGARC_PPC83XX_REG_IMM_BR0(r30)
//
//#compute the ROM address to return to
//lwi		r3, back_2_flash
//mtlr	r3
//
//blr
//pc_trick_done:


        .globl  hal_hardware_init
hal_hardware_init:

//enable instruction cache

lwi r3 , 0xff400000
lwi r30, CYGARC_IMM_BASE
stw r30, CYGARC_REG_IMM_IMMRBAR(r3) //MABR to 0xe0000000 - moving the memory mapped registers base address (cap 2.3)
isync

lwi r3, 0xffff0000	//SWCRR: disable watchdog - System watchdog control register
stw r3, CYGARC_REG_IMM_SWCRR(r30)
isync

lwi r3, 0x00400000  //SPCR : enable timebase unit
stw r3, CYGARC_REG_IMM_SPCR(r30)
isync

lwi r3, 0x80000002  //LCRR : CLKDIV = 4
stw r3, CYGARC_REG_IMM_LCRR(r30)
lwz r3, CYGARC_REG_IMM_LCRR(r30)
isync


//led 0
lwi r3, 0x100
stw r3, CYGARC_REG_IMM_GP1DIR(r30)
isync

#if  (defined(CYG_HAL_STARTUP_ROMRAM) || defined(CYG_HAL_STARTUP_ROM))

lwi r3, 0x80000016  //LBLAWAR1 - LBLAWAR1有BC local access window 0 attribute register
stw r3, CYGARC_REG_IMM_LBLAWAR1(r30)

lwi r3, 0x80000000  //LBLAWBAR1 : Flash - LBLAWBAR1有BC local access window 0 base address register
stw r3, CYGARC_REG_IMM_LBLAWBAR1(r30)

lwi r3, 0x80001001      //BR0 : Flash - Ref: 4
stw r3, CYGARC_PPC83XX_REG_IMM_BR0(r30)

lwi r3, 0x00006FF7      //OR0 : Flash - Ref: 5
stw r3, CYGARC_PPC83XX_REG_IMM_OR0(r30)
//flash should be mirrored on the entire address space

mflr r6

lwi r3, back_2_flash
mtlr r3
blr

.globl back_2_flash
back_2_flash:

mtlr r6

lwi r3, 0xff806FF7      //OR0 : Flash - Ref: 5
stw r3, CYGARC_PPC83XX_REG_IMM_OR0(r30)

lwi r3, 0x00000016  //LBLAWAR0 - disable local window 0, so it does not stay in the way of SDRAM
stw r3, CYGARC_REG_IMM_LBLAWAR0(r30)

//led1
stw r3, CYGARC_REG_IMM_GP1DIR(r30)
lwi r3, 0x400
stw r3, CYGARC_REG_IMM_GP1DIR(r30)


// Initialize LAWBAR's

lwi r3, 0x00000000  //DDRLAWBAR0 : DDR Main SODIMM
stw r3, CYGARC_REG_IMM_DDRLAWBAR0(r30)
lwz r3, CYGARC_REG_IMM_DDRLAWBAR0(r30)
isync

lwi r3, 0x8000001a  //DDRLAWBAR0 : DDR Main SODIMM
stw r3, CYGARC_REG_IMM_DDRLAWAR0(r30)
lwz r3, CYGARC_REG_IMM_DDRLAWAR0(r30)
isync

lwi r3, 0x00000000  //DDRLAWBAR1 : DDR Main SODIMM
stw r3, CYGARC_REG_IMM_DDRLAWBAR1(r30)
lwz r3, CYGARC_REG_IMM_DDRLAWBAR1(r30)
isync

lwi r3, 0x8000001a  //DDRLAWBAR1 : DDR Main SODIMM
stw r3, CYGARC_REG_IMM_DDRLAWAR1(r30)
lwz r3, CYGARC_REG_IMM_DDRLAWAR1(r30)
isync


//Setup DDR - Reference : Memory_Register_Settings_2008_07_31.doc
lwi r3,  0x73000002  //DDRCDR - Ref: 3
stw r3, CYGARC_REG_IMM_DDRCDR(r30)
isync

//pause a bit, errata
        lwi     r3,0x10000
        mtctr   r3
99:     nop
        bdnz    99b

lwi r3, 0x02000000  //DDR_CLK_CNTL - Ref: 1.11
stw r3, CYGARC_REG_IMM_DDR_SDRAM_CLK_CNTL(r30)
isync

lwi r3, 0x0000007F  //CS0_BNDS - Ref: 1.1, changed the address values to fit the old mem map
stw r3, CYGARC_REG_IMM_CS0_BNDS(r30)
isync

lwi r3, 0x80840102  //CS0_CONFIG - Ref: 1.2
stw r3, CYGARC_REG_IMM_CS0_CONFIG(r30)
isync

//TODO: CHECK THIS
lwi r3, 0x43080000  //DDR_CFG	  - Ref: 1.7
stw r3, CYGARC_REG_IMM_DDR_SDRAM_CFG(r30)
isync

lwi r3, 0x00220802  //TIMING_CFG_0 - Ref: 1.3
stw r3, CYGARC_REG_IMM_TIMING_CFG_0(r30)
isync

lwi r3, 0x3935d322  //TIMING_CFG_1 - Ref: 1.4
stw r3, CYGARC_REG_IMM_TIMING_CFG_1(r30)
isync

lwi r3,  0x0f9048ca  //TIMING_CFG_2 - Ref: 1.6
stw r3, CYGARC_REG_IMM_TIMING_CFG_2(r30)
isync

lwi r3, 0x00000000  //TIMING_CFG_3 - Ref: 1.5
stw r3, CYGARC_REG_IMM_TIMING_CFG_3(r30)
isync

lwi r3, 0x00401010  //DDR_CFG_2	  - Ref: 1.8
stw r3, CYGARC_REG_IMM_DDR_SDRAM_CFG_2(r30)
isync

//pause a bit, errata
        lwi     r3,0x10000
        mtctr   r3
99:     nop
        bdnz    99b

lwi r3, 0x44400232  //DDR_MODE	  - Ref: 1.9, bit 25-31 reseted because there was no value specified
stw r3, CYGARC_REG_IMM_DDR_SDRAM_MODE(r30)
isync

lwi r3, 0x8000C000  //DDR_MODE	  - Ref: 1.9, bit 25-31 reseted because there was no value specified
stw r3, CYGARC_REG_IMM_DDR_SDRAM_MODE_2(r30)
isync


lwi r3, 0x03200064  //DDR_INTERVAL - Ref: 1.10
stw r3, CYGARC_REG_IMM_DDR_SDRAM_INTERVAL(r30)
isync

lwi r3, 0x02000000  //DDR_CLK_CNTL - Ref: 1.11
stw r3, CYGARC_REG_IMM_DDR_SDRAM_CLK_CNTL(r30)
isync

        lwi     r3,0x10000
        mtctr   r3
100:     nop
        bdnz    100b


lwi r3, 0xc3080000  //
stw r3, CYGARC_REG_IMM_DDR_SDRAM_CFG(r30)
isync


//check if D_INIT has been cleared by hardware
lwi     r4, 0x00000000

100:    lwz r3, CYGARC_REG_IMM_DDR_SDRAM_CFG_2(r30)
		lwi r5, 0x10
		and r3, r3, r5
		cmpwi cr0, r3, 0
		addi r4, r4, 1
        bne   cr0, 100b


//led 0
lwi r3, 0x100
stw r3, CYGARC_REG_IMM_GP1DIR(r30)
isync


#ifdef CYG_HAL_STARTUP_ROM
	lwi		r3, _hal_hardware_init_done
	mtlr	r3
#endif

#ifdef CYG_HAL_STARTUP_ROMRAM
        // Copy image from ROM to RAM
        mflr    r3
        lwi     r4,(CYGMEM_REGION_rom&0xFE000000)
        lwi     r5,0x01FFFFFF   // ROM/FLASH base
        and     r3,r3,r5        // segment relative
        lwi     r6,_hal_hardware_init_done
        mtlr    r6
        sub     r6,r3,r6        // Absolute address
        add     r6,r6,r4        // FLASH address
        lwi     r7,0            // where to copy to
        lwi     r8,__ram_data_end
10:     lwz     r5,0(r6)
        stw     r5,0(r7)
        addi    r6,r6,4
        addi    r7,r7,4
        cmplw   r7,r8
        bne     10b
#endif




# copy memory in a loop
lwi     r3,0xdeadbabe
lwi     r4,0x38000
lwi     r5,0x40000

        sub     r5,r5,r4                # compute number of words to copy
        srwi    r5,r5,2
        mtctr   r5
        subi    r4,r4,4
1:
        stwu    r3,4(r4)                # store in RAM
        bdnz    1b
2:

# verify memory in a loop
lwi     r3,0xdeadbabe
lwi     r4,0x38000
lwi     r5,0x40000

sub     r5,r5,r4                # compute number of words to copy
srwi    r5,r5,2
mtctr   r5
subi    r4,r4,4

101:
	lwzu    r6,4(r4)                # read from RAM
	cmplw	r6, r3
	bne     fail_led
	bdnz    101b
102:



#endif // ROM/ROMRAM

////clear RTC event register
//lwi	r3, 0x3	//write 1 to AIF and SIF
//stw r3, CYGARC_REG_IMM_RTEVR(r30)
//
////enable RTC
//lwi	r3, 0x81 //CLEN = 1, CLIN set to internal clock, Second interrupt mask bit
//stw r3, CYGARC_REG_IMM_RTCNR(r30)


hardware_init_done:
        blr
.globl fail_led
fail_led:
	//led1
	lwi r30, CYGARC_IMM_BASE
	lwi r3, 0x201
	stw r3, CYGARC_REG_IMM_GP1DIR(r30)
inf_loop:
	b inf_loop


