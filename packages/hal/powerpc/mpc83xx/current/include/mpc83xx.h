#ifndef CYGONCE_HAL_PPC_QUICC2_MPC83XX_H
#define CYGONCE_HAL_PPC_QUICC2_MPC83XX_H

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
//      mpc83xx.h
//
//      PowerPC QUICC2 PRO register definitions
//
//==========================================================================


#include <cyg/hal/plf_regs.h>           // For IMM base

//-----------------------------------------------------------------------------
// MMU setup macros. The correct place for them is in hal_mem.h

//-----------------------------------------------------------------------------
#define UART1_OFFSET 	0x4500
#define UART2_OFFSET 	0x4600

//Use the IMM base and the UART offset with these register offsets
#define URBR			0x0000		//UART receiver buffer register (R)
#define UTHR			0x0000		//UART transmitter holding register (W)
#define UDLB			0x0000		//UART divisor least significant byte register (R/W)

#define UIER			0x0001		//UART interrupt enable register R/W
#define UDMB			0x0001		//UART divisor most significant byte register R/W

#define UIIR			0x0002		//UART interrupt ID register R
#define UFCR			0x0002		//UART FIFO control register W
#define UAFR			0x0002		//UART alternate function register R/W

#define ULCR			0x0003		//UART line control register R/W
#define UMCR			0x0004		//UART MODEM control register R/W
#define ULSR			0x0005		//UART line status register R 0x60
#define UMSR			0x0006		//UART MODEM status register R 0x00
#define USCR			0x0007		//UART scratch register R/W 0x00
#define UDSR			0x0010		//UART DMA status register R 0x01


#define ULCR_DLAB		0x80		//Divisor latch access bit
									//0 Access to all registers except UDLB, UAFR, and UDMB.
									//1 Ability to access UDMB, UDLB, and UAFR.

#define ULCR_SB			0x40		//Set break
									//0 Send normal UTHR data onto the SOUT signal.
									//1 Force logic 0 to be on SOUT. Data in the UTHR is not affected.

#define ULCR_SP			0x20		//Stick parity
									//0 Stick parity is disabled.
									//1 If PEN = 1 and EPS = 1, space parity is selected; if PEN = 1 and EPS = 0, mark parity is selected.

#define ULCR_EPS		0x10		//Even parity select. See Table 18-14.
									//0 If PEN = 1 and SP = 0 then odd parity is selected.
									//1 If PEN = 1 and SP = 0 then even parity is selected.

#define ULCR_PEN		0x08		//Parity enable
									//0 No parity generation and checking.
									//1 Generate parity bit as a transmitter, and check parity as a receiver.

#define ULCR_NTSB		0x04		//Number of STOP bits
									//0 One STOP bit is generated in the transmitted data.
									//1 When a 5-bit data length is selected, 1 1/2 STOP bits are generated. When either a 6-, 7-, or 8-bit word
									//length is selected, two STOP bits are generated.

									//Word length select. Number of bits that comprise the character length.
#define ULCR_WLS_5		0x00		//00 5 bits
#define ULCR_WLS_6		0x01		//01 6 bits
#define ULCR_WLS_7		0x02		//10 7 bits
#define ULCR_WLS_8		0x03		//11 8 bits

#define ULSR_DR			0x01		//Data ready
									// 0 Cleared when URBR is read or when all of the data in the receiver FIFO is read.
									// 1 A character was received in the URBR or the receiver FIFO.
struct DUART_memmap
{
	union
	{
		cyg_uint8 memmap_URBR;	//UART receiver buffer register
		cyg_uint8 memmap_UTHR;	//UART transmitter holding register (W)
		cyg_uint8 memmap_UDLB;	//UART divisor least significant byte register (R/W)
	};

	union
	{
		cyg_uint8 memmap_UIER;	//UART interrupt enable register R/W
		cyg_uint8 memmap_UDMB;	//UART divisor most significant byte register R/W
	};

	union
	{
		cyg_uint8 memmap_UIIR;	//UART interrupt ID register R
		cyg_uint8 memmap_UFCR;	//UART FIFO control register W
		cyg_uint8 memmap_UAFR;	//UART alternate function register R/W
	};

	cyg_uint8 memmap_ULCR;	//UART line control register R/W
	cyg_uint8 memmap_UMCR;	//UART MODEM control register R/W
	cyg_uint8 memmap_ULSR;	//UART line status register R
	cyg_uint8 memmap_UMSR;	//UART MODEM status register R
	cyg_uint8 memmap_USCR;	//UART scratch register R/W
	cyg_uint8 memmap_UDSR;	//UART DMA status register R

};


typedef struct IMMR {

	volatile cyg_uint32 immrbar;
	volatile cyg_uint8 reserved0[0x4];
	volatile cyg_uint32 altcbar;

	volatile cyg_uint8 reserved1[0x14];

	struct local_acc_window {
		volatile cyg_uint32 lblawbar;
		volatile cyg_uint32 lblawar;
	} local_acc_windows[4];

	volatile cyg_uint8 reserved2[0x20];


	struct pci_acc_window {
		volatile cyg_uint32 pcilawbar;
		volatile cyg_uint32 pcilawar;
	}pci_acc_window[2];


	volatile cyg_uint8 reserved3[0x30];

	struct DDR_local_acc_win
	{
		volatile cyg_uint32 	ddrlawbar;
		volatile cyg_uint32 	ddrlawar;
	}DDR_local_acc_win[2];

	volatile cyg_uint8 reserved4[0x50];
	volatile cyg_uint32 sgprl;
	volatile cyg_uint32 sgprh;
	volatile cyg_uint32 spridr;


	volatile cyg_uint8 reserved5[0x4];
	volatile cyg_uint32 spcr;
	volatile cyg_uint32 sicrl;
	volatile cyg_uint32 sicrh;

	volatile cyg_uint8 reserved6[0xC];


	volatile cyg_uint32 ddrcdr;
	volatile cyg_uint32 ddrdsr;



	volatile cyg_uint8 reserved7[0xD0];



	/* Watchdog Timer (WDT) Registers */



	volatile cyg_uint8 reserved8[0x4];
	volatile cyg_uint32 swcrr;
	volatile cyg_uint32 swcnr;
	volatile cyg_uint8 reserved9[0x2];

	volatile cyg_uint16 swsrr;

	volatile cyg_uint8 reserved10[0xF0];


	/* Real Time Clock Module Registers (RTC) */
	volatile cyg_uint32 rtcnr;
	volatile cyg_uint32 rtldr;
	volatile cyg_uint32 rtpsr;
	volatile cyg_uint32 rtctr;
	volatile cyg_uint32 rtevr;
	volatile cyg_uint32 rtalr;

	volatile cyg_uint8 reserved11[0xE8];


	/* Periodic Interval Timer (PIT) Registers */

	volatile cyg_uint32 ptcnr;
	volatile cyg_uint32 ptldr;
	volatile cyg_uint32 ptpsr;
	volatile cyg_uint32 ptctr;
	volatile cyg_uint32 ptevr;
	volatile cyg_uint8 reserved12[0xEC];


	struct global_timers_module
	{
		volatile cyg_uint8 gtcfr1;
		volatile cyg_uint8 reserved13[0x3];
		volatile cyg_uint8 gtcfr2;
		volatile cyg_uint8 reserved14[0xB];


		volatile cyg_uint16 gtmdr1;
		volatile cyg_uint16 gtmdr2;
		volatile cyg_uint16 gtrfr1;
		volatile cyg_uint16 gtrfr2;
		volatile cyg_uint16 gtcpr1;
		volatile cyg_uint16 gtcpr2;
		volatile cyg_uint16 gtcnr1;
		volatile cyg_uint16 gtcnr2;
		volatile cyg_uint16 gtmdr3;
		volatile cyg_uint16 gtmdr4;
		volatile cyg_uint16 gtrfr3;
		volatile cyg_uint16 gtrfr4;
		volatile cyg_uint16 gtcpr3;
		volatile cyg_uint16 gtcpr4;
		volatile cyg_uint16 gtcnr3;
		volatile cyg_uint16 	gtcnr4;
		volatile cyg_uint16 	gtevr1;
		volatile cyg_uint16 	gtevr2;
		volatile cyg_uint16 	gtevr3;
		volatile cyg_uint16 	gtevr4;

		volatile cyg_uint16 	gtpsr1;
		volatile cyg_uint16 		gtpsr2;
		volatile cyg_uint16 		gtpsr3;
		volatile cyg_uint16 		gtpsr4;

		volatile cyg_uint8 	reserved15[0xC0];



	}global_timers_module[2];

	/* Integrated Programmable Interrupt Controller (IPIC) */
	volatile cyg_uint32 sicfr;
	volatile cyg_uint32 sivcr;
	volatile cyg_uint32 sipnr_h;
	volatile cyg_uint32 sipnr_l;
	volatile cyg_uint32 siprr_a;
	volatile cyg_uint8 	reserved16[0x8];
	volatile cyg_uint32 siprr_d;
	volatile cyg_uint32 simsr_h;
	volatile cyg_uint32 simsr_l;
	volatile cyg_uint32 sicnr;
	volatile cyg_uint32 sepnr;
	volatile cyg_uint32 smprr_a;
	volatile cyg_uint32 	smprr_b;
	volatile cyg_uint32 semsr;
	volatile cyg_uint32 secnr;
	volatile cyg_uint32 sersr;
	volatile cyg_uint32 sermr;
	volatile cyg_uint32 sercr;

	volatile cyg_uint8 	reserved17[0x4];




	volatile cyg_uint32 sifcr_h;
	volatile cyg_uint32 	sifcr_l;
	volatile cyg_uint32 	sefcr;
	volatile cyg_uint32 serfr;
	volatile cyg_uint32 scvcr;
	volatile cyg_uint32 smvcr;

	volatile cyg_uint8 	reserved18[0x98];








	/* System Arbiter Registers */

	volatile cyg_uint32 acr;
	volatile cyg_uint32 atr;
	volatile cyg_uint8 Reserved19[0x4];
	volatile cyg_uint32 aer;
	volatile cyg_uint32 aidr;
	volatile cyg_uint32 amr;
	volatile cyg_uint32 aeatr;
	volatile cyg_uint32 aeadr;
	volatile cyg_uint32 aerr;

	volatile cyg_uint8 Reserved34[0xDC];

	/* Reset Module */
	volatile cyg_uint32 rcwlr;
	volatile cyg_uint32 rcwhr;


	volatile cyg_uint8 Reserved20[0x8]; /* SACHIN..... Location 1 ... It could be that the remaining bytes
					      could have been added using padding */

	volatile cyg_uint32 rsr;
	volatile cyg_uint32 rmr;
	volatile cyg_uint32 rpr;
	volatile cyg_uint32 rcr;
	volatile cyg_uint32 rcer;

	volatile cyg_uint8 Reserved21[0xDC]; /* SACHIN ... Like in this case missing bytes are 4, so i am not sure y is it so */


	/* Clock Module */


	volatile cyg_uint32 spmr;
	volatile cyg_uint32 occr;
	volatile cyg_uint32 sccr;
	volatile cyg_uint8 Reserved22[0xF4];

	/*Power Management Control Module */


	volatile cyg_uint32 pmccr;

	volatile cyg_uint32 pmcer;
	volatile cyg_uint32 pmcmr;
	volatile cyg_uint32 pmccr1;


	volatile cyg_uint32 pmccr2;
	volatile cyg_uint8 reserved23[0xEC];/* SACHIN..... Location 1 ... It could be that the remaining bytes
					      could have been added using padding */


	/* GPIO Registers */


	volatile cyg_uint32 gp1dir;
	volatile cyg_uint32 gp1dr;
	volatile cyg_uint32 gp1dat;
	volatile cyg_uint32 gp1ier;
	volatile cyg_uint32 gp1imr	;
	volatile cyg_uint32 gp1icr;
	volatile cyg_uint8 Reserved24[0x2E7];
	volatile cyg_uint8 missing[0x400];
	volatile cyg_uint8 Reserved25[0xD00];


	/* DDR Memory Controller Memory Map */

	volatile cyg_uint32 cs0_bnds;
	volatile cyg_uint8 Reserved26[0x4];
	volatile cyg_uint32 cs1_bnds;

	volatile cyg_uint8 Reserved27[0x4];

	volatile cyg_uint8 Reserved28[0x70];


	volatile cyg_uint32 cs0_config;
	volatile cyg_uint32 cs1_config;

	volatile cyg_uint8 Reserved29[0x78];

	volatile cyg_uint32 timing_cfg_3;
	volatile cyg_uint32 timing_cfg_0;
	volatile cyg_uint32 timing_cfg_1;
	volatile cyg_uint32 timing_cfg_2;

	volatile cyg_uint32 ddr_sdram_cfg;
	volatile cyg_uint32 	ddr_sdram_cfg_2;

	volatile cyg_uint32 	ddr_sdram_mode;
	volatile cyg_uint32 	ddr_sdram_mode_2;

	volatile cyg_uint32 	ddr_sdram_md_cntl;
	volatile cyg_uint32 	ddr_sdram_interval;
	volatile cyg_uint32 	ddr_data_init;


	volatile cyg_uint8 Reserved30[0x4];

	volatile cyg_uint32 	ddr_sdram_clk_cntl;


	volatile cyg_uint8 Reserved31[0x14];

	volatile cyg_uint32 	ddr_init_address;
	volatile cyg_uint8 Reserved32[0xAAC];

	volatile cyg_uint32 	ddr_ip_rev1;
	volatile cyg_uint32 	ddr_ip_rev2;
	volatile cyg_uint8 Reserved33[0x400];


	/*I2C Controller*/










	struct i2c_controller {
		volatile cyg_uint8 i2c1adr;
		volatile cyg_uint8 Reserved35[0x3];

		volatile cyg_uint8 i2c1fdr;
		volatile cyg_uint8 Reserved36[0x3];

		volatile cyg_uint8 i2c1cr;
		volatile cyg_uint8 Reserved37[0x3];

		volatile cyg_uint8 i2c1sr;
		volatile cyg_uint8 Reserved38[0x3];

		volatile cyg_uint8 i2c1dr;
		volatile cyg_uint8 Reserved39[0x3];

		volatile cyg_uint8 i2c1dfsrr;
		volatile cyg_uint8 Reserved40[0xEB];


	}i2c_controller[2];

	volatile cyg_uint8 Reserved40[0x1300]; /* Includes the i2c Reserved as well */

	struct {

		union {

			volatile cyg_uint8 urbr;
			volatile cyg_uint8 uthr;
			volatile cyg_uint8 	udlb;
		};


		union {

			volatile cyg_uint8 uier;
			volatile cyg_uint8 udmb;
		};

		union {

			volatile cyg_uint8 uiir;
			volatile cyg_uint8 ufcr;
			volatile cyg_uint8 uafr;

		};




		volatile cyg_uint8 ulcr;
		volatile cyg_uint8 umcr;
		volatile cyg_uint8 ulsr;
		volatile cyg_uint8 umsr;
		volatile cyg_uint8 uscr;


		volatile cyg_uint8 Reserved41[0x8];

		volatile cyg_uint8 udsr;
		volatile cyg_uint8 Reserved42[0xEF];


	}UART[2];


	volatile cyg_uint8 Reserved43[0x8FF];

	/* Enhanced Local Bus Controller (eLBC) Registers */

	volatile cyg_uint32 	br0;
	volatile cyg_uint32   or0;

	volatile cyg_uint32 	br1;
	volatile cyg_uint32   or1;

	volatile cyg_uint32 	br2;
	volatile cyg_uint32   or2;

	volatile cyg_uint32 	br3;
	volatile cyg_uint32   or3;

	volatile cyg_uint8 Reserved44[0x48];

	volatile cyg_uint32   mar;
	volatile cyg_uint8 Reserved45[0x4];



	volatile cyg_uint32   mamr;
	volatile cyg_uint32   mbmr;
	volatile cyg_uint32   mcmr;

	volatile cyg_uint8 Reserved46[0x8];


	volatile cyg_uint32   mrtpr;
	volatile cyg_uint32   mdr;
	volatile cyg_uint8 Reserved47[0x4];

	volatile cyg_uint32   lsor;

	volatile cyg_uint8 Reserved48[0xC];


	volatile cyg_uint32   				lurt;

	volatile cyg_uint8 Reserved49[0xC];


	volatile cyg_uint32 ltesr;

	volatile cyg_uint32 ltedr;
	volatile cyg_uint32 lteir;
	volatile cyg_uint32 lteatr;
	volatile cyg_uint32 ltear;


	volatile cyg_uint8 Reserved50[0xC];

	volatile cyg_uint32 lbcr;
	volatile cyg_uint32 lcrr;

	volatile cyg_uint8 Reserved51[0x8];

	volatile cyg_uint32 fmr;
	volatile cyg_uint32 fir;
	volatile cyg_uint32 fcr;
	volatile cyg_uint32 fbar;
	volatile cyg_uint32 fpar;
	volatile cyg_uint32 fbcr;

	volatile cyg_uint8 Reserved52[0x1F08];

	/* SPI */
	volatile cyg_uint8 Reserved53[0x20];

	volatile cyg_uint32 spmode;
	volatile cyg_uint32 spie;
	volatile cyg_uint32 spim;
	volatile cyg_uint32 spcom;
	volatile cyg_uint32 spitd;
	volatile cyg_uint32 spird;

	volatile cyg_uint8 Reserved54[0xFF8];
	/* DMA Registers  */


	volatile cyg_uint32 omisr;
	volatile cyg_uint32 omimr;

	volatile cyg_uint8 Reserved55[0x18];

	volatile cyg_uint32 	imr0;
	volatile cyg_uint32 	imr1;
	volatile cyg_uint32 	omr0;
	volatile cyg_uint32 	omr1;
	volatile cyg_uint32 	odr;

	volatile cyg_uint8 Reserved56[0x4];


	volatile cyg_uint32 		idr;


	volatile cyg_uint8 Reserved57[0x14];

	volatile cyg_uint32 		imisr;
	volatile cyg_uint32 		imimr;

	volatile cyg_uint8 Reserved58[0x78];


	volatile cyg_uint32 		dmamr0;
	volatile cyg_uint32 		dmasr0;
	volatile cyg_uint32 		dmacdar0;

	volatile cyg_uint8 Reserved59[0x4];


	volatile cyg_uint32 		dmasar0;
	volatile cyg_uint8 Reserved60[0x4];

	volatile cyg_uint32 		dmadar0;
	volatile cyg_uint8 Reserved61[0x4];

	volatile cyg_uint32 		dmabcr0;
	volatile cyg_uint32 DMANDAR0;



	volatile cyg_uint8 Reserved62[0x58];


	volatile cyg_uint32		dmamr1;
	volatile cyg_uint32		dmasr1;
	volatile cyg_uint32		dmacdar1;

	volatile cyg_uint8 Reserved63[0x4];


	volatile cyg_uint32		dmasar1;
	volatile cyg_uint8 Reserved64[0x4];

	volatile cyg_uint32		dmadar1;
	volatile cyg_uint8 Reserved65[0x4];

	volatile cyg_uint32		dmabcr1;
	volatile cyg_uint32 dmandar1;



	volatile cyg_uint8 Reserved66[0x58];

	volatile cyg_uint32 		dmamr2;
	volatile cyg_uint32 		dmasr2;
	volatile cyg_uint32 		dmacdar2;

	volatile cyg_uint8 Reserved67[0x4];


	volatile cyg_uint32 		dmasar2;
	volatile cyg_uint8 Reserved68[0x4];

	volatile cyg_uint32 		dmadar2;
	volatile cyg_uint8 Reserved69[0x4];

	volatile cyg_uint32 		dmabcr2;
	volatile cyg_uint32 DMANDAR2;



	volatile cyg_uint8 Reserved70[0x58];



	volatile cyg_uint32		dmamr3;
	volatile cyg_uint32		dmasr3;
	volatile cyg_uint32		dmacdar3;

	volatile cyg_uint8 Reserved71[0x4];


	volatile cyg_uint32		dmasar3;
	volatile cyg_uint8 Reserved72[0x4];

	volatile cyg_uint32		dmadar3;
	volatile cyg_uint8 Reserved73[0x4];

	volatile cyg_uint32		dmabcr3;
	volatile cyg_uint32 DMANDAR3;




	volatile cyg_uint32 dmagsr;

	volatile cyg_uint8 Reserved74[0x54];






	/* PCI SOFTWARE CONFIGURATION REGISTERS */
	volatile cyg_uint32	pci_config_address;
	volatile cyg_uint32	pci_config_data;
	volatile cyg_uint32	pci_int_ack;
	volatile cyg_uint8 Reserved75[0X74];
	volatile cyg_uint32	pcipmr0;
	volatile cyg_uint32	pcipmr1;
	volatile cyg_uint8 Reserved76[0X78];




	/* Sequencer (IOS) */


	struct {

		volatile cyg_uint32	potar;
		volatile cyg_uint8 Reserved77[0X4];

		volatile cyg_uint32	pobar;
		volatile cyg_uint8 Reserved78[0X4];

		volatile cyg_uint32	pocmr;
		volatile cyg_uint8 Reserved79[0X4];


	} ios_array[6];


	volatile cyg_uint8 Reserved80[0x60];
	volatile cyg_uint32	pmcr;
	volatile cyg_uint8 Reserved81[0x4];
	volatile cyg_uint32	dtcr;



	volatile cyg_uint8 Reserved82[0x4];

	/* PCI Error Management Registers */

	volatile cyg_uint32	pci_esr;
	volatile cyg_uint32	pci_ecdr;
	volatile cyg_uint32	pci_eer;
	volatile cyg_uint32	pci_eatcr;
	volatile cyg_uint32	pci_eacr;
	volatile cyg_uint32	pci_eeacr;
	volatile cyg_uint32	pci_edcr;

	volatile cyg_uint8 Reserved83[0x4];

	/* PCI Control and Status Registers */

	volatile cyg_uint32		pci_gcr;
	volatile cyg_uint32		pci_ecr;
	volatile cyg_uint32		pci_gsr;

	volatile cyg_uint8 Reserved84[0xc];





	/* PCI Inbound ATU Registers */
	struct {
		volatile cyg_uint32			pitar2;
		volatile cyg_uint8			Reserved85[0x4];
		volatile cyg_uint32			pibar2;
		volatile cyg_uint32			piebar2;
		volatile cyg_uint32			piwar2;

		volatile cyg_uint8 Reserved86[0x4]; /* See the Reserved next to PIWAR0,
						      4 bytes will be covered here,
						      NOt sure if this is right but can reallign it later
						      as it is also Reserved*/
	}Pci_register_array[3];

	volatile cyg_uint8 	Reserved87[0x1AA80];

	/* USB DR Controller Registers */



	volatile cyg_uint8	Reserved88[0x100];

	volatile cyg_uint8	caplength;
	volatile cyg_uint8	Reserved89;
	volatile cyg_uint16 		hciversion;
	volatile cyg_uint32			hcsparams;
	volatile cyg_uint32			hccparams;
	volatile cyg_uint8	Reserved90[0x14];

	volatile cyg_uint16 		dciversion;
	volatile cyg_uint8	Reserved91[0x2];
	volatile cyg_uint32			dccparams;



	volatile cyg_uint8	Reserved92[0x18];



	volatile cyg_uint32				usbcmd;
	volatile cyg_uint32			 usbsts;
	volatile cyg_uint32			usbintr;
	volatile cyg_uint32			frindex;
	volatile cyg_uint8	Reserved93[0x4];

	union {
		volatile cyg_uint32			periodiclistbase;
		volatile cyg_uint32			deviceaddr;
	};

	union {

		volatile cyg_uint32			asynclistaddr;
		volatile cyg_uint32			endpointaddr;


	};
	volatile cyg_uint8	Reserved94[0x4];


	volatile cyg_uint32			burstsize;
	volatile cyg_uint32			txfilltuning;
	volatile cyg_uint8	Reserved95[0x18];



	volatile cyg_uint32			configflag;
	volatile cyg_uint32			portsc1;


	volatile cyg_uint8	Reserved96[0x1c];



	volatile cyg_uint32			otgsc;
	volatile cyg_uint32			usbmode;
	volatile cyg_uint32			endptsetupstat;
	volatile cyg_uint32			endpointprime;
	volatile cyg_uint32			endptflush;
	volatile cyg_uint32			endptstatus;
	volatile cyg_uint32			endptcomplete;
	volatile cyg_uint32			endptctrl0;
	volatile cyg_uint32			endptctrl1	;

	volatile cyg_uint32			endptctrl2;

	volatile cyg_uint8	Reserved97[0x234];



	volatile cyg_uint32			snoop1;
	volatile cyg_uint32			snoop2;
	volatile cyg_uint32			age_cnt_thresh;
	volatile cyg_uint32			pri_ctrl;
	volatile cyg_uint32			si_ctrl;

	volatile cyg_uint8	Reserved98[0xEC];

	volatile cyg_uint32			control;


	volatile cyg_uint8	Reserved99[0xafc];


	struct {
		volatile cyg_uint32			tsec_id;
		volatile cyg_uint32			tsec_id2;
		volatile cyg_uint8	Reserved100[0x8];


		volatile cyg_uint32			ievent;
		volatile cyg_uint32			imask;
		volatile cyg_uint32			edis;

		volatile cyg_uint8	Reserved101[0x4];
		volatile cyg_uint32			ecntrl;

		volatile cyg_uint8	Reserved102[0x4];
		volatile cyg_uint32			 ptv;
		volatile cyg_uint32			 dmactrl;
		volatile cyg_uint8			 Reserved103[0xd0];


		volatile cyg_uint32			tctrl;
		volatile cyg_uint32			tstat;
		volatile cyg_uint32			dfvlan;
		volatile cyg_uint8			 Reserved104[0x4];

		volatile cyg_uint32			txic;
		volatile cyg_uint32			tqueue;
		volatile cyg_uint8			Reserved105[0x28];





		volatile cyg_uint32			tr03wt;
		volatile cyg_uint32			tr47wt;

		volatile cyg_uint8			Reserved106[0x38];

		volatile cyg_uint32			tbdbph;
		volatile cyg_uint32			tbptr0;
		volatile cyg_uint8			 Reserved107[0x4];

		volatile cyg_uint32			tbptr1;
		volatile cyg_uint8			 Reserved108[0x4];

		volatile cyg_uint32			tbptr2;
		volatile cyg_uint8			 Reserved109[0x4];

		volatile cyg_uint32			tbptr3;
		volatile cyg_uint8			 Reserved110[0x4];

		volatile cyg_uint32			tbptr4;
		volatile cyg_uint8			 Reserved111[0x4];

		volatile cyg_uint32			tbptr5;
		volatile cyg_uint8			 Reserved112[0x4];

		volatile cyg_uint32			tbptr6;
		volatile cyg_uint8			 Reserved113[0x4];

		volatile cyg_uint32			tbptr7;


		volatile cyg_uint8			 Reserved114[0x40];


		volatile cyg_uint32			 tbaseh;
		volatile cyg_uint32			 tbase0;
		volatile cyg_uint8			 Reserved115[0x4];

		volatile cyg_uint32			 tbase1;
		volatile cyg_uint8			 Reserved116[0x4];

		volatile cyg_uint32			 tbase2;
		volatile cyg_uint8			 Reserved117[0x4];

		volatile cyg_uint32			 tbase3;
		volatile cyg_uint8			 Reserved118[0x4];

		volatile cyg_uint32			 tbase4;
		volatile cyg_uint8			 Reserved119[0x4];

		volatile cyg_uint32			 tbase5;
		volatile cyg_uint8			 Reserved120[0x4];


		volatile cyg_uint32	tbase6;
		volatile cyg_uint8			 Reserved121[0x4];

		volatile cyg_uint32	tbase7;

		volatile cyg_uint8			 Reserved122[0x40];



		volatile cyg_uint32	tmr_txts1_id;
		volatile cyg_uint32	tmr_txts2_id;

		volatile cyg_uint8			 Reserved123[0x38];



		volatile cyg_uint32		 		tmr_txts1_h;
		volatile cyg_uint32				 tmr_txts1_l;
		volatile cyg_uint32				 tmr_txts2_h;
		volatile cyg_uint32				 tmr_txts2_l;
		volatile cyg_uint8			 Reserved124[0x30];



		/*eTSEC1 Receive Control and Status Registers*/


		volatile cyg_uint32				 rctrl;
		volatile cyg_uint32				 rstat;

		volatile cyg_uint8			 Reserved125[0x8];


		volatile cyg_uint32				 rxic;
		volatile cyg_uint32				 rqueue;
		volatile cyg_uint8			 Reserved126[0x18];

		volatile cyg_uint32				 rbifx;
		volatile cyg_uint32				 rqfar;
		volatile cyg_uint32				 rqfcr;
		volatile cyg_uint32				 rqfpr;
		volatile cyg_uint32				 mrblr;


		volatile cyg_uint8			 Reserved127[0x3C];


		volatile cyg_uint32				 rbdbph;
		volatile cyg_uint32				 rbptr0;
		volatile cyg_uint8			 Reserved128[0x4];


		volatile cyg_uint32				 rbptr1;


		volatile cyg_uint8			 Reserved129[0x4];

		volatile cyg_uint32				 rbptr2;
		volatile cyg_uint8			 Reserved130[0x4];

		volatile cyg_uint32				 rbptr3;
		volatile cyg_uint8			 Reserved131[0x4];

		volatile cyg_uint32				 rbptr4;
		volatile cyg_uint8			 Reserved132[0x4];

		volatile cyg_uint32				 rbptr5;
		volatile cyg_uint8			 Reserved133[0x4];

		volatile cyg_uint32				 rbptr6;
		volatile cyg_uint8			 Reserved134[0x4];

		volatile cyg_uint32				 rbptr7;

		volatile cyg_uint8			 Reserved135[0x40];



		volatile cyg_uint32				 rbaseh;
		volatile cyg_uint32				  rbase0;
		volatile cyg_uint8			 Reserved136[0x4];
		volatile cyg_uint32				  rbase1;
		volatile cyg_uint8			 Reserved137[0x4];
		volatile cyg_uint32				  rbase2;
		volatile cyg_uint8			 Reserved138[0x4];
		volatile cyg_uint32				  rbase3;
		volatile cyg_uint8			 Reserved139[0x4];
		volatile cyg_uint32				  rbase4;
		volatile cyg_uint8			 Reserved140[0x4];
		volatile cyg_uint32				  rbase5;
		volatile cyg_uint8			 Reserved141[0x4];
		volatile cyg_uint32				  rbase6;
		volatile cyg_uint8			 Reserved142[0x4];
		volatile cyg_uint32				  rbase7;



		volatile cyg_uint8			 Reserved143[0x80];
		volatile cyg_uint32				tmr_rxts_h;
		volatile cyg_uint32				tmr_rxts_l;

		volatile cyg_uint8			 Reserved144[0x38];


		/* eTSEC1 MAC Registers */



		volatile cyg_uint32				maccfg1;
		volatile cyg_uint32				 maccfg2;
		volatile cyg_uint32				 ipgifg;
		volatile cyg_uint32				 hafdup;
		volatile cyg_uint32				 maxfrm;

		volatile cyg_uint8			 Reserved145[0xc];

		volatile cyg_uint32				 miimcfg;
		volatile cyg_uint32				 miimcom;
		volatile cyg_uint32				 miimadd;
		volatile cyg_uint32				 miimcon;
		volatile cyg_uint32				 miimstat;
		volatile cyg_uint32				 miimind;

		volatile cyg_uint8			 Reserved146[0x4];


		volatile cyg_uint32				 ifstat;
		volatile cyg_uint32				 macstnaddr1;
		volatile cyg_uint32				 macstnaddr2;




		volatile cyg_uint32				 mac01addr1;

		volatile cyg_uint32				  mac01addr2;
		volatile cyg_uint32				  mac02addr1;
		volatile cyg_uint32				  mac02addr2;
		volatile cyg_uint32				  mac03addr1;
		volatile cyg_uint32				  mac03addr2;
		volatile cyg_uint32				  mac04addr1;
		volatile cyg_uint32				  mac04addr2;
		volatile cyg_uint32				  mac05addr1;
		volatile cyg_uint32				  mac05addr2;
		volatile cyg_uint32				  mac06addr1;
		volatile cyg_uint32				  mac06addr2;
		volatile cyg_uint32				  mac07addr1;
		volatile cyg_uint32				  mac07addr2;
		volatile cyg_uint32				  mac08addr1;
		volatile cyg_uint32				  mac08addr2;
		volatile cyg_uint32				  mac09addr1;
		volatile cyg_uint32				  mac09addr2;
		volatile cyg_uint32				  mac10addr1;
		volatile cyg_uint32				  mac10addr2;
		volatile cyg_uint32				  mac11addr1;
		volatile cyg_uint32				  mac11addr2;
		volatile cyg_uint32				  mac12addr1;
		volatile cyg_uint32				  mac12addr2;
		volatile cyg_uint32				  mac13addr1;
		volatile cyg_uint32				  mac13addr2;
		volatile cyg_uint32				  mac14addr1;
		volatile cyg_uint32				  mac14addr2;
		volatile cyg_uint32				  mac15addr1;
		volatile cyg_uint32				  mac15addr2;

		volatile cyg_uint8			 Reserved147[0xC0];

		/* eTSEC1 Transmit and Receive Counters */


		volatile cyg_uint32				  tr64;

		volatile cyg_uint32	tr127;
		volatile cyg_uint32 tr255;
		volatile cyg_uint32 tr511;
		volatile cyg_uint32 tr1k;
		volatile cyg_uint32 trmax;
		volatile cyg_uint32 trmgv;



		/*  eTSEC1 Receive Counters */

		volatile cyg_uint32 rbyt;
		volatile cyg_uint32  rpkt;
		volatile cyg_uint32  rfcs;
		volatile cyg_uint32  rmca;
		volatile cyg_uint32  rbca;
		volatile cyg_uint32  rxcf;
		volatile cyg_uint32  rxpf;
		volatile cyg_uint32  rxuo;
		volatile cyg_uint32  raln;
		volatile cyg_uint32  rflr;
		volatile cyg_uint32  rcde;
		volatile cyg_uint32  rcse;
		volatile cyg_uint32  rund;
		volatile cyg_uint32  rovr;
		volatile cyg_uint32  rfrg;
		volatile cyg_uint32  rjbr;
		volatile cyg_uint32  rdrp;


		/* eTSEC1 Transmit Counters */


		volatile cyg_uint32  tbyt;
		volatile cyg_uint32   tpkt;
		volatile cyg_uint32   tmca;
		volatile cyg_uint32   tbca;
		volatile cyg_uint32   txpf;

		volatile cyg_uint32  tdfr;
		volatile cyg_uint32   tedf;
		volatile cyg_uint32   tscl;
		volatile cyg_uint32   tmcl;
		volatile cyg_uint32   tlcl;
		volatile cyg_uint32   txcl;
		volatile cyg_uint32  tncl;


		volatile cyg_uint8			 Reserved148[0x4];


		volatile cyg_uint32 tdrp;
		volatile cyg_uint32  tjbr;
		volatile cyg_uint32  tfcs;
		volatile cyg_uint32  txcf;
		volatile cyg_uint32  tovr;
		volatile cyg_uint32  tund;
		volatile cyg_uint32 tfrg;

		/* eTSEC1 Counter Control and TOE Statistics Registers */
		volatile cyg_uint32 car1;
		volatile cyg_uint32  car2;
		volatile cyg_uint32  cam1;
		volatile cyg_uint32  cam2;
		volatile cyg_uint32  rrej;

		volatile cyg_uint8			 Reserved149[0xbc];


		/* eTSEC1 Hash Function Registers */

		volatile cyg_uint32  igaddr0;
		volatile cyg_uint32   igaddr1;
		volatile cyg_uint32   igaddr2;
		volatile cyg_uint32   igaddr3;
		volatile cyg_uint32   igaddr4;
		volatile cyg_uint32   igaddr5;
		volatile cyg_uint32   igaddr6;
		volatile cyg_uint32   igaddr7;

		volatile cyg_uint8			 Reserved150[0x60];


		volatile cyg_uint32   gaddr0;
		volatile cyg_uint32    gaddr1;
		volatile cyg_uint32    gaddr2;
		volatile cyg_uint32    gaddr3;
		volatile cyg_uint32    gaddr4;
		volatile cyg_uint32    gaddr5;
		volatile cyg_uint32    gaddr6;
		volatile cyg_uint32    gaddr7;


		volatile cyg_uint8			 Reserved151[0x260];

		/* eTSEC1 DMA Attribute Registers */
		volatile cyg_uint8			 Reserved152[0xF8];
		volatile cyg_uint32  attr;
		volatile cyg_uint32  attreli;

		/* eTSEC1 Future Expansion Space */
		volatile cyg_uint8			 Reserved153[0x200];

		volatile cyg_uint32  tmr_ctrl;
		volatile cyg_uint32   tmr_tevent;
		volatile cyg_uint32   tmr_temask;
		volatile cyg_uint32   tmr_pevent;
		volatile cyg_uint32   tmr_pemask;
		volatile cyg_uint32   tmr_stat;
		volatile cyg_uint32   tmr_cnt_h;
		volatile cyg_uint32   tmr_cnt_l;
		volatile cyg_uint32   tmr_add;
		volatile cyg_uint32   tmr_acc;
		volatile cyg_uint32   tmr_prsc;
		volatile cyg_uint8			 Reserved155[0x4];
		volatile cyg_uint32   tmr_off_h;
		volatile cyg_uint32   tmr_off_l;

		volatile cyg_uint8			 Reserved156[0x8];

		volatile cyg_uint32   tmr_alarm1_h;
		volatile cyg_uint32    tmr_alarm1_l;
		volatile cyg_uint32    tmr_alarm2_h;
		volatile cyg_uint32    tmr_alarm2_l;

		volatile cyg_uint8			 Reserved157[0x30];

		volatile cyg_uint32   tmr_fiper1;
		volatile cyg_uint32    tmr_fiper2;
		volatile cyg_uint32    tmr_fiper;
		volatile cyg_uint8			 Reserved158[0x14];

		volatile cyg_uint32    tmr_etts1_h;
		volatile cyg_uint32     tmr_etts1_l;
		volatile cyg_uint32     tmr_etts2_h;
		volatile cyg_uint32     tmr_etts2_l;

		volatile cyg_uint8			 Reserved159[0x150];
	} etsec[2];

	volatile cyg_uint8			 Reserved190[0xA000];
	volatile cyg_uint8			 Reserved160[0x1008];

	volatile cyg_uint64	imr;
	volatile cyg_uint64 isr;
	volatile cyg_uint64 icr;
	volatile cyg_uint64 id;
	volatile cyg_uint64 euasr;
	volatile cyg_uint64 mcr;


	volatile cyg_uint8			 Reserved161[0xd0];



	volatile cyg_uint64		cccr;
	volatile cyg_uint64		ccpsr;

	volatile cyg_uint8			 Reserved162[0x28];


	volatile cyg_uint64		cdpr;
	volatile cyg_uint64		FF;



	volatile cyg_uint8			 Reserved163[0x30];
	volatile cyg_uint64		db[8];

	volatile cyg_uint8			 Reserved164[0xa38];

	volatile cyg_uint64		ip_block_revision;

	/*	Data Encryption Standard Execution Unit (DEU) 		*/

	volatile cyg_uint8			 Reserved165[0x400];


	volatile cyg_uint64	deumr;
	volatile cyg_uint64	 deuksr;
	volatile cyg_uint64	 deudsr;
	volatile cyg_uint64	 deurcr;
	volatile cyg_uint8			 Reserved192[0x8];

	volatile cyg_uint64	 deusr;
	volatile cyg_uint64	 deuisr;
	volatile cyg_uint64	 deuicr;

	volatile cyg_uint8			 Reserved166[0x10];

	volatile cyg_uint64	 deuemr;

	volatile cyg_uint8			 Reserved167[0xA8];
	volatile cyg_uint64	 deuiv;

	volatile cyg_uint8			 Reserved193[0x2f8];


	volatile cyg_uint64	 deuk1;
	volatile cyg_uint64	  deuk2;
	volatile cyg_uint64	  deuk3;

	volatile cyg_uint8			 Reserved168[0x3E8];

	volatile cyg_uint64	  deu;

	volatile cyg_uint8			 Reserved169[0x17F8];

	/* Advanced Encryption Standard Execution Unit (AESU) */

	volatile cyg_uint64	  aesumr;
	volatile cyg_uint64	  aesuksr;
	volatile cyg_uint64	   aesudsr;
	volatile cyg_uint64	   aesurcr;

	volatile cyg_uint8			 Reserved170[0x8];

	volatile cyg_uint64	   aesusr;
	volatile cyg_uint64	    aesuisr;
	volatile cyg_uint64	    aesuicr;


	volatile cyg_uint8			 Reserved171[0x10];

	volatile cyg_uint64	  aesuemr;
	volatile cyg_uint8			 Reserved172[0xA8];
	volatile cyg_uint64	  aesu_cmr;
	volatile cyg_uint8			 Reserved173[0x2F8];
	volatile cyg_uint64	  	aesu_kmem;
	volatile cyg_uint8			 Reserved174[0x3F8];
	volatile cyg_uint64	  	aesu_fifo;


	volatile cyg_uint8			 Reserved175[0x17F8];

	/*	Message Digest Execution Unit (MDEU) */
	volatile cyg_uint64	  	mdeumr;
	volatile cyg_uint64	  	 mdeuksr;
	volatile cyg_uint64	  	 mdeudsr;
	volatile cyg_uint64	  	 mdeurcr;


	volatile cyg_uint8			 Reserved176[0x8];

	volatile cyg_uint64	  	 mDeusr;
	volatile cyg_uint64	  	  mdeuisr;
	volatile cyg_uint64	  	  mdeuicr;
	volatile cyg_uint64	  	  	mdeu_icvsreg;

	volatile cyg_uint8			 Reserved177[0x8];
	volatile cyg_uint64	  	  	mdeuemr;

	volatile cyg_uint8			 Reserved178[0xA8];
	volatile cyg_uint64	  	  	mdeu_cmr;


	volatile cyg_uint8			 Reserved179[0x2F8];
	volatile cyg_uint64	  	  	mdeu_kmem;

	volatile cyg_uint8			 Reserved180[0x3F8];

	volatile cyg_uint64 	mdeu_fifo;

} IMMR;



externC void _mpc83xx_reset_cpm(void);
externC unsigned int _mpc83xx_allocBd(int len);

#endif // ifndef CYGONCE_HAL_PPC_QUICC2_MPC83XX_H
