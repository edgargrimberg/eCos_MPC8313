//==========================================================================
//
//      fec.h
//
//      PowerPC MPC8xxT fast ethernet (FEC)
//
//==========================================================================
/// ####ECOSGPLCOPYRIGHTBEGIN####
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
// Author(s):    gthomas
// Contributors: gthomas
// Date:         2001-01-21
// Purpose:
// Description:
//
//
//####DESCRIPTIONEND####
//
//==========================================================================

// PowerPC FEC (MPC8xxT) Fast Ethernet

// Buffer descriptor
struct tsec_bd {
    unsigned short  ctrl;
    unsigned short  length;
    unsigned char  *buffer;
};

// control flags differ for Rx and Tx buffers
#define FEC_BD_Rx_Empty  0x8000  // Buffer is empty [FEC can fill it]
#define FEC_BD_Rx_Wrap   0x2000  // Last buffer in ring [wrap]
#define FEC_BD_Rx_Intr   0x1000  // Interrupt
#define FEC_BD_Rx_Last   0x0800  // Last buffer in frame
#define FEC_BD_Rx_Miss   0x0100  //
#define FEC_BD_Rx_BC     0x0080
#define FEC_BD_Rx_MC     0x0040
#define FEC_BD_Rx_LG     0x0020
#define FEC_BD_Rx_NO     0x0010
#define FEC_BD_Rx_SH     0x0008  // Short frame
#define FEC_BD_Rx_CR     0x0004  // CRC error
#define FEC_BD_Rx_OV     0x0002  // Overrun
#define FEC_BD_Rx_TR     0x0001  // Frame truncated

#define FEC_BD_Tx_Ready  0x8000  // Frame ready
#define FEC_BD_Tx_Wrap   0x2000  // Last buffer in ring
#define FEC_BD_Tx_Intr   0x1000  // Generate interrupt
#define FEC_BD_Tx_Last   0x0800  // Last buffer in frame
#define FEC_BD_Tx_TC     0x0400  // Send CRC after data
#define FEC_BD_Tx_DEF    0x0200
#define FEC_BD_Tx_HB     0x0100
#define FEC_BD_Tx_LC     0x0080
#define FEC_BD_Tx_RL     0x0040
#define FEC_BD_Tx_RC     0x003C
#define FEC_BD_Tx_UN     0x0002  // Underrun
#define FEC_BD_Tx_CSL    0x0001  // Carrier sense lost

#define FEC_BD_Tx_STATS  0x03FF  // Status mask

struct tsec_eth_info {
    volatile struct mpq_tsec    *tsec;
    volatile struct tsec_bd     *txbd, *rxbd;     // Next Tx,Rx descriptor to use
    volatile struct tsec_bd     *tbase, *rbase;   // First Tx,Rx descriptor
    volatile struct tsec_bd     *rnext;   // Next descriptor to check for interrupt
    int                         txsize, rxsize;  // Length of individual buffers
    unsigned long               txkey[CYGNUM_DEVS_ETH_POWERPC_TSEC_TxNUM];
    cyg_uint32 phyAddress;						//the PHY address
};

// MIB specific registers
struct rmon_tsec
{
	cyg_uint32 tr64;
	cyg_uint32 tr127;
	cyg_uint32 tr255;
	cyg_uint32 tr511;
	cyg_uint32 tr1k;
	cyg_uint32 trmax;
	cyg_uint32 trmgv;
	cyg_uint32 rbyt;
	cyg_uint32 rpkt;
	cyg_uint32 rfcs;
	cyg_uint32 rmca;
	cyg_uint32 rbca;
	cyg_uint32 rxcf;
	cyg_uint32 rxpf;
	cyg_uint32 rxuo;
	cyg_uint32 raln;
	cyg_uint32 rflr;
	cyg_uint32 rcde;
	cyg_uint32 rcse;
	cyg_uint32 rund;
	cyg_uint32 rovr;
	cyg_uint32 rfrg;
	cyg_uint32 rjbr;
	cyg_uint32 rdrp;
	cyg_uint32 tbyt;
	cyg_uint32 tpkt;
	cyg_uint32 tmca;
	cyg_uint32 tbca;
	cyg_uint32 txpf;
	cyg_uint32 tdfr;
	cyg_uint32 tedf;
	cyg_uint32 tscl;
	cyg_uint32 tmcl;
	cyg_uint32 tlcl;
	cyg_uint32 txcl;
	cyg_uint32 tncl;
	cyg_uint8 _reserved20[0x04];
	cyg_uint32 tdrp;
	cyg_uint32 tjbr;
	cyg_uint32 tfcs;
	cyg_uint32 txcf;
	cyg_uint32 tovr;
	cyg_uint32 tund;
	cyg_uint32 tfrg;
	cyg_uint32 car1;
	cyg_uint32 car2;
	cyg_uint32 cam1;
	cyg_uint32 cam2;
};

// Fast Ethernet Controller [in PPC8xxT parameter RAM space]
struct mpq_tsec
{
	cyg_uint8 _reserved0[0x10];
	cyg_uint32 ievent;
	cyg_uint32 imask;
	cyg_uint32 edis;
	cyg_uint8 _reserved1[0x04];
	cyg_uint32 ecntrl;
	cyg_uint32 minflr;
	cyg_uint32 ptv;
	cyg_uint32 dmactrl;
	cyg_uint32 tbipa;
	cyg_uint8 _reserved2[0x58];
	cyg_uint32 fifo_tx_thr;
	cyg_uint8 _reserved3[0x08];
	cyg_uint32 fifo_tx_starve;
	cyg_uint32 fifo_tx_starve_shutoff;
	cyg_uint8 _reserved4[0x60];
	cyg_uint32 tctrl;
	cyg_uint32 tstat;
	cyg_uint8 _reserved5[0x04];
	cyg_uint32 tbdlen;
	cyg_uint32 txic;
	cyg_uint8 _reserved6[0x10];
	cyg_uint32 ctbptr;
	cyg_uint8 _reserved7[0x5C];
	cyg_uint32 tbptr;
	cyg_uint8 _reserved8[0x7C];
	cyg_uint32 tbase;
	cyg_uint8 _reserved9[0xA8];
	cyg_uint32 ostbd;
	cyg_uint32 ostbdp;
	cyg_uint8 _reserved10[0x48];
	cyg_uint32 rctrl;
	cyg_uint32 rstat;
	cyg_uint8 _reserved11[0x04];
	cyg_uint32 rbdlen;
	cyg_uint32 rxic;
	cyg_uint8 _reserved12[0x10];
	cyg_uint32 crbptr;
	cyg_uint8 _reserved13[0x18];
	cyg_uint32 mrblr;
	cyg_uint8 _reserved14[0x40];
	cyg_uint32 rbptr;
	cyg_uint8 _reserved15[124];
	cyg_uint32 rbase;
	cyg_uint8 _reserved16[0xF8];
	cyg_uint32 maccfg1;
	cyg_uint32 maccfg2;
	cyg_uint32 ipgifgi;
	cyg_uint32 hafdup;
	cyg_uint32 maxfrm;
	cyg_uint8 _reserved17[0x0C];
	cyg_uint32 miimcfg;
	cyg_uint32 miimcom;
	cyg_uint32 miimadd;
	cyg_uint32 miimcon;
	cyg_uint32 miimstat;
	cyg_uint32 miimind;
	cyg_uint8 _reserved18[0x04];
	cyg_uint32 ifstat;
	cyg_uint32 macstnaddr1;
	cyg_uint32 macstnaddr2;
	cyg_uint8 _reserved19[0x138];
       struct rmon_tsec rmon;
	cyg_uint8 _reserved21[0xC0];
	cyg_uint32 igaddr[8];
	cyg_uint8 _reserved22[0x60];
	cyg_uint32 gaddr[8];
	cyg_uint8 _reserved23[0x358];
	cyg_uint32 attr;
	cyg_uint32 attreli;
	cyg_uint8 _reserved24[1 * 1024];
} ;

#define FEC_OFFSET 0x0E00      // Offset in 8xx parameter RAM



// MII interface
#define MII_Start            0x40000000
#define MII_Read             0x20000000
#define MII_Write            0x10000000
#define MII_Phy(phy)         (phy << 23)
#define MII_Reg(reg)         (reg << 18)
#define MII_TA               0x00020000

// Transceiver mode
#define PHY_BMCR             0x00    // Register number
#define PHY_BMCR_RESET       0x8000
#define PHY_BMCR_LOOPBACK    0x4000
#define PHY_BMCR_100MB       0x2000
#define PHY_BMCR_AUTO_NEG    0x1000
#define PHY_BMCR_POWER_DOWN  0x0800
#define PHY_BMCR_ISOLATE     0x0400
#define PHY_BMCR_RESTART     0x0200
#define PHY_BMCR_FULL_DUPLEX 0x0100
#define PHY_BMCR_COLL_TEST   0x0080

#define PHY_BMSR             0x01    // Status register
#define PHY_BMSR_AUTO_NEG    0x0020
#define PHY_BMSR_LINK        0x0004

#define IEEE_8023_MAX_FRAME         1518    // Largest possible ethernet frame
#define IEEE_8023_MIN_FRAME           60    // Smallest possible ethernet frame

/* MII Management Configuration Register (MIIMCFG) */
#define MIIMCFG_RESET			0x80000000
#define MIIMCFG_CLOCK_DIV_2		0x00000000
#define MIIMCFG_CLOCK_DIV_4		0x00000001
#define MIIMCFG_CLOCK_DIV_6		0x00000002
#define MIIMCFG_CLOCK_DIV_8		0x00000003
#define MIIMCFG_CLOCK_DIV_10	0x00000004
#define MIIMCFG_CLOCK_DIV_14	0x00000005
#define MIIMCFG_CLOCK_DIV_20	0x00000006
#define MIIMCFG_CLOCK_DIV_28	0x00000007

/* MII Management Indicator Register (MIIMIND) */
#define MIIMIND_NOT_VALID		0x00000004
#define MIIMIND_SCAN			0x00000002
#define MIIMIND_BUSY			0x00000001

/* MII Management Status Register (MIIMSTAT) */
#define MIIMSTAT_PHY_STATUS		0x0000FFFF

/* MII Management Command Register (MIIMCOM) */
#define MIIMCOM_READ			0x00000001


/* Interrupt Mask Register (IMASK) */
#define IMASK_BREN			0x80000000
#define IMASK_RXCEN			0x40000000
#define IMASK_BSYEN			0x20000000
#define IMASK_EBERREN		0x10000000
#define IMASK_MSROEN		0x04000000
#define IMASK_GTSCEN		0x02000000
#define IMASK_BTEN			0x01000000
#define IMASK_TXCEN			0x00800000
#define IMASK_TXEEN			0x00400000
#define IMASK_TXBEN			0x00200000
#define IMASK_TXFEN			0x00100000
#define IMASK_LCEN			0x00040000
#define IMASK_CRLEN			0x00020000
#define IMASK_XFUNEN		       0x00010000
#define IMASK_RXBEN			0x00008000
#define IMASK_MAG			0x00000800
#define IMASK_GRSCEN		0x00000100
#define IMASK_RXFEN			0x00000080
#define IMASK_FIR			0x00000008
#define IMASK_FIQ			0x00000004
#define IMASK_DPE			0x00000002
#define IMASK_PERR			0x00000001
#define IMASK_DEFAULT (IMASK_TXEEN | IMASK_TXBEN | IMASK_TXFEN | IMASK_RXFEN | IMASK_RXBEN | \
	IMASK_BSYEN | IMASK_EBERREN | IMASK_BREN | IMASK_XFUNEN | IMASK_RXCEN | IMASK_BTEN | IMASK_DPE | IMASK_PERR | \
	IMASK_MSROEN | IMASK_LCEN | IMASK_CRLEN)


/* DMA Control Register (DMACTRL) */
#define DMACTRL_TDSEN		0x00000080
#define DMACTRL_TBDSEN		0x00000040
#define DMACTRL_GRS			0x00000010
#define DMACTRL_GTS			0x00000008
#define DMACTRL_TOD			0x00000004
#define DMACTRL_WWR		0x00000002
#define DMACTRL_WOP			0x00000001

/* Interrupt Event Register (IEVENT) */
#define IEVENT_BABR			0x80000000
#define IEVENT_RXC			0x40000000
#define IEVENT_BSY			0x20000000
#define IEVENT_EBERR		0x10000000
#define IEVENT_MSRO			0x04000000
#define IEVENT_GTSC			0x02000000
#define IEVENT_BABT			0x01000000
#define IEVENT_TXC			0x00800000
#define IEVENT_TXE			0x00400000
#define IEVENT_TXB			0x00200000
#define IEVENT_TXF			0x00100000
#define IEVENT_LC			0x00040000
#define IEVENT_CRL			0x00020000
#define IEVENT_XFUN			0x00010000
#define IEVENT_RXBO			0x00008000
#define IEVENT_GRSC			0x00000100
#define IEVENT_RXF			0x00000080
#define IEVENT_DPE			0x00000002
#define IEVENT_PERR			0x00000001


#define ECNTRL_FIFM			0x00008000		/*FIFO mode. Not supported.*/
#define ECNTRL_CLRCNT 		0x00004000		/* Clear all statistics counters
												0 Allow MIB counters to continue to increment.
												1 Reset all MIB counters.
												This bit is self-resetting.*/
#define ECNTRL_AUTOZ 		0x00002000		/* Automatically zero MIB counter values.
												0 The user must write the addressed counter zero after a host read.
												1 The addressed counter value is automatically cleared to zero after a host read.
												This is a steady state signal and must be set prior to enabling the Ethernet controller and must not be
												changed without proper care.*/
#define ECNTRL_STEN 		0x00001000		/* MIB counter statistics enabled.
												0 Statistics not enabled
												1 Enables internal counters to update
												This is a steady state signal and must be set prior to enabling the Ethernet controller and must not be
												changed without proper care.*/
#define ECNTRL_GMIIM  		0x00000040		/* GMII interface mode. If this bit is set, a PHY with a RGMII interface is expected to be connected. If cleared,
												a PHY with an MII or RMII interface is expected. The user should then set MACCFG2[I/F Mode]
												accordingly. The state of this status bit is defined during power-on reset. See Section 4.3.2, “Reset
												Configuration Words.”
												0 MII or RMII mode interface expected
												1 RGMII mode interface expected*/
#define ECNTRL_TBIM  		0x00000020		/* Ten-bit interface mode. If this bit is set, ten-bit interface mode is enabled. This bit can be pin-configured at
												reset to set or clear. See Section 4.3.2, “Reset Configuration Words.”
												0 GMII or MII or RMII mode interface
												1 TBI mode interface*/
#define ECNTRL_RPM  		0x00000010		/* Reduced-pin mode for Gigabit interfaces. If this bit is set, a reduced-pin interface is expected on Ethernet
												interfaces. RPM and RMM are never set together. This register can be pin-configured at reset to 0 or 1.
												See Section 4.3.2, “Reset Configuration Words.”
												0 MII or TBI in non-reduced-pin mode configuration
												1 RGMII or RTBI reduced-pin mode*/
#define ECNTRL_R100M  		0x00000008		/* RGMII/RMII 100 mode. This bit is ignored unless SGMIIM, RPM or RMM are set and MACCFG2[I/F Mode]
												is assigned to 10/100 (01). If this bit is set, the eTSEC interface is in 100 Mbps speed.
												0 RGMII is in 10 Mbps mode;
												RMII is in 10 Mbps mode, and every 10th RMII Reference clock is used to transfer data
												SGMII is in 10 Mbps mode, and every 100th SGMII Reference clock is used to transfer data
												1 RGMII is in 100 Mbps mode;
												RMII is in 100 Mbps mode, and data is transferred on every Reference clock
												SGMII is in 100 Mbps mode, and every 10th SGMII Reference clock is used to transfer data*/
#define ECNTRL_RMM  		0x00000004		/* Reduced-pin mode for 10/100 interfaces. If this bit is set, a RMII pin interface is expected. Valid only if
												FIFM = 0 and TBIM = 0. RPM and RMM are never set together. This register can be pin-configured at reset
												to 0 or 1. See Section 4.3.2, “Reset Configuration Words.”
												0 MII, RGMII, TBI, or RTBI mode configuration
												1 RMII mode*/
#define ECNTRL_SGMIIM  		0x00000002		/* Serial GMII mode. If this bit is set, a SGMII pin interface is expected to be connected via an on chip
												SERDES.
												This register can be pin-configured at reset to 0 or 1. See Section 4.3.2.2.5, “eTSEC1 Mode” and
												Section 4.3.2.2.6, “eTSEC2 Mode.”
												0 SGMII mode disabled. eTSEC connected via a parallel interface.
												1 SGMII mode enabled.*/


/* MAC Configuration Register #1 (MACCFG1) */
#define MACCFG1_SOFT_RESET		0x80000000
#define MACCFG1_RESET_RX_MC	0x00080000
#define MACCFG1_RESET_TX_MC	0x00040000
#define MACCFG1_RESET_RX_FUN	0x00020000
#define MACCFG1_RESET_TX_FUN	0x00010000
#define MACCFG1_LOOPBACK		0x00000100
#define MACCFG1_RXFLOW			0x00000020
#define MACCFG1_TXFLOW			0x00000010
#define MACCFG1_SYNC_RXEN		0x00000008
#define MACCFG1_RXEN			0x00000004
#define MACCFG1_SYNC_TXEN		0x00000002
#define MACCFG1_TXEN			0x00000001


/* MAC Configuration Register #2 (MACCFG2) */
#define MACCFG2_IF_MODE_NIBBLE	0x00000100
#define MACCFG2_IF_MODE_BYTE	0x00000200
#define MACCFG2_PreAM_RxEN		0x00000080
#define MACCFG2_PreAM_TxEN		0x00000040
#define MACCFG2_HUGE_FRAME		0x00000020
#define MACCFG2_LENGTH_CHECK	0x00000010
#define MACCFG2_MPEN			0x00000008
#define MACCFG2_PAD_CRC			0x00000004
#define MACCFG2_CRC_EN			0x00000002
#define MACCFG2_FULL_DUPLEX		0x00000001


/* Attribute Register (ATTR) */
#define ATTR_RDSEN 0x00000080	/*Rx data snoop enable.
								 * 0 Disables snooping of all receive frames data to memory.
								 * 1 Enables snooping of all receive frames data to memory.
								 */
#define ATTR_RBDSEN 0x00000040	/*RxBD snoop enable.
								 * 0 Disables snooping of all receive BD memory accesses.
								 * 1 Enables snooping of all receive BD memory accesses.
								 */



/* Receive Control Register (RCTRL) */
#define RCTRL_TS    		       0x01000000
#define RCTRL_PAL_MASK		0x001f0000
#define RCTRL_RESERVED2         0x00008000
#define RCTRL_LFC		       0x00004000
#define RCTRL_VLEX		       0x00002000
#define RCTRL_FILREN		       0x00001000
#define RCTRL_GHTX		       0x00000400
#define RCTRL_IPCSEN		       0x00000200
#define RCTRL_TUCSEN		0x00000100
#define RCTRL_PRSDEP_MASK	0x000000c0
#define RCTRL_RESERVED       	0x00000020
#define RCTRL_BC_REJ	       	0x00000010
#define RCTRL_PROM			0x00000008
#define RCTRL_RSF			0x00000004
#define RCTRL_EMEN		       0x00000002

/* Transmit Control Register (TCTRL) */
#define TCTRL_IPCSEN		      0x00004000
#define TCTRL_TUCSEN		      0x00002000
#define TCTRL_VLINS	   	      0x00001000
#define TCTRL_THDF                 0x00000800
#define TCTRL_RFC_PAUSE        0x00000010
#define TCTRL_TFC_PAUSE        0x00000008
#define TCTRL_TXSCHED_MASK 0x00000006

/* Receive Status Register (RSTAT) */
#define RSTAT_QHLT			0x00800000
#define RSTAT_RXF			0x00000080

/* Transmit Status Register (TSTAT) */
#define TSTAT_THLT			0x80000000
#define TSTAT_TXF			0x00008000


#define IEVENT_CLEAR_ALL        0xFFFFFFFF
#define IMASK_CLEAR_ALL         0x00000000


