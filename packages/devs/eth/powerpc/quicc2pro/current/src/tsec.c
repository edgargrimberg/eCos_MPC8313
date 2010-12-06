//==========================================================================
//
//      dev/if_fec.c
//
//      Fast ethernet device driver for PowerPC MPC8xxT boards
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
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
// Description:  hardware driver for MPC8xxT FEC
//
//
//####DESCRIPTIONEND####
//
//==========================================================================

// Ethernet device driver for MPC8xx FEC

#include <pkgconf/system.h>
#include <pkgconf/devs_eth_powerpc_tsec.h>
#include <pkgconf/io_eth_drivers.h>
#include CYGDAT_DEVS_TSEC_ETH_INL

#ifdef CYGPKG_NET
#include <pkgconf/net.h>
#endif

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/diag.h>

#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/ppc_regs.h>

#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>

#include "tsec.h"
#include "ip101A.h"

#include <cyg/hal/var_regs.h>
#include <cyg/hal/mpc83xx.h>

/* There is some documentation that the TSEC does cache snooping, in which case we
 * shouldn't try to flush/invalidate ?
 *
 * Is the snooping done at before or after the cache?
 *
 * By setting this to 0, the descriptors & buffers will be placed in non-cached
 * RAM(only available for testing purposes in the RAM type eCos application)
 */
#define CACHE() 0
#define PLACE_IN_CACHED() 1

////debug:
//static int index_in_memory_area;
//static int stopper;
//static cyg_uint8 memory_area[1024*1024];

// Align buffers on a cache boundary
#define RxBUFSIZE CYGNUM_DEVS_ETH_POWERPC_TSEC_RxNUM*CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_RX
#define TxBUFSIZE CYGNUM_DEVS_ETH_POWERPC_TSEC_TxNUM*CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_TX

static unsigned char tsec_eth_rxbufs[RxBUFSIZE]
#if !CACHE() && !PLACE_IN_CACHED()
                                     __attribute__ ((section (".nocache")))
#endif
                                     __attribute__((aligned(HAL_DCACHE_LINE_SIZE)))
;
static unsigned char tsec_eth_txbufs[TxBUFSIZE]
#if !CACHE() && !PLACE_IN_CACHED()
                                     __attribute__ ((section (".nocache")))
#endif
                                     __attribute__((aligned(HAL_DCACHE_LINE_SIZE)))
;

static struct tsec_bd tsec_eth_rxring[CYGNUM_DEVS_ETH_POWERPC_TSEC_RxNUM]
#if !CACHE() && !PLACE_IN_CACHED()
                                     __attribute__ ((section (".nocache")))
#endif
                                      __attribute__((aligned(HAL_DCACHE_LINE_SIZE)))

;
static struct tsec_bd tsec_eth_txring[CYGNUM_DEVS_ETH_POWERPC_TSEC_TxNUM]
#if !CACHE() && !PLACE_IN_CACHED()
                                     __attribute__ ((section (".nocache")))
#endif
                                      __attribute__((aligned(HAL_DCACHE_LINE_SIZE)))

;

#define RESET_FULL_DUPLEX	0x00000001
#define RESET_100MB 		0x00000002

static struct tsec_eth_info tsec_eth0_info __attribute__((aligned(HAL_DCACHE_LINE_SIZE)));

static unsigned char _default_enaddr[] =
{ 0x00, 0x20, 0x0E, 0x10, 0x1F, 0x36 };
static unsigned char enaddr[6];
#ifdef CYGPKG_REDBOOT
#include <pkgconf/redboot.h>
#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#include <redboot.h>
#include <flash_config.h>
RedBoot_config_option("Network hardware address [MAC]",
		tsec_esa,
		ALWAYS_ENABLED, true,
		CONFIG_ESA, 0
);
#endif
#endif

#ifdef CYGPKG_POLL_NETWORKING
#include <support.h>
#endif

#ifdef CYGHWR_DEVS_ETH_POWERPC_MPC8313E_RDB_DEBUG
#define os_printf printf
#else
static inline void os_printf(const char *fmt, ...)
{
	/* do nothing */
}
#endif

// For fetching the ESA from RedBoot
#include <cyg/hal/hal_if.h>
#ifndef CONFIG_ESA
#define CONFIG_ESA 6
#endif

ETH_DRV_SC(tsec_eth0_sc,
		&tsec_eth0_info, // Driver specific data
		"eth0", // Name for this interface
		tsec_eth_start,
		tsec_eth_stop,
		tsec_eth_control,
		tsec_eth_can_send,
		tsec_eth_send,
		tsec_eth_recv,
		tsec_eth_deliver,
		tsec_eth_int,
		tsec_eth_int_vector)
;
NETDEVTAB_ENTRY(tsec_netdev,
		"tsec_eth",
		tsec_eth_init,
		&tsec_eth0_sc)
;

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
#define _TSEC_USE_INTS 1
#ifdef _TSEC_USE_INTS
static cyg_interrupt tsec_eth_interrupt_tx;
static cyg_interrupt tsec_eth_interrupt_rx;
static cyg_interrupt tsec_eth_interrupt_err;
static cyg_handle_t tsec_eth_interrupt_handle_tx;
static cyg_handle_t tsec_eth_interrupt_handle_rx;
static cyg_handle_t tsec_eth_interrupt_handle_err;
#else
#define STACK_SIZE CYGNUM_HAL_STACK_SIZE_MINIMUM
static char tsec_fake_int_stack[STACK_SIZE];
static cyg_thread tsec_fake_int_thread_data;
static cyg_handle_t tsec_fake_int_thread_handle;
static void tsec_fake_int(cyg_addrword_t);
#endif // _TSEC_USE_INTS
#endif // CYGINT_IO_ETH_INT_SUPPORT_REQUIRED

static cyg_handle_t phy_interrupt_handle;
static cyg_interrupt phy_interrupt;


static void tsec_eth_int(struct eth_drv_sc *data);

#ifndef TSEC_ETH_INT
#error  TSEC_ETH_INT must be defined
#endif

#ifndef TSEC_ETH_PHY
#error  TSEC_ETH_PHY must be defined
#endif

#ifndef FEC_EPPC_BD_OFFSET
#define FEC_EPPC_BD_OFFSET CYGNUM_DEVS_ETH_POWERPC_FEC_BD_OFFSET
#endif

#ifdef CYGSEM_DEVS_ETH_POWERPC_FEC_STATUS_LEDS
// LED activity [exclusive of hardware bits]
#ifndef _get_led
#define _get_led()
#define _set_led(v)
#endif
#ifndef LED_TxACTIVE
#define LED_TxACTIVE  7
#define LED_RxACTIVE  6
#define LED_IntACTIVE 5
#endif

static void
set_led(int bit)
{
	_set_led(_get_led() | (1<<bit));
}

static void
clear_led(int bit)
{
	_set_led(_get_led() & ~(1<<bit));
}
#else
#define set_led(b)
#define clear_led(b)
#endif

#ifdef _TSEC_USE_INTS
// This ISR is called when the ethernet interrupt occurs
static int tsec_eth_isr(cyg_vector_t vector, cyg_addrword_t data,
		HAL_SavedRegisters *regs)
{
	cyg_drv_interrupt_mask(vector);
	cyg_drv_interrupt_acknowledge(vector);
//	os_printf("TSEC ISR on interrupt %d", vector);
	return (CYG_ISR_HANDLED | CYG_ISR_CALL_DSR); // Run the DSR
}
#endif

static void resetPHY(void)
{
	volatile cyg_uint32 value = 0;
	//reset phy
	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_GP1DAT , value );
	value &= ~((cyg_uint32) 0x10);
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_GP1DAT , value );
	HAL_DELAY_US(125);
	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_GP1DAT , value );
	value |= 0x10;
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_GP1DAT , value );

	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCFG , MIIMCFG_RESET);
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCFG , MIIMCFG_CLOCK_DIV_8);

	HAL_DELAY_US(12500);

}

// Deliver function (ex-DSR) handles the ethernet [logical] processing
static void tsec_eth_deliver(struct eth_drv_sc * sc)
{
	tsec_eth_int(sc);
}

#define CYGSEM_DEVS_ETH_POWERPC_FEC_RESET_PHY 1

#ifdef CYGSEM_DEVS_ETH_POWERPC_FEC_RESET_PHY

static cyg_uint32 phy_read_register(cyg_uint32 address, cyg_uint8 reg)
{
	cyg_uint32 retVal;
	cyg_uint32 miimcom, miimind;
	int i;

	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMADD , ((((cyg_uint32)address) << 8) | reg));

	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCOM , miimcom);
	miimcom &= ~MIIMCOM_READ;
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCOM , miimcom);
	miimcom |= MIIMCOM_READ;
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCOM , miimcom);

	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMIND , miimind);
	i = 0;
	while ((miimind & MIIMIND_BUSY) == MIIMIND_BUSY && i++ < 500)
	{
//		os_printf(".");
		HAL_DELAY_US(10000);
		HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMIND , miimind);
	}
	//status register
	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMSTAT , retVal);
	//			phy_state = qi->regs->miimstat;
	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCOM , miimcom);
	miimcom &= ~MIIMCOM_READ;
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCOM , miimcom);
	return retVal;
}

static void phy_write_register(cyg_uint32 address, cyg_uint8 reg,
		cyg_uint32 value)
{
	int i = 0;
	cyg_uint32 miimind;
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMADD , ((cyg_uint32)address << 8) | reg );
	//AN advertisement register
	HAL_WRITE_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCON , value);

	HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMIND , miimind);
	while (miimind != 0 && i++ < 500)
	{
//		os_printf(".");
		HAL_DELAY_US(10000);
		HAL_READ_UINT32( CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMIND , miimind);
	}

}

static cyg_uint32 phy_isr(cyg_vector_t vector, cyg_addrword_t data)
{
	cyg_drv_interrupt_mask(vector);
	cyg_drv_interrupt_acknowledge(vector);
//	os_printf("PHY ISR on interrupt %d", vector);
	//just call the DSR
	return (CYG_ISR_CALL_DSR | CYG_ISR_HANDLED);
}

static void phy_dsr(cyg_vector_t vector, cyg_ucount32 count,
		cyg_addrword_t data)
{
	cyg_uint32 status;
	struct eth_drv_sc *sc = (struct eth_drv_sc *) data;
	cyg_uint32 address =
			((struct tsec_eth_info *) sc->driver_private)->phyAddress;
//	os_printf("PHY DSR");
	status = phy_read_register(address, MII_PHY_IRQ_REG);
	if (status & (MII_PHY_IRQ_LINK_CHANGE| MII_PHY_IRQ_DUPLEX_CHANGE
			| MII_PHY_IRQ_SPEED_CHANGE))
	{
		int esa_ok = 0;
		(sc->funs->stop)(sc);
	    // Try to read the ethernet address of the transciever ...
#ifdef CYGPKG_REDBOOT
	    esa_ok = flash_get_config("tsec_esa", enaddr, CONFIG_ESA);
#else
		esa_ok = CYGACC_CALL_IF_FLASH_CFG_OP(CYGNUM_CALL_IF_FLASH_CFG_GET,
				"tsec_esa", enaddr, CONFIG_ESA);
#endif
		if (!esa_ok)
		{
			// Can't figure out ESA
			os_printf("TSEC_ETH - Warning! ESA unknown\n");
			memcpy(&enaddr, &_default_enaddr, sizeof(enaddr));
		}

		(sc->funs->start)(sc, (unsigned char *) enaddr, 1);
	}
	cyg_drv_interrupt_unmask(vector);
}

void createPHYInterrupt(struct eth_drv_sc *sc)
{
	cyg_drv_interrupt_create(CYGNUM_HAL_INTERRUPT_IRQ2,
						0,
						(cyg_addrword_t) sc, //  Data item passed to interrupt handler
						(cyg_ISR_t *) phy_isr,
						(cyg_DSR_t *) phy_dsr,
						&phy_interrupt_handle,
						&phy_interrupt);
	cyg_drv_interrupt_attach(phy_interrupt_handle);
	cyg_drv_interrupt_acknowledge(CYGNUM_HAL_INTERRUPT_IRQ2);
	cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_IRQ2);

}

#endif // CYGSEM_DEVS_ETH_POWERPC_FEC_RESET_PHY
//
// [re]Initialize the ethernet controller
//   Done separately since shutting down the device requires a
//   full reconfiguration when re-enabling.
//   when
static bool tsec_eth_reset(struct eth_drv_sc *sc, unsigned char *enaddr,
		int flags)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct mpq_tsec *tsec =
			(volatile struct mpq_tsec *) ((unsigned char *) CYGARC_IMM_BASE
					+ CYGARC_REG_IMM_TSEC1);
	volatile struct tsec_bd *rxbd, *txbd;
	unsigned char *RxBUF, *TxBUF;
	cyg_uint32 int_state;
	int i = 0;

	HAL_DISABLE_INTERRUPTS(int_state);

	/**
	 * MAC configuration -> MPC8313 User Manual, Chapter 15.6.2.2
	 *
	 */
	// Shut down ethernet controller, in case it is already running
	// Disable WWR, because of the errata eTSEC5
	tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS | DMACTRL_WWR);
	//asm("sync");
	tsec->dmactrl |= (DMACTRL_GRS| DMACTRL_GTS);

	/* FIX??? when should this be enabled?? memory interface snooping */
	tsec->dmactrl |= (DMACTRL_TDSEN | DMACTRL_TBDSEN);
	tsec->attr |= ATTR_RDSEN | ATTR_RBDSEN;

	/* we clear halt upon transmit rather than let the hw poll */
	tsec->dmactrl |= DMACTRL_WOP;

	while ((tsec->ievent & (IEVENT_GRSC| IEVENT_GTSC)) != (IEVENT_GRSC
			| IEVENT_GTSC))
	{
		HAL_DELAY_US(1000);
		if (++i >= 100)
		{
//			os_printf("ETH Controller DMA stop failed, return from tsec_eth_reset\n");
			HAL_RESTORE_INTERRUPTS(int_state);
			return false;
		}
	}

	//clear the graceful stop events
	tsec->ievent = IEVENT_GRSC | IEVENT_GTSC;
	tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS);
	/* Reset MAC for at least 3 TX clock cycle*/
	tsec->maccfg1 = MACCFG1_SOFT_RESET;
	HAL_DELAY_US(1000); /* this should be long enought !! */
	/* Clear soft reset */
	tsec->maccfg1 = (MACCFG1_RXFLOW| MACCFG1_TXFLOW);

	txbd = tsec_eth_txring;
	rxbd = tsec_eth_rxring;

	/* Init Rx / Tx ring base */
	qi->tbase = qi->txbd = txbd;
	qi->rbase = qi->rxbd = qi->rnext = rxbd;

	RxBUF = &tsec_eth_rxbufs[0];
	TxBUF = &tsec_eth_txbufs[0];

	/* Initialize Rx BDs */
	for (i = 0; i < CYGNUM_DEVS_ETH_POWERPC_TSEC_RxNUM; i++)
	{
		rxbd->length = 0;
		rxbd->buffer = RxBUF;
//									interrupts are triggered once every 2 packages
		rxbd->ctrl = FEC_BD_Rx_Empty | ((i % 1 == 0) ? FEC_BD_Rx_Intr : 0);
		RxBUF += CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_RX;
		rxbd++;
	}
	rxbd--;
	rxbd->ctrl |= FEC_BD_Rx_Wrap; // Last buffer
#if CACHE()
	HAL_DCACHE_FLUSH(tsec_eth_rxbufs, sizeof(tsec_eth_rxbufs));
	HAL_DCACHE_FLUSH(tsec_eth_rxring, sizeof(tsec_eth_rxring));
#endif
	/* Initialize Tx BDs */
	for (i = 0; i < CYGNUM_DEVS_ETH_POWERPC_TSEC_TxNUM; i++)
	{
		txbd->length = 0;
		txbd->buffer = TxBUF;
		// let interrupts happen every frame
		txbd->ctrl = FEC_BD_Tx_Intr;
		TxBUF += CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_TX;
		txbd++;
	}
	txbd--;
	txbd->ctrl |= FEC_BD_Tx_Wrap; // Last buffer
#if CACHE()
	HAL_DCACHE_FLUSH(tsec_eth_txbufs, sizeof(tsec_eth_txbufs));
	HAL_DCACHE_FLUSH(tsec_eth_txring, sizeof(tsec_eth_txring));
#endif


	/* Initialize shared PRAM */
	tsec->rbase = tsec->rbptr = (cyg_uint32) (qi->rbase);
	tsec->tbase = tsec->tbptr = (cyg_uint32) (qi->tbase);

	/* Init default value */
	tsec->maccfg2 = 0x00007000 | MACCFG2_IF_MODE_NIBBLE | MACCFG2_PAD_CRC
			| ((flags & RESET_FULL_DUPLEX) ? MACCFG2_FULL_DUPLEX : 0x0);
	tsec->ecntrl = ECNTRL_STEN | ECNTRL_RMM
			| ((flags & RESET_100MB) ? ECNTRL_R100M : 0);

	/* Disables all interrupts */
	tsec->ievent = IEVENT_CLEAR_ALL;
	/* Init mask */
	tsec->imask = IMASK_CLEAR_ALL;

	/* Size of receive buffers */
	tsec->mrblr = CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_RX;
	/* Minimum size, 64 bytes */
	tsec->minflr = 0x00000040;

	/* Rx / Tx control */
	tsec->rctrl = 0x00000000; /* CC: TODO, double check what to do for VLAN tag */
	tsec->tctrl = 0x00000000; //tsec->tctrl = 0x4; //related with DMACTRL_TOD

	/* Largest possible ethernet frame */
	tsec->maxfrm = IEEE_8023_MAX_FRAME;

	//tsec->attr
	//tsec->fifo_tx_thr
	//tsec->fifo_tx_starve
	//tsec->fifo_tx_starve_shutoff

	// Group address hash
	tsec->gaddr[0] = 0;
	tsec->gaddr[1] = 0;
	tsec->gaddr[2] = 0;
	tsec->gaddr[3] = 0;
	tsec->gaddr[4] = 0;
	tsec->gaddr[5] = 0;
	tsec->gaddr[6] = 0;
	tsec->gaddr[7] = 0;

	tsec->igaddr[0] = 0;
	tsec->igaddr[1] = 0;
	tsec->igaddr[2] = 0;
	tsec->igaddr[3] = 0;
	tsec->igaddr[4] = 0;
	tsec->igaddr[5] = 0;
	tsec->igaddr[6] = 0;
	tsec->igaddr[7] = 0;

	// Init MIB
	memset((void *) &(tsec->rmon), 1, sizeof(struct rmon_tsec));
	tsec->rmon.cam1 = 0xFFFFFFFF;
	tsec->rmon.cam2 = 0xFFFFFFFF;

	/* Device physical address */
	tsec->macstnaddr1 = (cyg_uint32) ((cyg_uint32) enaddr[5] << 24
			| (cyg_uint32) enaddr[4] << 16 | (cyg_uint32) enaddr[3] << 8
			| (cyg_uint32) enaddr[2]);
	tsec->macstnaddr2 = (cyg_uint32) ((cyg_uint32) enaddr[1] << 24
			| (cyg_uint32) enaddr[0] << 16);

	tsec->tstat = TSTAT_TXF;
	tsec->rstat = RSTAT_QHLT | RSTAT_RXF;

	tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS);

	/* Enable Tx / Rx */
	tsec->maccfg1 |= (MACCFG1_RXEN| MACCFG1_TXEN);

	/* Umask interrupt */
#ifdef _TSEC_USE_INTS
	tsec->imask = IMASK_DEFAULT;
#endif

//	//debug:
//	index_in_memory_area = 0;
//	stopper = 0;

	HAL_RESTORE_INTERRUPTS(int_state);

	return true;
}

static void tsec_eth_set_rmii_io(void)
{
	volatile IMMR * pIMMR = (IMMR *) (CYGARC_IMM_BASE);
	pIMMR->sicrh &= ~(1 << 5);
	pIMMR->sicrh &= ~(1 << 6);
	pIMMR->sicrh &= ~(1 << 7);
}

void phyAutoNegociate(cyg_uint32 address, int *link, int *speed100,
		int *full_duplex)
{
	cyg_uint32 phy_state;
	int i;

	phy_write_register(address, MII_CTRL_REG, 0xb100);
	do
	{
		phy_state = phy_read_register(address, MII_STAT_REG);
//		os_printf("waiting for reset\n");
	} while (phy_state & 0x8000);

	phy_write_register(address, MII_AUTO_NEG_ADV_REG, 0x01e1);
	phy_write_register(address, MII_PHY_CTRL_REG, 0x0000);
	phy_write_register(address, MII_PHY_IRQ_REG, 0x0000);

#ifdef CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_Auto
//auto negotiate speed and duplex
	phy_write_register(address, MII_CTRL_REG, 0x3300);

	for (i = 0; i < 500; i++)
	{
		phy_state = phy_read_register(address, MII_STAT_REG);
		if (phy_state & 0x20)
			break;
	}
	if (i == 500)
	{
		os_printf("auto negotiation failed\n");
		os_printf("status register reads: 0x%08x\n", phy_state);
	}
#else

//manually set speed and duplex
	phy_state = 0x0000;
#ifdef CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_100Mb
	phy_state |= 0x2000;
#endif //CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_100Mb
#ifdef CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_10Mb
	phy_state |= 0x0000;
#endif //CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_10Mb

#ifdef CYGNUM_DEVS_ETH_POWERPC_TSEC_DUPLEX_MODE_FULL
	phy_state |= 0x0100;
#else
	phy_state |= 0x0000;
#endif //CYGNUM_DEVS_ETH_POWERPC_TSEC_DUPLEX_MODE_FULL

	phy_write_register(address, MII_CTRL_REG, phy_state);
#endif //CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_100Mb


	os_printf("TSEC : ");
	if ((phy_state & 0x0004) != 0)
	{
		*link = 1;
		if ((phy_state & 0x6000) != 0)
		{
			*speed100 = 1;
			// Link can handle 100Mb
			os_printf("100Mb");
			if ((phy_state & 0x4000) != 0)
			{
				os_printf("/Full Duplex\n");
				*full_duplex = 1;
			}
			else
			{
				*full_duplex = 0;
			}
		}
		else
		{
			// Assume 10Mb, half duplex
			os_printf("10Mb\n");
			*speed100 = 0;
			*full_duplex = 0;
		}
	}
	else
	{
		*link = 0;
		os_printf("***NO LINK***\n");
		os_printf("Reg 1: 0x%04x\n", phy_state);

	}
	for (i = 0; i < 32; i++)
	{
		phy_state = phy_read_register(address, i);
//		os_printf("Reg %d: 0x%04x\n", i, phy_state);
	}
	phy_write_register(address, MII_PHY_IRQ_REG, 0x8000);
}

//
// Initialize the interface - performed at system startup
// This function must set up the interface, including arranging to
// handle interrupts, etc, so that it may be "started" cheaply later.
//
static bool tsec_eth_init(struct cyg_netdevtab_entry *tab)
{
	struct eth_drv_sc *sc = (struct eth_drv_sc *) tab->device_instance;
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct mpq_tsec *tsec =
			(volatile struct mpq_tsec *) ((unsigned char *) CYGARC_IMM_BASE
					+ CYGARC_REG_IMM_TSEC1);
	int speed100 = 0;
	int full_duplex = 0;
	int link = 0;
	bool esa_ok;
	int i;
	cyg_uint32 phy_state = 0;
	cyg_uint32 int_state;

	HAL_DISABLE_INTERRUPTS(int_state);


	os_printf("ETH Init\n");

	tsec_eth_set_rmii_io();
	qi->tsec = tsec;
	tsec_eth_stop(sc);
#ifdef _TSEC_USE_INTS

	cyg_drv_interrupt_create(TSEC_ETH_INT,
							0,
							(cyg_addrword_t) sc, //  Data item passed to interrupt handler
							(cyg_ISR_t *) tsec_eth_isr,
							(cyg_DSR_t *) eth_drv_dsr,
							&tsec_eth_interrupt_handle_err,
							&tsec_eth_interrupt_err);
	cyg_drv_interrupt_attach(tsec_eth_interrupt_handle_err);
	cyg_drv_interrupt_acknowledge(TSEC_ETH_INT);
	cyg_drv_interrupt_unmask(TSEC_ETH_INT);
	cyg_drv_interrupt_create(TSEC_ETH_INT + 1,
							0,
							(cyg_addrword_t) sc, //  Data item passed to interrupt handler
							(cyg_ISR_t *) tsec_eth_isr,
							(cyg_DSR_t *) eth_drv_dsr,
							&tsec_eth_interrupt_handle_rx,
							&tsec_eth_interrupt_rx);
	cyg_drv_interrupt_attach(tsec_eth_interrupt_handle_rx);
	cyg_drv_interrupt_acknowledge(TSEC_ETH_INT + 1);
	cyg_drv_interrupt_unmask(TSEC_ETH_INT + 1);
	cyg_drv_interrupt_create(TSEC_ETH_INT + 2,
							0,
							(cyg_addrword_t) sc, //  Data item passed to interrupt handler
							(cyg_ISR_t *) tsec_eth_isr,
							(cyg_DSR_t *) eth_drv_dsr,
							&tsec_eth_interrupt_handle_tx,
							&tsec_eth_interrupt_tx);
	cyg_drv_interrupt_attach(tsec_eth_interrupt_handle_tx);
	cyg_drv_interrupt_acknowledge(TSEC_ETH_INT + 2);
	cyg_drv_interrupt_unmask(TSEC_ETH_INT + 2);

#endif

	esa_ok = CYGACC_CALL_IF_FLASH_CFG_OP(CYGNUM_CALL_IF_FLASH_CFG_GET,
			"tsec_esa", enaddr, CONFIG_ESA);
	if (!esa_ok)
	{
		// Can't figure out ESA
		os_printf("TSEC_ETH - Warning! ESA unknown\n");
		memcpy(&enaddr, &_default_enaddr, sizeof(enaddr));
	}
	resetPHY();
	i = 0;
	while ((tsec->miimind & MIIMIND_BUSY) != 0 && i++ < 500)
	{
//		os_printf(".");
		HAL_DELAY_US(10000);
	}
	for (qi->phyAddress = 1; qi->phyAddress < 0x1f; qi->phyAddress++)
	{
		phy_state = phy_read_register(qi->phyAddress, MII_PHY_ID1_REG);
		phy_state = (phy_state & 0xffff) << 16;
		phy_state |= phy_read_register(qi->phyAddress, MII_PHY_ID2_REG)
				& 0xffff;
		if (phy_state == 0xffffffff)
		{
			os_printf("The PHY management interface is not available! Hardware problem!\n");
		}
		else
		{
//			os_printf("life on 0x%02x, state = 0x%08x\n", qi->phyAddress, phy_state);
			break;
		}
	}
	createPHYInterrupt(sc);
	phyAutoNegociate(qi->phyAddress, &link, &speed100, &full_duplex);
	if (!tsec_eth_reset(sc, enaddr, (full_duplex ? RESET_FULL_DUPLEX
			: 0x00000000) | (speed100 ? RESET_100MB : 0x00000000)))
	{
		return false;
	}
	(sc->funs->eth_drv->init)(sc, (unsigned char *) &enaddr);

	HAL_RESTORE_INTERRUPTS(int_state);
#ifdef CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_Auto
	if (!link)
	{
		return false;
	}
#endif
	return true;
}

//
// This function is called to shut down the interface.
//
static void tsec_eth_stop(struct eth_drv_sc *sc)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	int i = 0;

	os_printf("ETH stop\n");

	// Mask and clear all interrupt
	qi->tsec->imask = IMASK_CLEAR_ALL; // No interrupts enabled
	qi->tsec->ievent = IEVENT_CLEAR_ALL; // Clear all interrupts

	//Stop DMA
	qi->tsec->dmactrl |= (DMACTRL_GRS| DMACTRL_GTS);
	while (i++ < 500 && ((qi->tsec->ievent & (IEVENT_GRSC| IEVENT_GTSC))
			!= (IEVENT_GRSC| IEVENT_GTSC)))
	{
		HAL_DELAY_US(1000);
	}

	// Disable TX and RX
	qi->tsec->maccfg1 &= ~(MACCFG1_RXEN| MACCFG1_TXEN);
}

//
// This function is called to "start up" the interface.  It may be called
// multiple times, even when the hardware is already running.  It will be
// called whenever something "hardware oriented" changes and should leave
// the hardware ready to send/receive packets.
//
static void tsec_eth_start(struct eth_drv_sc *sc, unsigned char *enaddr,
		int flags)
{
	int link = 0, speed100 = 0, full_duplex = 0;
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;

	os_printf("ETH start\n");

	//if stopped
	if ((qi->tsec->maccfg1 & (MACCFG1_RXEN| MACCFG1_TXEN)) != (MACCFG1_RXEN
			| MACCFG1_TXEN))
	{
		cyg_uint32 int_state;
		HAL_DISABLE_INTERRUPTS(int_state);
		// Initialize DMACTRL
		qi->tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS);

		qi->tsec->tstat = TSTAT_TXF;
		qi->tsec->rstat = RSTAT_QHLT | RSTAT_RXF;

		// Unmask interrupt
		qi->tsec->imask = IMASK_DEFAULT;


		if(flags)
		{
			//redo autonegociation
			phyAutoNegociate(qi->phyAddress, &link, &speed100, &full_duplex);
#ifdef CYGNUM_DEVS_ETH_POWERPC_TSEC_LINK_MODE_Auto
			if (!link)
			{
				//the generic driver will set some flags
				(sc->funs->stop)(sc);
			}
			else
#endif
			{
				//set MAC connection parameters
				qi->tsec->maccfg2 = 0x00007000 | MACCFG2_IF_MODE_NIBBLE
						| MACCFG2_PAD_CRC | ((full_duplex) ? MACCFG2_FULL_DUPLEX
						: 0x0);
				qi->tsec->ecntrl = ECNTRL_STEN | ECNTRL_RMM
						| ((speed100) ? ECNTRL_R100M : 0);

				// Enable Rx and Tx
				qi->tsec->maccfg1 |= (MACCFG1_RXEN| MACCFG1_TXEN);
			}
		}
		else
		{
			qi->tsec->maccfg1 |= (MACCFG1_RXEN| MACCFG1_TXEN);
		}
		HAL_RESTORE_INTERRUPTS(int_state);
	}
}

//
// This function is called for low level "control" operations
//
static int tsec_eth_control(struct eth_drv_sc *sc, unsigned long key,
		void *data, int length)
{
#ifdef ETH_DRV_SET_MC_ALL
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct mpq_tsec *tsec = qi->tsec;
#endif

	switch (key)
	{
	case ETH_DRV_SET_MAC_ADDRESS:
	{
		cyg_uint8 *esa = (cyg_uint8 *)data;
		qi->tsec->macstnaddr1 = (cyg_uint32) ((cyg_uint32) esa[5] << 24
				| (cyg_uint32) esa[4] << 16 | (cyg_uint32) esa[3] << 8
				| (cyg_uint32) esa[2]);
		qi->tsec->macstnaddr2 = (cyg_uint32) ((cyg_uint32) esa[1] << 24
				| (cyg_uint32) esa[0] << 16);
#ifdef CYGPKG_REDBOOT
		memcpy(&(sc->sc_arpcom.esa), esa, length);
#else
#ifdef CYGPKG_POLL_NETWORKING
		memcpy(&(sc->sc_arpcom.esa), esa, length);
#else
		memcpy(&(sc->sc_arpcom.ac_enaddr), esa, length);
#endif
#endif
		return 0;
		break;
	}
#ifdef ETH_DRV_SET_MC_ALL
	case ETH_DRV_SET_MC_ALL:
	case ETH_DRV_SET_MC_LIST:
		tsec->gaddr[0] = 0xFFFFFFFF;
		tsec->gaddr[1] = 0xFFFFFFFF;
		return 0;
		break;
#endif
	default:
		return 1;
		break;
	}
}

//
// This function is called to see if another packet can be sent.
// It should return the number of packets which can be handled.
// Zero should be returned if the interface is busy and can not send any more.
//
static int tsec_eth_can_send(struct eth_drv_sc *sc)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct tsec_bd *txbd;
	txbd = qi->txbd;
#if CACHE()
	int cache_state;
	HAL_DCACHE_IS_ENABLED(cache_state);
	if (cache_state)
	{
		/* avoid any naggig doubt that a txbd could cross a cache line and flush each bd */
		HAL_DCACHE_INVALIDATE(txbd, sizeof(*txbd));
	}
#endif
    /* FIX!!!! how to do this robustly??? */
	return (txbd->ctrl & FEC_BD_Tx_Ready)==0;
}

//
// This routine is called to send data to the hardware.
static void tsec_eth_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
		int sg_len, int total_len, unsigned long key)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct tsec_bd *txbd;
	volatile unsigned char *bp;
	int i, txindex;

	volatile struct mpq_tsec *tsec =
			(volatile struct mpq_tsec *) ((unsigned char *) CYGARC_IMM_BASE
					+ CYGARC_REG_IMM_TSEC1);

#if CACHE()
	int cache_state;
	HAL_DCACHE_IS_ENABLED(cache_state);
#endif


    /* if the next buffer isn't free, we're broken */
	txbd = qi->txbd;

//	while ((tsec->tstat & TSTAT_THLT)==0)
//	{
//		/* wait for ring to halt to be able to robustly read the tbptr */
//	}


//	if(qi->txbd != tsec->tbptr)
//	{
//		printf("wtf\n");
//	}

//trust the software, not set it to the value pointed by the HW
//	txbd = tsec->tbptr; // why the "#&¤"#¤&"#&¤" do we fail to keep track of txbd in software???  :-)

#if CACHE()
	if (cache_state)
	{
		/* avoid any naggig doubt that a txbd could cross a cache line and flush each bd */
		HAL_DCACHE_INVALIDATE(txbd, sizeof(*txbd));
	}
#endif

	CYG_ASSERT((txbd->ctrl & FEC_BD_Tx_Ready)==0, "TX buffer not ready when it was expected to be");

	// Set up buffer
	bp = txbd->buffer;
	for (i = 0; i < sg_len; i++)
	{
		memcpy((void *) bp, (void *) sg_list[i].buf, sg_list[i].len);
		bp += sg_list[i].len;
	}
	txbd->length = total_len;
	txindex = ((unsigned long) txbd - (unsigned long) qi->tbase)
			/ sizeof(*txbd);
	qi->txkey[txindex] = key;
#if CACHE()
	// Note: the MPC8xx does not seem to snoop/invalidate the data cache properly!
	HAL_DCACHE_IS_ENABLED(cache_state);
	if (cache_state)
	{
		HAL_DCACHE_FLUSH(txbd->buffer, txbd->length); // Make sure no stale data
	}
#endif
	txbd->ctrl |= FEC_BD_Tx_Ready | FEC_BD_Tx_Last | FEC_BD_Tx_TC;

#if CACHE()
	if (cache_state)
	{
		/* and off it goes to the hardware! */
		HAL_DCACHE_FLUSH(txbd, sizeof(*txbd));;
	}
#endif


	/* clear halt condition, send it on it's way */
	tsec->tstat = TSTAT_THLT;

	/* for debug purposes we wait for the frame to be on it's way */
//	for (;;)
//	{
//#if CACHE()
//		if (cache_state)
//		{
//			/* and off it goes to the hardware! */
//			HAL_DCACHE_FLUSH(txbd, sizeof(*txbd));;
//		}
//#endif
//		if ((txbd->ctrl&FEC_BD_Tx_Ready)==0)
//		{
//			/* it's been sent */
//			break;
//		}
//	}


	// Remember the next buffer to try
	if (txbd->ctrl & FEC_BD_Tx_Wrap)
	{
		qi->txbd = qi->tbase;
	}
	else
	{
		qi->txbd = txbd + 1;
	}
#if CACHE()
	HAL_DCACHE_FLUSH(qi, sizeof(*qi));
#endif
}

//
// This function is called when a packet has been received.  It's job is
// to prepare to unload the packet from the hardware.  Once the length of
// the packet is known, the upper layer of the driver can be told.  When
// the upper layer is ready to unload the packet, the internal function
// 'fec_eth_recv' will be called to actually fetch it from the hardware.
//
static void tsec_eth_RxEvent(struct eth_drv_sc *sc)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct tsec_bd *rxbd, *rxfirst;

#if CACHE()
	int cache_state;
	HAL_DCACHE_IS_ENABLED(cache_state);
#endif

	rxbd = rxfirst = qi->rnext;
	while (true)
	{
#if CACHE()
		if (cache_state)
		{
			HAL_DCACHE_INVALIDATE(rxbd, sizeof(*rxbd));
		}
#endif
		if ((rxbd->ctrl & FEC_BD_Rx_Empty) == 0)
		{
			qi->rxbd = rxbd; // Save for callback
			//            set_led(LED_RxACTIVE);     // Remove the CRC

			/* cache invalidation happens closer to actual use */
			(sc->funs->eth_drv->recv)(sc, rxbd->length - 4);
		}
		if (rxbd->ctrl & FEC_BD_Rx_Wrap)
		{
			rxbd = qi->rbase;
		}
		else
		{
			rxbd++;
		}
		if (rxbd == rxfirst)
		{
			break;
		}
	}
	// Remember where we left off
	qi->rnext = (struct tsec_bd *) rxbd;
}

//
// This function is called as a result of the "eth_drv_recv()" call above.
// It's job is to actually fetch data for a packet from the hardware once
// memory buffers have been allocated for the packet.  Note that the buffers
// may come in pieces, using a scatter-gather list.  This allows for more
// efficient processing in the upper layers of the stack.
//
static void tsec_eth_recv(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
		int sg_len)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	unsigned char *bp;
	int i;

	bp = (unsigned char *) qi->rxbd->buffer;
#if CACHE()
	int cache_state;
	// Note: the MPC8xx does not seem to snoop/invalidate the data cache properly!
	HAL_DCACHE_IS_ENABLED(cache_state);
	if (cache_state)
	{
		HAL_DCACHE_INVALIDATE(qi->rxbd->buffer, qi->rxbd->length); // Make sure no stale data
	}
#endif
	for (i = 0; i < sg_len; i++)
	{
		if (sg_list[i].buf != 0)
		{
			memcpy((void *) sg_list[i].buf, bp, sg_list[i].len);
			bp += sg_list[i].len;
//			//debug:
//			if(index_in_memory_area + sg_list[i].len < 1024 * 1024 )
//			{
//				stopper ++;
//				memcpy(memory_area + index_in_memory_area, bp, sg_list[i].len);
//				index_in_memory_area += sg_list[i].len;
//				if(stopper == 20)
//					os_printf("break!\n");
//			}
//			else
//				os_printf("break!\n");
		}
	}
	qi->rxbd->ctrl |= FEC_BD_Rx_Empty;
	qi->rxbd->length = 0;
#if CACHE()
	if (cache_state)
	{
		HAL_DCACHE_FLUSH(qi->rxbd, sizeof(*qi->rxbd));
	}
#endif

	//    clear_led(LED_RxACTIVE);
}

/* check *all* buffer descriptors */
static void tsec_eth_TxEvent(struct eth_drv_sc *sc)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	volatile struct tsec_bd *txbd;
	int key, txindex;

	// Make sure no stale data
#if CACHE()
	int cache_state;
	HAL_DCACHE_IS_ENABLED(cache_state);
#endif

	txbd = qi->tbase;
	for (;;)
	{
#if CACHE()
		if (cache_state)
		{
			HAL_DCACHE_INVALIDATE(txbd, sizeof(*txbd));
		}
#endif
		// Note: TC field is used to indicate the buffer has/had data in it
		int wrap=(txbd->ctrl & FEC_BD_Tx_Wrap);
		if ((txbd->ctrl & (FEC_BD_Tx_Ready)) == 0)
		{
			txindex = ((unsigned long) txbd - (unsigned long) qi->tbase)
					/ sizeof(*txbd);

			if ((key = qi->txkey[txindex]) != 0)
			{
				qi->txkey[txindex] = 0;
				(sc->funs->eth_drv->tx_done)(sc, key, 0);
			}
		}
		if (wrap)
		{
			break;
		}
		txbd++;
	}
}

//
// Interrupt processing
//
static void tsec_eth_int(struct eth_drv_sc *sc)
{
	struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
	cyg_uint32 event;
	int error = 0;

	while ((event = qi->tsec->ievent) != 0)
	{
		qi->tsec->ievent = event; // Reset the bits we handled

		if ((event & IEVENT_BABR) != 0)
		{
			os_printf("Babbling receive error.\n");
			error = 1;
		}
		if ((event & IEVENT_RXC) != 0)
		{
						os_printf("Receive control interrupt.\n");
						error =1 ;
		}
		if ((event & IEVENT_BSY) != 0)
		{
			os_printf(
					"Busy condition interrupt. Indicates that a frame was received and discarded due to a lack of buffers.\n");
			error = 1;
		}
		if ((event & IEVENT_EBERR) != 0)
		{
			os_printf("Internal bus error.\n");
			error = 1;
		}
		if ((event & IEVENT_MSRO) != 0)
		{
			os_printf("MIB counter overflow.\n");
			error = 1;
		}
		if ((event & IEVENT_BABT) != 0)
		{
			os_printf("Babbling transmit error.\n");
			error = 1;
		}
		if ((event & IEVENT_TXC) != 0)
		{
						os_printf("Transmit control interrupt.\n");
						error = 1;
		}
		if ((event & IEVENT_TXE) != 0)
		{
			os_printf("Transmit error.\n");
			error = 1;
		}
		if ((event & IEVENT_LC) != 0)
		{
			os_printf("Late collision.\n");
			error = 1;
		}
		if ((event & IEVENT_CRL) != 0)
		{
			os_printf("Collision retry limit.\n");
			error = 1;
		}
		if ((event & IEVENT_XFUN) != 0)
		{
			os_printf("Transmit FIFO underrun.\n");
			error = 1;
		}
		if ((event & IEVENT_DPE) != 0)
		{
			os_printf("Internal data parity error.\n");
			error = 1;
		}
		if ((event & IEVENT_MSRO) != 0)
		{
			os_printf("MIB counter overflow.\n");
			error = 1;
		}
		if ((event & IEVENT_PERR) != 0)
		{
			os_printf("Receive frame parse error for TCP/IP off-load.\n");
			error = 1;
		}
		if (error)
		{
			//unhalt transmition in case of an error
			qi->tsec->rstat |= qi->tsec->rstat;
#ifdef _TSEC_USE_INTS
			// Allow interrupts to happen again
			cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_TSEC1_Err);
#endif
		}
		if ((event & IEVENT_RXF) != 0 || (event & IEVENT_RXBO) != 0)
		{

			if((event & IEVENT_RXF) != 0)
			{
				//clear the frame receive flag
				qi->tsec->rstat |= qi->tsec->rstat;
//				os_printf("IEVENT_RXF ");
			}
			if((event & IEVENT_RXBO) != 0)
//				os_printf("IEVENT_RXBO ");
			os_printf("\n");

			tsec_eth_RxEvent(sc);
#ifdef _TSEC_USE_INTS
			// Allow interrupts to happen again
			cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_TSEC1_Rx);
#endif
		}
		if ((event & IEVENT_TXF) != 0 || (event & IEVENT_TXB) != 0)
		{
			tsec_eth_TxEvent(sc);
#ifdef _TSEC_USE_INTS
			// Allow interrupts to happen again
			cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_TSEC1_Tx);
#endif

		}
	}
}

//
// Interrupt vector
//
static int tsec_eth_int_vector(struct eth_drv_sc *sc)
{
	return (TSEC_ETH_INT);
}

#ifndef _TSEC_USE_INTS
static int tsec_fake_int(cyg_addrword_t param)
{
	struct eth_drv_sc *sc = (struct eth_drv_sc *) param;
	cyg_uint32 int_state;

	while (true)
	{
		cyg_thread_delay(1); // 10ms
		HAL_DISABLE_INTERRUPTS(int_state);
		tsec_eth_int(sc);
		HAL_RESTORE_INTERRUPTS(int_state);
	}
}
#endif

/**
 * Display eTSEC1 RMON statistics
 *
 */
void tsec_disp_stats(void)
{
	volatile struct mpq_tsec *tsec =
			(volatile struct mpq_tsec *) ((unsigned char *) CYGARC_IMM_BASE
					+ CYGARC_REG_IMM_TSEC1);
	volatile IMMR * pppIMMR = (IMMR *) (CYGARC_IMM_BASE);

	os_printf("Ethernet Stats\n");

	os_printf("%6s %10d\n", "tr64", tsec->rmon.tr64);
	os_printf("%6s %10d\n", "tr127", tsec->rmon.tr127);
	os_printf("%6s %10d\n", "tr255", tsec->rmon.tr255);
	os_printf("%6s %10d\n", "tr511", tsec->rmon.tr511);
	os_printf("%6s %10d\n", "tr1k", tsec->rmon.tr1k);
	os_printf("%6s %10d\n", "trmax", tsec->rmon.trmax);
	os_printf("%6s %10d\n", "trmgv", tsec->rmon.trmgv);
	os_printf("%6s %10d\n", "rbyt", tsec->rmon.rbyt);
	os_printf("%6s %10d\n", "rpkt", tsec->rmon.rpkt);
	os_printf("%6s %10d\n", "rfcs", tsec->rmon.rfcs);
	os_printf("%6s %10d\n", "rmca", tsec->rmon.rmca);
	os_printf("%6s %10d\n", "rbca", tsec->rmon.rbca);
	os_printf("%6s %10d\n", "rxcf", tsec->rmon.rxcf);
	os_printf("%6s %10d\n", "rxpf", tsec->rmon.rxpf);
	os_printf("%6s %10d\n", "rxuo", tsec->rmon.rxuo);
	os_printf("%6s %10d\n", "raln", tsec->rmon.raln);
	os_printf("%6s %10d\n", "rflr", tsec->rmon.rflr);
	os_printf("%6s %10d\n", "rcde", tsec->rmon.rcde);
	os_printf("%6s %10d\n", "rcse", tsec->rmon.rcse);
	os_printf("%6s %10d\n", "rund", tsec->rmon.rund);
	os_printf("%6s %10d\n", "rovr", tsec->rmon.rovr);
	os_printf("%6s %10d\n", "rfrg", tsec->rmon.rfrg);
	os_printf("%6s %10d\n", "rjbr", tsec->rmon.rjbr);
	os_printf("%6s %10d\n", "rdrp", tsec->rmon.rdrp);
	os_printf("%6s %10d\n", "tbyt", tsec->rmon.tbyt);
	os_printf("%6s %10d\n", "tpkt", tsec->rmon.tpkt);
	os_printf("%6s %10d\n", "tmca", tsec->rmon.tmca);
	os_printf("%6s %10d\n", "tbca", tsec->rmon.tbca);
	os_printf("%6s %10d\n", "txpf", tsec->rmon.txpf);
	os_printf("%6s %10d\n", "tdfr", tsec->rmon.tdfr);
	os_printf("%6s %10d\n", "tedf", tsec->rmon.tedf);
	os_printf("%6s %10d\n", "tscl", tsec->rmon.tscl);
	os_printf("%6s %10d\n", "tmcl", tsec->rmon.tmcl);
	os_printf("%6s %10d\n", "tlcl", tsec->rmon.tlcl);
	os_printf("%6s %10d\n", "txcl", tsec->rmon.txcl);
	os_printf("%6s %10d\n", "tncl", tsec->rmon.tncl);
	os_printf("%6s %10d\n", "tdrp", tsec->rmon.tdrp);
	os_printf("%6s %10d\n", "tjbr", tsec->rmon.tjbr);
	os_printf("%6s %10d\n", "tfcs", tsec->rmon.tfcs);
	os_printf("%6s %10d\n", "txcf", tsec->rmon.txcf);
	os_printf("%6s %10d\n", "tovr", tsec->rmon.tovr);
	os_printf("%6s %10d\n", "tund", tsec->rmon.tund);

	os_printf("%6s %08x\n", "sipnr_h", pppIMMR->sipnr_h);
	os_printf("%6s %08x\n", "sipnr_l", pppIMMR->sipnr_l);
	os_printf("%6s %08x\n", "sivcr", pppIMMR->sivcr);
	os_printf("%6s %08x\n", "sicfr", pppIMMR->sicfr);
	os_printf("%6s %08x\n", "ievent", tsec->ievent);
	os_printf("%6s %08x\n", "imask", tsec->imask);

}

