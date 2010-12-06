//==========================================================================
//
//      dev/tsec.c
//
//      Fast ethernet device driver for PowerPC MPC83xx, MPC85xx boards
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System. 
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 
// 2008, 2009 Free Software Foundation, Inc. 
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
// Author(s):    Edgar Grimberg
// Contributors: Sachin Sushil Chaddha, Christophe Coutand
// Date:         2009-11-01
// Purpose:      
// Description:  hardware driver for Freescale eTSEC
//              
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/system.h>
#include <pkgconf/devs_eth_powerpc_tsec.h>
#include <pkgconf/io_eth_drivers.h>

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

#ifdef CYGPKG_DEVS_ETH_PHY
#include <cyg/io/eth_phy.h>
#endif

#include "cyg/hal/var_regs.h"
#include "cyg/io/tsec.h"

static unsigned char enaddr[6];

#ifdef CYGARC_REG_IMM_TSEC1
// Configure PHY using MII interface of TSEC1
static tsec_mii_info_t tsec1_mii =
{
  .miimadd  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMADD),
  .miimcom  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCOM),
  .miimstat = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMSTAT),
  .miimind  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMIND),
  .miimcfg  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCFG),
  .miimcon  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1_MIIMCON)
};
#endif

#ifdef CYGARC_REG_IMM_TSEC3
// Configure PHY using MII interface of TSEC3
static tsec_mii_info_t tsec3_mii =
{
  .miimadd  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3_MIIMADD),
  .miimcom  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3_MIIMCOM),
  .miimstat = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3_MIIMSTAT),
  .miimind  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3_MIIMIND),
  .miimcfg  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3_MIIMCFG),
  .miimcon  = (CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3_MIIMCON)
};
#endif

#include CYGDAT_DEVS_TSEC_ETH_CDL
#include CYGDAT_DEVS_TSEC_ETH_INL

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

#ifdef CYGSEM_DEVS_ETH_POWERPC_TSEC_CHATTER
#include <cyg/infra/diag.h>
#define tsec_printf(args...)   diag_printf(args)
#else
#define tsec_printf(args...)   /* NOOP */
#endif

// For fetching the ESA from RedBoot
#include <cyg/hal/hal_if.h>
#ifndef CONFIG_ESA
#define CONFIG_ESA 6
#endif

#ifndef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
#ifdef CYGPKG_KERNEL
#define STACK_SIZE CYGNUM_HAL_STACK_SIZE_MINIMUM
static char tsec_fake_int_stack[TSEC_MAX_INTERFACE][STACK_SIZE];
static cyg_thread tsec_fake_int_thread_data[TSEC_MAX_INTERFACE];
static cyg_handle_t tsec_fake_int_thread_handle[TSEC_MAX_INTERFACE];
static int tsec_fake_int_count = 0;
static void tsec_fake_int(cyg_addrword_t);
#endif
#endif // CYGINT_IO_ETH_INT_SUPPORT_REQUIRED

static void tsec_eth_int(struct eth_drv_sc *data);


// Deliver function (ex-DSR) handles the ethernet [logical] processing
static void tsec_eth_deliver(struct eth_drv_sc * sc)
{
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
  tsec_eth_int(sc);
#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED  
  // Allow interrupts to happen again
  cyg_drv_interrupt_unmask(qi->rx_irq_vector);
#endif
}

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
// This ISR is called when the ethernet interrupt occurs
static int tsec_eth_isr(cyg_vector_t vector, cyg_addrword_t data,
		HAL_SavedRegisters *regs)
{
  cyg_drv_interrupt_mask(vector);
  cyg_drv_interrupt_acknowledge(vector);
  return (CYG_ISR_HANDLED | CYG_ISR_CALL_DSR); // Run the DSR
}

void tsec_eth_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
  tsec_eth_deliver((struct eth_drv_sc *) data);
  // Allow interrupts to happen again
  cyg_drv_interrupt_unmask(vector);
}

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
   cyg_spinlock_t                 dsr_spin_lock;	
   int dsr_spin_lock_flag = 0;
#endif

void tsec_eth_smp_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
  struct eth_drv_sc * sc = (struct eth_drv_sc *) data;
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
  CYG_INTERRUPT_STATE istate;
	
#ifdef CYGPKG_KERNEL_SMP_SUPPORT  
  cyg_drv_spinlock_spin_intsave( &dsr_spin_lock , &istate); 
#endif

  eth_drv_dsr(vector, count, data);

#ifdef CYGPKG_KERNEL_SMP_SUPPORT  
  cyg_drv_spinlock_clear_intsave( &dsr_spin_lock , istate); 
#endif

}
#endif

//
// Reset PHY
//
static void tsec_reset_phy(tsec_mii_info_t *mii)
{
  HAL_WRITE_UINT32(mii->miimcfg, MIIMCFG_RESET);
  HAL_WRITE_UINT32(mii->miimcfg, MIIMCFG_CLOCK_DIV_14);
  HAL_DELAY_US(12500);
}

static void tsec_phy_init(void)
{
  HAL_DELAY_US(12500);
}

//
// Read PHY register
//
static bool 
tsec_phy_get_reg(tsec_mii_info_t *mii, int reg, int phy, unsigned short *val)
{
  cyg_uint32 address = phy;
  cyg_uint32 ret_val;
  cyg_uint32 miimcom, miimind;
  int i;

  HAL_WRITE_UINT32(mii->miimadd, ((((cyg_uint32) address) << 8) | reg));
  HAL_READ_UINT32(mii->miimcom, miimcom);
  miimcom &= ~MIIMCOM_READ;
  HAL_WRITE_UINT32(mii->miimcom, miimcom);
  miimcom |= MIIMCOM_READ;
  HAL_WRITE_UINT32(mii->miimcom, miimcom);

  HAL_READ_UINT32(mii->miimcom, miimind);
  i = 0;
  while ((miimind & MIIMIND_BUSY) == MIIMIND_BUSY && i++ < 500)
  {
    HAL_DELAY_US(10000);
    HAL_READ_UINT32(mii->miimind, miimind);
  }
  //status register
  HAL_READ_UINT32(mii->miimstat, ret_val);
  HAL_READ_UINT32(mii->miimcom, miimcom);
  miimcom &= ~MIIMCOM_READ;
  HAL_WRITE_UINT32(mii->miimcom, miimcom);

  *val = (unsigned short ) ret_val;
  
  return true;
}

#ifdef CYGARC_REG_IMM_TSEC1
static bool 
tsec1_phy_get_reg(int reg, int phy, unsigned short *val)
{
  return (tsec_phy_get_reg(&tsec1_mii, reg, phy, val));
}
#endif

#ifdef CYGARC_REG_IMM_TSEC3
static bool 
tsec3_phy_get_reg(int reg, int phy, unsigned short *val)
{
  return (tsec_phy_get_reg(&tsec3_mii, reg, phy, val));
}
#endif


//
// Write PHY register
//
static void 
tsec_phy_put_reg(tsec_mii_info_t *mii, int reg, int addr, unsigned short data)
{
  cyg_uint32 address = addr;
  cyg_uint32 miimind;
  cyg_uint32 value = (cyg_uint32) data;
  int i = 0;
  
  HAL_WRITE_UINT32(mii->miimadd, ((cyg_uint32) address << 8) | reg);
  //AN advertisement register
  HAL_WRITE_UINT32(mii->miimcon, value);
  HAL_READ_UINT32(mii->miimind, miimind);
  while (miimind != 0 && i++ < 500)
  {
    HAL_DELAY_US(10000);
    HAL_READ_UINT32(mii->miimind, miimind);
  }
}

#ifdef CYGARC_REG_IMM_TSEC1
static void 
tsec1_phy_put_reg(int reg, int addr, unsigned short data)
{
  tsec_phy_put_reg(&tsec1_mii, reg, addr, data);
}
#endif

#ifdef CYGARC_REG_IMM_TSEC3
static void 
tsec3_phy_put_reg(int reg, int addr, unsigned short data)
{
  tsec_phy_put_reg(&tsec3_mii, reg, addr, data);
}
#endif

//
// [re]Initialize the ethernet controller
//   Done separately since shutting down the device requires a
//   full reconfiguration when re-enabling.
//   when
static bool tsec_eth_reset(struct eth_drv_sc *sc, unsigned char *enaddr,
		int flags)
{
    struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
    volatile struct mpq_tsec *tsec = (volatile struct mpq_tsec *) qi->tsec;
    volatile struct tsec_bd *rxbd, *txbd;
    unsigned char *RxBUF, *TxBUF;
    cyg_uint32 int_state;
    int i = 0;

    HAL_DISABLE_INTERRUPTS(int_state);

    tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS | DMACTRL_WWR);
    tsec->dmactrl |= (DMACTRL_GRS| DMACTRL_GTS);
    /* Enable memory interface snooping */
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

    txbd = qi->init_rxring;
    rxbd = qi->init_txring;

    /* Init Rx / Tx ring base */
    qi->tbase = qi->txbd = qi->tnext = txbd;
    qi->rbase = qi->rxbd = qi->rnext = rxbd;
    qi->txactive = 0;

    RxBUF = qi->init_rxbufs;
    TxBUF = qi->init_txbufs;

    /* Initialize Rx BDs */
    for (i = 0; i < CYGNUM_DEVS_ETH_POWERPC_TSEC_RxNUM; i++)
    {
      rxbd->length = 0;
      rxbd->buffer = RxBUF;  
      rxbd->ctrl = FEC_BD_Rx_Empty | FEC_BD_Rx_Intr;
      RxBUF += CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_RX;
      rxbd++;
    }
    rxbd--;
    rxbd->ctrl |= FEC_BD_Rx_Wrap; // Last buffer

    /* Initialize Tx BDs */
    for (i = 0; i < CYGNUM_DEVS_ETH_POWERPC_TSEC_TxNUM; i++)
    {
      txbd->length = 0;
      txbd->buffer = TxBUF;	  
      txbd->ctrl = FEC_BD_Tx_Intr;
      TxBUF += CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_TX;
      txbd++;
    }
    txbd--;
    txbd->ctrl |= FEC_BD_Tx_Wrap; // Last buffer

    /* Initialize shared PRAM */
    tsec->rbase = tsec->rbptr = (cyg_uint32) (qi->rbase);
    tsec->tbase = tsec->tbptr = (cyg_uint32) (qi->tbase);

    /* Init default value */
    tsec->maccfg2 = 0x00007000 
                     | ((flags & RESET_1000MB) ? MACCFG2_IF_MODE_BYTE : MACCFG2_IF_MODE_NIBBLE) 
                     | MACCFG2_PAD_CRC
			| ((flags & RESET_FULL_DUPLEX) ? MACCFG2_FULL_DUPLEX : 0x0);
	
    tsec->ecntrl = ECNTRL_STEN
			| ((flags & RESET_100MB) ? ECNTRL_R100M : 0);

#ifdef CYGPKG_DEVS_ETH_POWERPC_TSEC_RMII
     tsec->ecntrl |= ECNTRL_RMM;
#endif	

    /* Disables all interrupts */
    tsec->ievent = IEVENT_CLEAR_ALL;
    /* Init mask */
    tsec->imask = IMASK_CLEAR_ALL;

    /* Size of receive buffers */
    tsec->mrblr = CYGNUM_DEVS_ETH_POWERPC_TSEC_BUFSIZE_RX;
    /* Minimum size, 64 bytes */
    tsec->minflr = 0x00000040;

    /* Rx / Tx control */
    tsec->rctrl = 0x00000000; 
    tsec->tctrl = 0x00000000; 

    /* Largest possible ethernet frame */
    tsec->maxfrm = IEEE_8023_MAX_FRAME;

    // Group address hash
    tsec->gaddr[0] = 0;
    tsec->gaddr[1] = 0;
    tsec->gaddr[2] = 0;
    tsec->gaddr[3] = 0;
    tsec->gaddr[4] = 0;
    tsec->gaddr[5] = 0;
    tsec->gaddr[6] = 0;
    tsec->gaddr[7] = 0;
    tsec->gaddr[8] = 0;

    tsec->igaddr[0] = 0;
    tsec->igaddr[1] = 0;
    tsec->igaddr[2] = 0;
    tsec->igaddr[3] = 0;
    tsec->igaddr[4] = 0;
    tsec->igaddr[5] = 0;
    tsec->igaddr[6] = 0;
    tsec->igaddr[7] = 0;
    tsec->igaddr[8] = 0;

    // Init MIB
    memset((void *) &(tsec->rmon), 0, sizeof(struct rmon_tsec));
    tsec->rmon.cam1 = 0xFFFFFFFF;
    tsec->rmon.cam2 = 0xFFFFFFFF;

    /* Device physical address */
    tsec->macstnaddr1 = (cyg_uint32) ((cyg_uint32) enaddr[5] << 24
			| (cyg_uint32) enaddr[4] << 16 | (cyg_uint32) enaddr[3] << 8
			| (cyg_uint32) enaddr[2]);
    tsec->macstnaddr2 = (cyg_uint32) ((cyg_uint32) enaddr[1] << 24
			| (cyg_uint32) enaddr[0] << 16);

  tsec->tstat = TSTAT_THLT  | TSTAT_TXF;
  tsec->rstat = RSTAT_QHLT | RSTAT_RXF;

    tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS);

    /* Enable Tx / Rx */
    tsec->maccfg1 |= (MACCFG1_RXEN| MACCFG1_TXEN);

    /* Umask interrupt */
#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
    tsec->imask = IMASK_DEFAULT;
#endif

    HAL_RESTORE_INTERRUPTS(int_state);

    return true;
}

//
// This function is called to shut down the interface.
//
static void tsec_eth_stop(struct eth_drv_sc *sc)
{
    struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
    int i = 0;
    CYG_INTERRUPT_STATE istate; 
	
    tsec_printf("TSEC - Stop for %s\n", \
	  	sc->dev_name);

#ifdef CYGPKG_KERNEL_SMP_SUPPORT  
    cyg_drv_spinlock_spin_intsave( &qi->tx_spin_lock, &istate); 
    cyg_drv_spinlock_spin( &qi->rx_spin_lock );
#else
    HAL_DISABLE_INTERRUPTS(istate);
#endif

    // Mask and clear all interrupt
    qi->tsec->imask = IMASK_CLEAR_ALL; // No interrupts enabled
    qi->tsec->ievent = IEVENT_CLEAR_ALL; // Clear all interrupts

#if 0
    //Stop DMA
    qi->tsec->dmactrl |= (DMACTRL_GRS| DMACTRL_GTS);
    while (i++ < 500 && ((qi->tsec->ievent & (IEVENT_GRSC| IEVENT_GTSC))
			!= (IEVENT_GRSC| IEVENT_GTSC)))
    {
      HAL_DELAY_US(1000);
    }
#endif
	
    // Disable TX and RX
    qi->tsec->maccfg1 &= ~(MACCFG1_RXEN| MACCFG1_TXEN);

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
    cyg_drv_spinlock_clear( &qi->rx_spin_lock );
    cyg_drv_spinlock_clear_intsave( &qi->tx_spin_lock, istate); 
#else
    HAL_RESTORE_INTERRUPTS(istate);
#endif

    qi->state = 0;		
}

// 
// Init TSEC interface pointer
//
static void tsec_eth_set_if(volatile struct mpq_tsec **tsec, int tsec_num)
{
  switch (tsec_num)
  {
    case 1:
	*tsec = (volatile struct mpq_tsec *) ((unsigned char *) \
		CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC1);	
	break;
#ifdef CYGARC_REG_IMM_TSEC2	
    case 2:
	*tsec = (volatile struct mpq_tsec *) ((unsigned char *) \
		CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC2);		
	break;
#endif	
#ifdef CYGARC_REG_IMM_TSEC3
    case 3:
	*tsec = (volatile struct mpq_tsec *) ((unsigned char *) \
		CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC3);		
	break;
#endif	
#ifdef CYGARC_REG_IMM_TSEC4	
    case 4:
	*tsec = (volatile struct mpq_tsec *) ((unsigned char *) \
		CYGARC_IMM_BASE + CYGARC_REG_IMM_TSEC4);			
	break;
#endif	
    default:
	*tsec = NULL;		
	break;			
  }
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

    int speed100 = 0;
    int speed1000 = 0;	
    int full_duplex = 0;
    bool esa_ok;
    cyg_uint16 phy_state = 0;
    cyg_uint32 int_state;

    tsec_eth_set_if(&qi->tsec, qi->tsec_if);
		
    if(qi->tsec == NULL)
	return false;

    qi->state = 0;

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
    cyg_drv_spinlock_init( &qi->tx_spin_lock , 0);
    cyg_drv_spinlock_init( &qi->rx_spin_lock , 0);
    if (dsr_spin_lock_flag++ == 0)
       cyg_drv_spinlock_init( &dsr_spin_lock , 0);
#endif
		
    HAL_DISABLE_INTERRUPTS(int_state);

    tsec_printf("TSEC - Initialisation for %s\n", \
	  	sc->dev_name);

    tsec_eth_stop(sc);

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED	
    cyg_drv_interrupt_create(qi->err_irq_vector,
							0,
							(cyg_addrword_t) sc, //  Data item passed to interrupt handler
							(cyg_ISR_t *) tsec_eth_isr,
							(cyg_DSR_t *) tsec_eth_smp_dsr,
							&qi->eth_interrupt_handle_err,
							&qi->eth_interrupt_err);
    cyg_drv_interrupt_attach(qi->eth_interrupt_handle_err);
    cyg_drv_interrupt_unmask(qi->err_irq_vector);
    cyg_drv_interrupt_create(qi->rx_irq_vector,
							0,
							(cyg_addrword_t) sc, //  Data item passed to interrupt handler
							(cyg_ISR_t *) tsec_eth_isr,
							(cyg_DSR_t *) tsec_eth_smp_dsr,
							&qi->eth_interrupt_handle_rx,
							&qi->eth_interrupt_rx);
    cyg_drv_interrupt_attach(qi->eth_interrupt_handle_rx);
    cyg_drv_interrupt_unmask(qi->rx_irq_vector);
    cyg_drv_interrupt_create(qi->tx_irq_vector,
							0,
							(cyg_addrword_t) sc, //  Data item passed to interrupt handler
							(cyg_ISR_t *) tsec_eth_isr,
							(cyg_DSR_t *) tsec_eth_smp_dsr,
							&qi->eth_interrupt_handle_tx,
							&qi->eth_interrupt_tx);
    cyg_drv_interrupt_attach(qi->eth_interrupt_handle_tx);
    cyg_drv_interrupt_unmask(qi->tx_irq_vector);
#else
#if defined(CYGPKG_KERNEL)
    // Hack - use a thread to simulate interrupts
    cyg_thread_create(1,              
                      tsec_fake_int,  
                      (cyg_addrword_t)sc, 
                      "TSEC int",     
                      &tsec_fake_int_stack[tsec_fake_int_count][0],     
                      STACK_SIZE,    
                      &tsec_fake_int_thread_handle[tsec_fake_int_count], 
                      &tsec_fake_int_thread_data[tsec_fake_int_count]  
            );
    cyg_thread_resume(tsec_fake_int_thread_handle[tsec_fake_int_count]);  // Start it
    tsec_fake_int_count++;
#endif
#endif

    esa_ok = CYGACC_CALL_IF_FLASH_CFG_OP(CYGNUM_CALL_IF_FLASH_CFG_GET,
			"tsec_esa", enaddr, CONFIG_ESA);

    if (!esa_ok)
    {
      tsec_printf("TSEC - Warning! ESA unknown for %s\n", \
	  	sc->dev_name);
      memcpy(&enaddr, &qi->mac_address, sizeof(enaddr));
    }
	
    tsec_reset_phy(qi->mii);

#ifdef CYGPKG_DEVS_ETH_PHY
    if (!_eth_phy_init(qi->phy)) {
        tsec_printf("TSEC - Failed to initialise PHY of %s\n", \
	  	sc->dev_name);		
	 HAL_RESTORE_INTERRUPTS(int_state);	
        return false;
    }
    phy_state = _eth_phy_state(qi->phy);

    if(phy_state & ETH_PHY_STAT_1000MB)
      speed1000 = 1;
    if(phy_state & ETH_PHY_STAT_100MB)
      speed100 = 1;
    if(phy_state & ETH_PHY_STAT_FDX)
       full_duplex  = 1;
    if(phy_state & ETH_PHY_STAT_LINK){
        tsec_printf("Ethernet Mode (%s): %sMbps/%s\n", \
	  	sc->dev_name, \
	  	(speed100 ? "100" : "10"), \
	  	(full_duplex ? "Full" : "Half"));		
    }	
    else {
        tsec_printf("TSEC - NO LINK on %s\n", \
	  	sc->dev_name);	
    }
#else	
    tsec_printf("TSEC - No PHY interface specified for %s\n", \
	  	sc->dev_name);	
    HAL_RESTORE_INTERRUPTS(int_state);
    return false;	
#endif
	
    if (!tsec_eth_reset(sc, enaddr, (full_duplex ? RESET_FULL_DUPLEX
			: 0x00000000) | (speed100 ? RESET_100MB : 0x00000000) | (speed1000 ? RESET_1000MB : 0x00000000)))
    {
      HAL_RESTORE_INTERRUPTS(int_state);
      return false;
    }
    (sc->funs->eth_drv->init)(sc, (unsigned char *) &enaddr);

#if defined(CYGPKG_DEVS_ETH_PHY) && defined(CYGINT_IO_ETH_INT_SUPPORT_REQUIRED)
    // Create PHY interrupt if defined by user (can be used to handle link state change)
    if(qi->create_phy_irq != NULL)
	 qi->create_phy_irq(sc);
#endif

    HAL_RESTORE_INTERRUPTS(int_state);
	
    return true;
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
    int link = 0, speed100 = 0, full_duplex = 0, speed1000 = 0;
    cyg_uint16 phy_state = 0;	
    CYG_INTERRUPT_STATE istate;  
    struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;

    tsec_printf("TSEC - Start for %s\n", sc->dev_name);

#if 0
    //if stopped
    if ((qi->tsec->maccfg1 & (MACCFG1_RXEN | MACCFG1_TXEN)) != (MACCFG1_RXEN
			| MACCFG1_TXEN))
    {
#endif

#ifdef CYGPKG_KERNEL_SMP_SUPPORT  
      cyg_drv_spinlock_spin_intsave( &qi->tx_spin_lock, &istate); 
      cyg_drv_spinlock_spin( &qi->rx_spin_lock );
#else
      HAL_DISABLE_INTERRUPTS(istate);
#endif
	  
      // Initialize DMACTRL
      qi->tsec->dmactrl &= ~(DMACTRL_GRS| DMACTRL_GTS);

      // Clear THLT/RHLT
      qi->tsec->tstat = TSTAT_THLT | TSTAT_TXF;
      qi->tsec->rstat = RSTAT_QHLT | RSTAT_RXF;

      if(flags)
      {
#ifdef CYGPKG_DEVS_ETH_PHY      
        //redo autonegociation
        phy_state = _eth_phy_state(qi->phy);

        if(phy_state & ETH_PHY_STAT_1000MB)
	   speed1000 = 1;		
        if(phy_state & ETH_PHY_STAT_100MB)
          speed100 = 1;
        if(phy_state & ETH_PHY_STAT_FDX)
          full_duplex  = 1;
        if(phy_state & ETH_PHY_STAT_LINK){
	   link = 1;
          tsec_printf("Ethernet Mode (%s): %sMbps/%s\n", \
	  	sc->dev_name, \
	  	(speed100 ? "100" : "10"), \
	  	(full_duplex ? "Full" : "Half"));	
        }	
        else {
          tsec_printf("TSEC - NO LINK on %s\n", \
	  	sc->dev_name);	
        }
#else
        tsec_printf("NO PHY interface specified for %s\n", \
	  	sc->dev_name);	
#endif
		
        if (!link)
        {
          //the generic driver will set some flags
          (sc->funs->stop)(sc);
        }
        else
        {
          //set MAC connection parameters
          qi->tsec->maccfg2 = 0x00007000
          	                            | (speed1000 ? MACCFG2_IF_MODE_BYTE : MACCFG2_IF_MODE_NIBBLE)
						| MACCFG2_PAD_CRC | ((full_duplex) ? MACCFG2_FULL_DUPLEX
						: 0x0);
          qi->tsec->ecntrl = ECNTRL_STEN | ((speed100) ? ECNTRL_R100M : 0);

#ifdef CYGPKG_DEVS_ETH_POWERPC_TSEC_RMII
          qi->tsec->ecntrl |= ECNTRL_RMM;
#endif		  

          // Enable Rx and Tx
          qi->tsec->maccfg1 |= (MACCFG1_RXEN| MACCFG1_TXEN);
        }
      }
      else
      {
        qi->tsec->maccfg1 |= (MACCFG1_RXEN| MACCFG1_TXEN);
      }
	  
    // Unmask interrupt
#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
    qi->tsec->imask = IMASK_DEFAULT;
#endif
	  
    qi->state = 1;

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
  cyg_drv_spinlock_clear( &qi->rx_spin_lock );
  cyg_drv_spinlock_clear_intsave( &qi->tx_spin_lock, istate); 
#else
  HAL_RESTORE_INTERRUPTS(istate);
#endif

#if 0
    }
#endif	
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
		return 0;
		break;
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

    return (qi->txactive < CYGNUM_DEVS_ETH_POWERPC_TSEC_TxNUM);
}

//
// This routine is called to send data to the hardware.
static void tsec_eth_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
		int sg_len, int total_len, unsigned long key)
{
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
  volatile struct tsec_bd *txbd, *txfirst;
  volatile unsigned char *bp;
  int i, txindex;
  
#ifdef CYGPKG_KERNEL_SMP_SUPPORT  
  CYG_INTERRUPT_STATE istate;  
  cyg_drv_spinlock_spin_intsave( &qi->tx_spin_lock, &istate); 
#endif

  // Find a free buffer
  txbd = txfirst = qi->txbd;
  while (txbd->ctrl & FEC_BD_Tx_Ready)
  {
    // This buffer is busy, move to next one
    if (txbd->ctrl & FEC_BD_Tx_Wrap)
    {
      txbd = qi->tbase;
    }
    else
    {
      txbd++;
    }
    if (txbd == txfirst)
    {
      tsec_printf("TSEC - No free xmit buffers for %s\n", \
	  	 sc->dev_name);
    }
  }
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

  // Send it on it's way
  txbd->ctrl |= FEC_BD_Tx_Ready | FEC_BD_Tx_Last | FEC_BD_Tx_TC;
  qi->txactive++;

  qi->tsec->tstat = TSTAT_THLT;
	
  // Remember the next buffer to try
  if (txbd->ctrl & FEC_BD_Tx_Wrap)
  {
    qi->txbd = qi->tbase;
  }
  else
  {
    qi->txbd = txbd + 1;
  }

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
  cyg_drv_spinlock_clear_intsave( &qi->tx_spin_lock, istate); 
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
  CYG_INTERRUPT_STATE istate;
	
#ifdef CYGPKG_KERNEL_SMP_SUPPORT  
  cyg_drv_spinlock_spin_intsave( &qi->rx_spin_lock, &istate); 
  qi->smp_stats[HAL_SMP_CPU_THIS()].rx_pkt++;
#endif

  rxbd = rxfirst = qi->rnext;
  while (true)
  {
    if ((rxbd->ctrl & FEC_BD_Rx_Empty) == 0)
    {
      qi->rxbd = rxbd; // Save for callback
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

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
  cyg_drv_spinlock_clear_intsave( &qi->rx_spin_lock, istate); 
#endif

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

  for (i = 0; i < sg_len; i++)
  {
    if (sg_list[i].buf != 0)
    {
      memcpy((void *) sg_list[i].buf, bp, sg_list[i].len);
      bp += sg_list[i].len;
    }
  }
  qi->rxbd->ctrl |= FEC_BD_Rx_Empty;
  qi->rxbd->length = 0;

}


static void tsec_eth_TxEvent(struct eth_drv_sc *sc)
{
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
  volatile struct tsec_bd *txbd;
  int key, txindex;

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
  qi->smp_stats[HAL_SMP_CPU_THIS()].tx_pkt++;
#endif

  txbd = qi->tnext;
  // Note: TC field is used to indicate the buffer has/had data in it
  while ((txbd->ctrl & (FEC_BD_Tx_Ready|FEC_BD_Tx_TC)) == FEC_BD_Tx_TC)
  {
    txindex = ((unsigned long) txbd - (unsigned long) qi->tbase)
				/ sizeof(*txbd);
    if ((key = qi->txkey[txindex]) != 0)
    {
      qi->txkey[txindex] = 0;
      (sc->funs->eth_drv->tx_done)(sc, key, 0);
    }
      if (--qi->txactive == 0)
      {
      }
      txbd->ctrl &= ~FEC_BD_Tx_TC;
      if (txbd->ctrl & FEC_BD_Tx_Wrap)
      {
        txbd = qi->tbase;
      }
      else
      {
        txbd++;
      }
  }
  // Remember where we left off
  qi->tnext = (struct tsec_bd *) txbd;

}

//
// Interrupt processing
//
static void tsec_eth_int(struct eth_drv_sc *sc)
{
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
  unsigned long event;
  int error = 0;
  int rx = 0;
  
  while ((event = qi->tsec->ievent) != 0)
  {
    // Reset the bits we are handling
    qi->tsec->ievent = event; 

    if ((event & IEVENT_BABR) != 0)
    {
      diag_printf("TSEC - Babbling receive error.\n");
    }
    if ((event & IEVENT_BSY) != 0)
    {
      diag_printf("TSEC - Busy condition interrupt.\n");
      rx = 1;
    }
    if ((event & IEVENT_EBERR) != 0)
    {
      diag_printf("TSEC - Internal bus error.\n");
    }
    if ((event & IEVENT_MSRO) != 0)
    {
      diag_printf("TSEC - MIB counter overflow.\n");
      // reset MIB
      qi->tsec->ecntrl |= ECNTRL_CLRCNT;
    }
    if ((event & IEVENT_BABT) != 0)
    {
      diag_printf("TSEC - Babbling transmit error.\n");
    }
    if ((event & IEVENT_TXE) != 0)
    {
      diag_printf("TSEC - Transmit error.\n");
    }
    if ((event & IEVENT_LC) != 0)
    {
      diag_printf("TSEC - Late collision.\n");
    }
    if ((event & IEVENT_CRL) != 0)
    {
      diag_printf("TSEC - Collision retry limit.\n");
    }
    if ((event & IEVENT_XFUN) != 0)
    {
      diag_printf("TSEC - Transmit FIFO underrun.\n");
      error = 1;
    }
    if ((event & IEVENT_DPE) != 0)
    {
      diag_printf("TSEC - Internal data parity error.\n");
    }
    if (error)
    {
      //unhalt transmition in case of an error
      error = 0;
      qi->tsec->rstat = RSTAT_QHLT;
    }

    if (((event & IEVENT_RXF) != 0) || (rx == 1))
    {
      rx = 0;
      tsec_printf("TSEC - Receive packet\n");
      tsec_eth_RxEvent(sc);
    }
    if ((event & IEVENT_TXF) != 0)
    {
      tsec_printf("TSEC - Transmit packet\n");
      tsec_eth_TxEvent(sc);
    }
  }
}

//
// Interrupt vector
//
static int tsec_eth_int_vector(struct eth_drv_sc *sc)
{
#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED  
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;
  return (qi->rx_irq_vector);
#else
  return 0;
#endif
}

#if ~defined(CYGINT_IO_ETH_INT_SUPPORT_REQUIRED) && defined(CYGPKG_KERNEL)
void tsec_fake_int(cyg_addrword_t param)
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
 * Display eTSEC RMON statistics
 *
 */
void tsec_disp_stats(struct eth_drv_sc *sc)
{
  int i;
  struct tsec_eth_info *qi = (struct tsec_eth_info *) sc->driver_private;

  diag_printf("Ethernet Stats\n");

  diag_printf("%6s %10d\n\r", \
  	"tr64", qi->tsec->rmon.tr64);
  diag_printf("%6s %10d\n\r", \
  	"tr127", qi->tsec->rmon.tr127);
  diag_printf("%6s %10d\n\r", \
  	"tr255", qi->tsec->rmon.tr255);
  diag_printf("%6s %10d\n\r", \
  	"tr511", qi->tsec->rmon.tr511);
  diag_printf("%6s %10d\n\r", \
  	"tr1k", qi->tsec->rmon.tr1k);
  diag_printf("%6s %10d\n\r", \
  	"trmax", qi->tsec->rmon.trmax);
  diag_printf("%6s %10d\n\r", \
  	"trmgv", qi->tsec->rmon.trmgv);
  diag_printf("%6s %10d\n\r", \
  	"rbyt", qi->tsec->rmon.rbyt);
  diag_printf("%6s %10d\n\r", \
  	"rpkt", qi->tsec->rmon.rpkt);
  diag_printf("%6s %10d\n\r", \
  	"rfcs", qi->tsec->rmon.rfcs);
  diag_printf("%6s %10d\n\r", \
  	"rmca", qi->tsec->rmon.rmca);
  diag_printf("%6s %10d\n\r", \
  	"rbca", qi->tsec->rmon.rbca);
  diag_printf("%6s %10d\n", \
  	"rxcf", qi->tsec->rmon.rxcf);
  diag_printf("%6s %10d\n\r", \
  	"rxpf", qi->tsec->rmon.rxpf);
  diag_printf("%6s %10d\n\r", \
  	"rxuo", qi->tsec->rmon.rxuo);
  diag_printf("%6s %10d\n\r", \
  	"raln", qi->tsec->rmon.raln);
  diag_printf("%6s %10d\n\r", \
  	"rflr", qi->tsec->rmon.rflr);
  diag_printf("%6s %10d\n\r", \
  	"rcde", qi->tsec->rmon.rcde);
  diag_printf("%6s %10d\n\r", \
  	"rcse", qi->tsec->rmon.rcse);
  diag_printf("%6s %10d\n\r", \
  	"rund", qi->tsec->rmon.rund);
  diag_printf("%6s %10d\n\r", \
  	"rovr", qi->tsec->rmon.rovr);
  diag_printf("%6s %10d\n", \
  	"rfrg", qi->tsec->rmon.rfrg);
  diag_printf("%6s %10d\n\r", \
  	"rjbr", qi->tsec->rmon.rjbr);
  diag_printf("%6s %10d\n\r", \
  	"rdrp", qi->tsec->rmon.rdrp);
  diag_printf("%6s %10d\n\r", \
  	"tbyt", qi->tsec->rmon.tbyt);
  diag_printf("%6s %10d\n\r", \
  	"tpkt", qi->tsec->rmon.tpkt);
  diag_printf("%6s %10d\n\r", \
  	"tmca", qi->tsec->rmon.tmca);
  diag_printf("%6s %10d\n\r", \
  	"tbca", qi->tsec->rmon.tbca);
  diag_printf("%6s %10d\n\r", \
  	"txpf", qi->tsec->rmon.txpf);
  diag_printf("%6s %10d\n\r", \
  	"tdfr", qi->tsec->rmon.tdfr);
  diag_printf("%6s %10d\n\r", \
  	"tedf", qi->tsec->rmon.tedf);
  diag_printf("%6s %10d\n\r", \
  	"tscl", qi->tsec->rmon.tscl);
  diag_printf("%6s %10d\n\r", \
  	"tmcl", qi->tsec->rmon.tmcl);
  diag_printf("%6s %10d\n\r", \
  	"tlcl", qi->tsec->rmon.tlcl);
  diag_printf("%6s %10d\n\r", \
  	"txcl", qi->tsec->rmon.txcl); 
  diag_printf("%6s %10d\n\r", \
  	"tncl", qi->tsec->rmon.tncl);
  diag_printf("%6s %10d\n\r", \
  	"tdrp", qi->tsec->rmon.tdrp);
  diag_printf("%6s %10d\n\r", \
  	"tjbr", qi->tsec->rmon.tjbr);
  diag_printf("%6s %10d\n\r", \
  	"tfcs", qi->tsec->rmon.tfcs);
  diag_printf("%6s %10d\n\r", \
  	"txcf", qi->tsec->rmon.txcf);
  diag_printf("%6s %10d\n\r", \
  	"tovr", qi->tsec->rmon.tovr);
  diag_printf("%6s %10d\n\r", \
  	"tund", qi->tsec->rmon.tund);

#ifdef CYGPKG_KERNEL_SMP_SUPPORT
  for(i = 0 ; i < HAL_SMP_CPU_MAX; i++)
  {
    diag_printf("%5s%1d %10d\n\r", \
  	"smprx", i, qi->smp_stats[i].rx_pkt);
    diag_printf("%5s%1d %10d\n\r", \
  	"smptx", i, qi->smp_stats[i].tx_pkt);  	
  }
#endif

}

void disp_eth_stats(int eth)
{
#if 0
   if(eth == 0)
     tsec_disp_stats( &tsec_eth0_sc );
   else
     tsec_disp_stats( &tsec_eth1_sc );
   #endif
}

