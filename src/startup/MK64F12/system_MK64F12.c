/*
** ###################################################################
**     Processor:           MK64FN1M0VMD12
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K64P144M120SF5RM, Rev.1, July 2013
**     Version:             rev. 1.0, 2013-08-12
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2013 Freescale, Inc. All Rights Reserved.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2013-08-12)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MK64F12
 * @version 1.0
 * @date 2013-08-12
 * @brief Device specific configuration file for MK64F12 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "device/fsl_device_registers.h"

#define DISABLE_WDOG    1

/* Predefined clock setups
   1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
         Reference clock source for MCG module is an external clock source 50MHz
         USB clock divider is set for USB to receive 48MHz input clock.
         Core clock = 120MHz, BusClock = 60MHz
*/

/*----------------------------------------------------------------------------
  Define clock source values
 *----------------------------------------------------------------------------*/
// CLOCK_SETUP 1
#define CPU_XTAL_CLK_HZ                 50000000u /* Value of the external crystal or oscillator clock frequency in Hz */
#define CPU_XTAL32k_CLK_HZ              32768u   /* Value of the external 32k crystal or oscillator clock frequency in Hz */
#define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
#define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
#define CPU_INT_IRC_CLK_HZ              48000000u /* Value of the 48M internal oscillator clock frequency in Hz  */
#define DEFAULT_SYSTEM_CLOCK            120000000u /* Default System clock value */

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = 20485760u;   // Chip resets to FEI mode (clock setup 0)

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */
#if (DISABLE_WDOG)
  /* Disable the WDOG module */
  /* WDOG->UNLOCK: WDOGUNLOCK=0xC520 */
  WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xC520); /* Key 1 */
  /* WDOG->UNLOCK: WDOGUNLOCK=0xD928 */
  WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xD928); /* Key 2 */
  /* WDOG->STCTRLH: ?=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,?=0,?=1,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
  WDOG->STCTRLH = WDOG_STCTRLH_BYTESEL(0x00) |
                 WDOG_STCTRLH_WAITEN_MASK |
                 WDOG_STCTRLH_STOPEN_MASK |
                 WDOG_STCTRLH_ALLOWUPDATE_MASK |
                 WDOG_STCTRLH_CLKSRC_MASK |
                 0x0100U;
#endif /* (DISABLE_WDOG) */

   /*
   * Release hold with ACKISO:  Only has an effect if recovering from VLLSx.
   * if ACKISO is set you must clear ackiso before initializing the PLL
   * if osc enabled in low power modes - enable it first before ack
   */
  if (PMC->REGSC &  PMC_REGSC_ACKISO_MASK)
  {
      PMC->REGSC |= PMC_REGSC_ACKISO_MASK;
  }
}

/* ----------------------------------------------------------------------------
   -- SystemConfigureClocks()
   ---------------------------------------------------------------------------- */

void SystemConfigureClocks(uint32_t clkdiv1, bool enableUsb)
{
  // CMSIS CLOCK_SETUP=1

  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */

  /* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV3=1,OUTDIV4=4,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = clkdiv1; /* Update system prescalers */

  /* SIM->SOPT2: PLLFLLSEL=1 */
  SIM_BWR_SOPT2_PLLFLLSEL(SIM, 1); /* Select PLL as a clock source for various peripherals */
  /* SIM->SOPT1: OSC32KSEL=3 */
  SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03); /* LPO 1kHz oscillator drives 32 kHz clock for various peripherals */
  /* PORTA->PCR[18]: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* Switch to FBE Mode */
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
  MCG->C2 = MCG_C2_RANGE0(0x02);
  /* OSC->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC->CR = OSC_CR_ERCLKEN_MASK;
  /* MCG->C7: OSCSEL=0 */
  MCG->C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL_MASK);
  /* MCG->C1: CLKS=2,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x05) | MCG_C1_IRCLKEN_MASK);
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0x13 */
  MCG->C5 = MCG_C5_PRDIV0(0x13);
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0x18 */
  MCG->C6 = MCG_C6_VDIV0(0x18);
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to PBE Mode */
  /* MCG->C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0x18 */
  MCG->C6 = (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x18));
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
  }
  /* Switch to PEE Mode */
  /* MCG->C1: CLKS=0,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x05) | MCG_C1_IRCLKEN_MASK);
  while((MCG->S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
  }
  // Set USB to 48MHz input clock if requested.
  if (enableUsb)
  {
      /* Set USB input clock to 48MHz  */
      /* SIM->CLKDIV2: USBDIV=4,USBFRAC=1 */
      SIM->CLKDIV2 = (uint32_t)((SIM->CLKDIV2 & (uint32_t)~(uint32_t)(
                     SIM_CLKDIV2_USBDIV(0x03)
                    )) | (uint32_t)(
                     SIM_CLKDIV2_USBDIV(0x04) |
                     SIM_CLKDIV2_USBFRAC_MASK
                    ));
  }
}
