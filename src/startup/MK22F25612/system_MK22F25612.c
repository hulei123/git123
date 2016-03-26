/*
** ###################################################################
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K22P121M120SF8RM, Rev. 0.61, January 10, 2014
**     Version:             rev. 1.4, 2014-02-10
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2014 Freescale, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2013-09-17)
**         Initial version.
**     - rev. 1.1 (2013-10-29)
**         Definition of BITBAND macros updated to support peripherals with 32-bit acces disabled.
**     - rev. 1.2 (2013-12-20)
**         Update according to reference manual rev. 0.6,
**     - rev. 1.3 (2014-02-06)
**         Update according to reference manual rev. 0.61,
**     - rev. 1.4 (2014-02-10)
**         The declaration of clock configurations has been moved to separate header file system_MK22F25612.h
**
** ###################################################################
*/

/*!
 * @file MK22F25612
 * @version 1.4
 * @date 2014-02-10
 * @brief Device specific configuration file for MK22F25612 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "device/fsl_device_registers.h"



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

#if (DISABLE_WDOG)
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
#if (CLOCK_SETUP == 0)
  /* SMC_PMPROT: AHSRUN=1,?=0,AVLP=1,?=0,ALLS=0,?=0,AVLLS=0,?=0 */
//  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=0,OUTDIV3=1,OUTDIV4=1,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x00) |
                SIM_CLKDIV1_OUTDIV4(0x01); /* Update system prescalers */
  /* SIM_SOPT2: PLLFLLSEL=0 */
  SIM_SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL(0x03)); /* Select FLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=2 */
  SIM_SOPT1 = (uint32_t)((SIM_SOPT1 & (uint32_t)~(uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x01)
              )) | (uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x02)
              ));                      /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* Switch to FEI Mode */
    /* MCG_C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = MCG_C1_CLKS(0x00) |
           MCG_C1_FRDIV(0x00) |
           MCG_C1_IREFS_MASK |
           MCG_C1_IRCLKEN_MASK;
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK |
              MCG_C2_LP_MASK |
              MCG_C2_IRCS_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK
             ));
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    /* OSC_CR: ERCLKEN=0,?=0,EREFSTEN=0,?=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC_CR = 0x00U;
    /* MCG_C7: OSCSEL=0 */
    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
    /* MCG_C5: ?=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
    MCG_C5 = MCG_C5_PRDIV0(0x00);
    /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
    MCG_C6 = MCG_C6_VDIV0(0x00);
    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x00U) {  /* Wait until output of the FLL is selected */
    }
#elif (CLOCK_SETUP == 1)
  /* SMC_PMPROT: AHSRUN=1,?=0,AVLP=1,?=0,ALLS=0,?=0,AVLLS=0,?=0 */
//  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV3=1,OUTDIV4=4,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x01) |
                SIM_CLKDIV1_OUTDIV4(0x04); /* Update system prescalers */
  /* SIM_SOPT2: PLLFLLSEL=1 */
  SIM_SOPT2 = (uint32_t)((SIM_SOPT2 & (uint32_t)~(uint32_t)(
               SIM_SOPT2_PLLFLLSEL(0x02)
              )) | (uint32_t)(
               SIM_SOPT2_PLLFLLSEL(0x01)
              ));                      /* Select PLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=2 */
  SIM_SOPT1 = (uint32_t)((SIM_SOPT1 & (uint32_t)~(uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x01)
              )) | (uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x02)
              ));                      /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
    /* Switch to FBE Mode */
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK |
              MCG_C2_LP_MASK |
              MCG_C2_IRCS_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK
             ));
    /* OSC_CR: ERCLKEN=1,?=0,EREFSTEN=0,?=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC_CR = OSC_CR_ERCLKEN_MASK;
    /* MCG_C7: OSCSEL=0 */
    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
    /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    /* MCG_C5: ?=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
    MCG_C5 = MCG_C5_PRDIV0(0x01);
    /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=6 */
    MCG_C6 = MCG_C6_VDIV0(0x06);
    while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
    }
    while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x08U) {  /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to PBE Mode */
    /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=6 */
    MCG_C6 = (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x06));
    while((MCG_S & 0x0CU) != 0x08U) {  /* Wait until external reference clock is selected as MCG output */
  }
    while((MCG_S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
  }
  /* Switch to PEE Mode */
    /* MCG_C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
    while((MCG_S & 0x0CU) != 0x0CU) {  /* Wait until output of the PLL is selected */
    }
#elif (CLOCK_SETUP == 2)
  /* SMC_PMPROT: AHSRUN=1,?=0,AVLP=1,?=0,ALLS=0,?=0,AVLLS=0,?=0 */
//  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=0,OUTDIV3=0,OUTDIV4=4,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x00) |
                SIM_CLKDIV1_OUTDIV4(0x04); /* Update system prescalers */
  /* SIM_SOPT2: PLLFLLSEL=3 */
  SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(0x03); /* Select PLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=2 */
  SIM_SOPT1 = (uint32_t)((SIM_SOPT1 & (uint32_t)~(uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x01)
              )) | (uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x02)
              ));                      /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* MCG_SC: FCRDIV=0,LOCS0=0 */
  MCG_SC &= (uint8_t)~(uint8_t)((MCG_SC_FCRDIV(0x07) | MCG_SC_LOCS0_MASK));
  /* Switch to FBI Mode */
    /* MCG_C1: CLKS=1,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = MCG_C1_CLKS(0x01) |
           MCG_C1_FRDIV(0x00) |
           MCG_C1_IREFS_MASK |
           MCG_C1_IRCLKEN_MASK;
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=1 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK |
              MCG_C2_LP_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK |
              MCG_C2_IRCS_MASK
             ));
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    /* OSC_CR: ERCLKEN=1,?=0,EREFSTEN=0,?=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC_CR = OSC_CR_ERCLKEN_MASK;
    /* MCG_C7: OSCSEL=0 */
    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
    /* MCG_C5: ?=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
    MCG_C5 = MCG_C5_PRDIV0(0x00);
    /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
    MCG_C6 = MCG_C6_VDIV0(0x00);
    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x04U) {  /* Wait until internal reference clock is selected as MCG output */
  }
  /* Switch to BLPI Mode */
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=1,IRCS=1 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK |
              MCG_C2_LP_MASK |
              MCG_C2_IRCS_MASK
             ));
    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
    }
    while((MCG_S & MCG_S_IRCST_MASK) == 0x00U) { /* Check that the fast external reference clock is selected. */
  }
#elif (CLOCK_SETUP == 3)
  /* SMC_PMPROT: AHSRUN=1,?=0,AVLP=1,?=0,ALLS=0,?=0,AVLLS=0,?=0 */
//  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
  /* SIM_CLKDIV1: OUTDIV1=1,OUTDIV2=1,OUTDIV3=1,OUTDIV4=7,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x01) |
                SIM_CLKDIV1_OUTDIV2(0x01) |
                SIM_CLKDIV1_OUTDIV4(0x07); /* Update system prescalers */
  /* SIM_SOPT2: PLLFLLSEL=3 */
  SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(0x03); /* Select PLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=2 */
  SIM_SOPT1 = (uint32_t)((SIM_SOPT1 & (uint32_t)~(uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x01)
              )) | (uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x02)
              ));                      /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* MCG_SC: FCRDIV=1,LOCS0=0 */
  MCG_SC = (uint8_t)((MCG_SC & (uint8_t)~(uint8_t)(
            MCG_SC_FCRDIV(0x06) |
            MCG_SC_LOCS0_MASK
           )) | (uint8_t)(
            MCG_SC_FCRDIV(0x01)
           ));
    /* Switch to FBE Mode */
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=1 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK |
              MCG_C2_LP_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK |
              MCG_C2_IRCS_MASK
             ));
    /* OSC_CR: ERCLKEN=0,?=0,EREFSTEN=0,?=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC_CR = 0x00U;
    /* MCG_C7: OSCSEL=0 */
    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
    /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    /* MCG_C5: ?=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
    MCG_C5 = MCG_C5_PRDIV0(0x00);
    /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
    MCG_C6 = MCG_C6_VDIV0(0x00);
    while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
    }
    while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x08U) {  /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to BLPE Mode */
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=1,IRCS=1 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK |
              MCG_C2_LP_MASK |
              MCG_C2_IRCS_MASK
             ));
    while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
    }
    while((MCG_S & 0x0CU) != 0x08U) {  /* Wait until external reference clock is selected as MCG output */
    }
#elif (CLOCK_SETUP == 4)
  /* SMC_PMPROT: AHSRUN=1,?=0,AVLP=1,?=0,ALLS=0,?=0,AVLLS=0,?=0 */
//  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV3=1,OUTDIV4=4,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0,?=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x01) |
                SIM_CLKDIV1_OUTDIV4(0x04); /* Update system prescalers */
  /* SIM_CLKDIV2: USBDIV=4,USBFRAC=1 */
  SIM_CLKDIV2 = (uint32_t)((SIM_CLKDIV2 & (uint32_t)~(uint32_t)(
                 SIM_CLKDIV2_USBDIV(0x03)
                )) | (uint32_t)(
                 SIM_CLKDIV2_USBDIV(0x04) |
                 SIM_CLKDIV2_USBFRAC_MASK
                ));                    /* Update USB clock prescalers */
  /* SIM_SOPT2: PLLFLLSEL=1 */
  SIM_SOPT2 = (uint32_t)((SIM_SOPT2 & (uint32_t)~(uint32_t)(
               SIM_SOPT2_PLLFLLSEL(0x02)
              )) | (uint32_t)(
               SIM_SOPT2_PLLFLLSEL(0x01)
              ));                      /* Select PLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=2 */
  SIM_SOPT1 = (uint32_t)((SIM_SOPT1 & (uint32_t)~(uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x01)
              )) | (uint32_t)(
               SIM_SOPT1_OSC32KSEL(0x02)
              ));                      /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
    /* Switch to FBE Mode */
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK |
              MCG_C2_LP_MASK |
              MCG_C2_IRCS_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK
             ));
    /* OSC_CR: ERCLKEN=1,?=0,EREFSTEN=0,?=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC_CR = OSC_CR_ERCLKEN_MASK;
    /* MCG_C7: OSCSEL=0 */
    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
    /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    /* MCG_C5: ?=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
    MCG_C5 = MCG_C5_PRDIV0(0x01);
    /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=6 */
    MCG_C6 = MCG_C6_VDIV0(0x06);
    while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
    }
    while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x08U) {  /* Wait until external reference clock is selected as MCG output */
    }
    /* Switch to PBE Mode */
    /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=6 */
    MCG_C6 = (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x06));
    while((MCG_S & 0x0CU) != 0x08U) {  /* Wait until external reference clock is selected as MCG output */
    }
    while((MCG_S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
    }
    /* Switch to PEE Mode */
    /* MCG_C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
    while((MCG_S & 0x0CU) != 0x0CU) {  /* Wait until output of the PLL is selected */
  }
#endif
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t MCGOUTClock;                                                        /* Variable to store output clock frequency of the MCG module */
  uint16_t Divider;

  if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x0u) {
    /* Output of FLL or PLL is selected */
    if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u) {
      /* FLL is selected */
      if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u) {
        /* External reference clock is selected */
        switch (MCG->C7 & MCG_C7_OSCSEL_MASK) {
        case 0x00u:
          MCGOUTClock = CPU_XTAL_CLK_HZ;                                       /* System oscillator drives MCG clock */
          break;
        case 0x01u:
          MCGOUTClock = CPU_XTAL32k_CLK_HZ;                                    /* RTC 32 kHz oscillator drives MCG clock */
          break;
        case 0x02u:
        default:
          MCGOUTClock = CPU_INT_IRC_CLK_HZ;                                              /* IRC 48MHz oscillator drives MCG clock */
        }
        if ((MCG->C2 & MCG_C2_RANGE_MASK) != 0x0u) {
          switch (MCG->C1 & MCG_C1_FRDIV_MASK) {
          case MCG_C1_FRDIV(0x07):
            Divider = 1536;
            break;
          case MCG_C1_FRDIV(0x06):
            Divider = 1280;
            break;
          default:
            Divider = (uint16_t)(32u << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
          }
        } else {/* ((MCG->C2 & MCG_C2_RANGE_MASK) != 0x0u) */
          Divider = (uint16_t)(1u << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
        }
        MCGOUTClock = (MCGOUTClock / Divider);  /* Calculate the divided FLL reference clock */
      } else { /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u)) */
        MCGOUTClock = CPU_INT_SLOW_CLK_HZ;                                     /* The slow internal reference clock is selected */
      } /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u)) */
      /* Select correct multiplier to calculate the MCG output clock  */
      switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) {
        case 0x0u:
          MCGOUTClock *= 640u;
          break;
        case 0x20u:
          MCGOUTClock *= 1280u;
          break;
        case 0x40u:
          MCGOUTClock *= 1920u;
          break;
        case 0x60u:
          MCGOUTClock *= 2560u;
          break;
        case 0x80u:
          MCGOUTClock *= 732u;
          break;
        case 0xA0u:
          MCGOUTClock *= 1464u;
          break;
        case 0xC0u:
          MCGOUTClock *= 2197u;
          break;
        case 0xE0u:
          MCGOUTClock *= 2929u;
          break;
        default:
          break;
      }
    } else { /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)) */
      /* PLL is selected */
      Divider = (1u + (MCG->C5 & MCG_C5_PRDIV0_MASK));
      MCGOUTClock = (uint32_t)(CPU_XTAL_CLK_HZ / Divider);                     /* Calculate the PLL reference clock */
      Divider = ((MCG->C6 & MCG_C6_VDIV0_MASK) + 24u);
      MCGOUTClock *= Divider;                       /* Calculate the MCG output clock */
    } /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40u) {
    /* Internal reference clock is selected */
    if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u) {
      MCGOUTClock = CPU_INT_SLOW_CLK_HZ;                                       /* Slow internal reference clock selected */
    } else { /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)) */
      MCGOUTClock = CPU_INT_FAST_CLK_HZ / (1 << ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT));  /* Fast internal reference clock selected */
    } /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u) {
    /* External reference clock is selected */
    switch (MCG->C7 & MCG_C7_OSCSEL_MASK) {
    case 0x00u:
      MCGOUTClock = CPU_XTAL_CLK_HZ;                                           /* System oscillator drives MCG clock */
      break;
    case 0x01u:
      MCGOUTClock = CPU_XTAL32k_CLK_HZ;                                        /* RTC 32 kHz oscillator drives MCG clock */
      break;
    case 0x02u:
    default:
      MCGOUTClock = CPU_INT_IRC_CLK_HZ;                                              /* IRC 48MHz oscillator drives MCG clock */
    }
  } else { /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)) */
    /* Reserved value */
    return;
  } /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)) */
  SystemCoreClock = (MCGOUTClock / (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));

}
