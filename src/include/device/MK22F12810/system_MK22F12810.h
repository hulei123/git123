/*
** ###################################################################
**     Processors:         MK22F12810
**
**     Reference manual:   K22P121M100SF9RM, Rev. 0.1, December 17, 2013
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
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
**     http: www.freescale.com
**     mail: support@freescale.com
**
** ###################################################################
*/



/*!
 * @file MK22F12810
 * @brief Device specific configuration file for MK02F12810 (header file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#ifndef SYSTEM_MK22F12810_H_
#define SYSTEM_MK22F12810_H_                     /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define DISABLE_WDOG    1

#ifndef CLOCK_SETUP
  #define CLOCK_SETUP   0
#endif

/* Predefined clock setups
   0 ... Default  part configuration  
         Multipurpose Clock Generator (MCG) in FEI mode.
         Reference clock source for MCG module: Slow internal reference clock
         Core clock = 20.97152MHz
         Bus clock  = 20.97152MHz
   1 ... Maximum achievable clock frequency configuration  
         Multipurpose Clock Generator (MCG) in FEE mode.
         Reference clock source for MCG module: System oscillator 0 reference clock
         Core clock = 80MHz
         Bus clock  = 40MHz
   2 ... Internally clocked, ready for Very Low Power Run mode.  
         Multipurpose Clock Generator (MCG) in BLPI mode.
         Reference clock source for MCG module: Fast internal reference clock
         Core clock = 4MHz
         Bus clock  = 4MHz
   3 ... Externally clocked, ready for Very Low Power Run mode.  
         Multipurpose Clock Generator (MCG) in BLPE mode.
         Reference clock source for MCG module: System oscillator 0 reference clock
         Core clock = 4MHz
         Bus clock  = 4MHz
   4 ... Maximum achievable clock frequency configuration, ready for USB operation.  
         Multipurpose Clock Generator (MCG) in FEE mode.
         Reference clock source for MCG module: System oscillator 0 reference clock
         Core clock = 80MHz
         Bus clock  = 40MHz
 */
 
/* Define clock source values */
                
#define CPU_XTAL_CLK_HZ                 8000000u   /* Value of the external crystal or oscillator clock frequency in Hz */
#define CPU_XTAL32k_CLK_HZ              0u /* RTC oscillator is disabled */
#define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
#define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
#define CPU_INT_IRC_CLK_HZ              48000000u /* Value of the 48M internal oscillator clock frequency in Hz  */
    
#if (CLOCK_SETUP == 0)
  #define DEFAULT_SYSTEM_CLOCK            20971520u /* Default System clock value */
#elif (CLOCK_SETUP == 1)
  #define DEFAULT_SYSTEM_CLOCK            80000000u /* Default System clock value */
#elif (CLOCK_SETUP == 2)
  #define DEFAULT_SYSTEM_CLOCK            4000000u /* Default System clock value */
#elif (CLOCK_SETUP == 3)
  #define DEFAULT_SYSTEM_CLOCK            4000000u /* Default System clock value */
#elif (CLOCK_SETUP == 4)
  #define DEFAULT_SYSTEM_CLOCK            80000000u /* Default System clock value */
#endif
/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the processor
 * core clock. This variable can be used by the user application to setup the
 * SysTick timer or configure other parameters. It may also be used by debugger to
 * query the frequency of the debug timer or configure the trace clock speed
 * SystemCoreClock is initialized with a correct predefined value.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit (void);

/**
 * @brief Updates the SystemCoreClock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif  /* #if !defined(SYSTEM_MK22F12810_H_) */
