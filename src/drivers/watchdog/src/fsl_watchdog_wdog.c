/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_platform_common.h" // needed for NULL
#include "device/fsl_device_registers.h"
#include "watchdog/fsl_watchdog.h"
#include "microseconds/microseconds.h"
#include "bootloader_common.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"

/*-----------------------------------------------------------------------------
  Define WDOG registers
Note:
    Here overwrites the default definition of below macros in headers
 -------------------------------------------------------------------------------*/
#ifdef WDOG_CS
#undef WDOG_CS
#endif
#ifdef WDOG_CNT
#undef WDOG_CNT
#endif
#ifdef WDOG_TOVAL
#undef WDOG_TOVAL
#endif

#define WDOG_CS(base)     (*(volatile uint32_t*)(base))
#define WDOG_CNT(base)    (*(volatile uint32_t*)((base)+4))
#define WDOG_TOVAL(base)  (*(volatile uint32_t*)((base)+8))
#define WDOG_UNLOCK_MAGIC_NUMBER   (0xd928c520u)

#ifdef WDOG0_BASE
#define WDOG_BASE   WDOG0_BASE
#elif defined(WDOG_BASE)
#else
#error "Unkown WDOG module"
#endif


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
//! @brief Initializes the Watchdog module.
void fsl_watchdog_init(void)
{
#if defined(PCC_BASE_ADDRS)
    WDOG_TOVAL(WDOG_BASE) |= 0; // Leave reset value of timeout
    WDOG_WR_CS_UPDATE(WDOG, 1); // Set WDOG_CS[UPDATE] to 1
    WDOG_CNT(WDOG_BASE) = WDOG_UNLOCK_MAGIC_NUMBER; // Unlock WDOG
    WDOG_WR_CS_EN(WDOG, 0); // Disable WDOG;
#else  
    WDOG_TOVAL(WDOG_BASE) |= 0; // Leave reset value of timeout
    WDOG_CS(WDOG_BASE) |= BM_WDOG_CS_UPDATE; // Set WDOG_CS[UPDATE] to 1
    WDOG_CNT(WDOG_BASE) = WDOG_UNLOCK_MAGIC_NUMBER; // Unlock WDOG
    WDOG_CS(WDOG_BASE) &= (uint32_t)~BM_WDOG_CS_EN; // Disable WDOG;
#endif    
}

//! @brief Deinitializes the device.
//!
//! Clears the control register and turns off the clock to the module.
void fsl_watchdog_deinit(void)
{
#if defined(PCC_BASE_ADDRS)
    WDOG_CNT(WDOG_BASE) = WDOG_UNLOCK_MAGIC_NUMBER; // Unlock WDOG
    WDOG_WR_CS_EN(WDOG, 1); // Enable WDOG;
#else  
    WDOG_CNT(WDOG_BASE) = WDOG_UNLOCK_MAGIC_NUMBER; // Unlock WDOG
    WDOG_CS(WDOG_BASE) |= BM_WDOG_CS_EN; // Enable WDOG;
#endif    
    uint32_t last_wdog_cnt = WDOG_CNT(WDOG_BASE);

    // 128 bus clock and 2 WDOG clock cylcle are needed to wait WDOG to get ready.
    // considering that maximum core clock is 48Mhz and WDOG clock is 1KHz,
    // about 96000 core clock cylcles are needed to wait until WDOG_CNT start increasing.
    for(uint32_t i=0; i<96000; i++)
    {
        if(WDOG_CNT(WDOG_BASE) != last_wdog_cnt)
        {
            break;
        }
    }
}

//! @brief Service the watchdog module.
void fsl_watchdog_service(void)
{

}