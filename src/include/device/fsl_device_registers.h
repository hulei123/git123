/*
** ###################################################################
**     Version:             rev. 1.0, 2014-05-15
**     Build:               b140528
**
**     Abstract:
**         Common include file for CMSIS register access layer headers.
**
**     Copyright (c) 2014 Freescale Semiconductor, Inc.
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
**     - rev. 1.0 (2014-05-15)
**         Customer release.
**
** ###################################################################
*/

#ifndef __FSL_DEVICE_REGISTERS_H__
#define __FSL_DEVICE_REGISTERS_H__

/*
 * Include the cpu specific register header files.
 *
 * The CPU macro should be declared in the project or makefile.
 */
#if (defined(CPU_MK02FN128VFM10) || defined(CPU_MK02FN64VFM10) || defined(CPU_MK02FN128VLF10) || \
    defined(CPU_MK02FN64VLF10) || defined(CPU_MK02FN128VLH10) || defined(CPU_MK02FN64VLH10))

    #define K02F12810_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK02F12810/MK02F12810.h"
    /* Extension register definitions */
    #include "device/MK02F12810/MK02F12810_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK02F12810/MK02F12810_features.h"

#elif (defined(CPU_MK11DX128AVLK5WS) || defined(CPU_MK11DX256AVLK5WS) || defined(CPU_MK11DN512AVLK5WS) || \
    defined(CPU_MK11DX128AVMC5WS) || defined(CPU_MK11DX256AVMC5WS) || defined(CPU_MK11DN512AVMC5WS))

    #define MK11DA5WS_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK11DA5WS/MK11DA5WS.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK11DA5WS/MK11DA5WS_adc.h"
    #include "device/MK11DA5WS/MK11DA5WS_cau.h"
    #include "device/MK11DA5WS/MK11DA5WS_cmp.h"
    #include "device/MK11DA5WS/MK11DA5WS_cmt.h"
    #include "device/MK11DA5WS/MK11DA5WS_crc.h"
    #include "device/MK11DA5WS/MK11DA5WS_dac.h"
    #include "device/MK11DA5WS/MK11DA5WS_dma.h"
    #include "device/MK11DA5WS/MK11DA5WS_dmamux.h"
    #include "device/MK11DA5WS/MK11DA5WS_dry.h"
    #include "device/MK11DA5WS/MK11DA5WS_ewm.h"
    #include "device/MK11DA5WS/MK11DA5WS_fmc.h"
    #include "device/MK11DA5WS/MK11DA5WS_ftfl.h"
    #include "device/MK11DA5WS/MK11DA5WS_ftm.h"
    #include "device/MK11DA5WS/MK11DA5WS_gpio.h"
    #include "device/MK11DA5WS/MK11DA5WS_i2c.h"
    #include "device/MK11DA5WS/MK11DA5WS_i2s.h"
    #include "device/MK11DA5WS/MK11DA5WS_llwu.h"
    #include "device/MK11DA5WS/MK11DA5WS_lptmr.h"
    #include "device/MK11DA5WS/MK11DA5WS_mcg.h"
    #include "device/MK11DA5WS/MK11DA5WS_mcm.h"
    #include "device/MK11DA5WS/MK11DA5WS_nv.h"
    #include "device/MK11DA5WS/MK11DA5WS_osc.h"
    #include "device/MK11DA5WS/MK11DA5WS_pdb.h"
    #include "device/MK11DA5WS/MK11DA5WS_pit.h"
    #include "device/MK11DA5WS/MK11DA5WS_pmc.h"
    #include "device/MK11DA5WS/MK11DA5WS_port.h"
    #include "device/MK11DA5WS/MK11DA5WS_rcm.h"
    #include "device/MK11DA5WS/MK11DA5WS_rfsys.h"
    #include "device/MK11DA5WS/MK11DA5WS_rfvbat.h"
    #include "device/MK11DA5WS/MK11DA5WS_rng.h"
    #include "device/MK11DA5WS/MK11DA5WS_rtc.h"
    #include "device/MK11DA5WS/MK11DA5WS_sim.h"
    #include "device/MK11DA5WS/MK11DA5WS_smc.h"
    #include "device/MK11DA5WS/MK11DA5WS_spi.h"
    #include "device/MK11DA5WS/MK11DA5WS_uart.h"
    #include "device/MK11DA5WS/MK11DA5WS_vref.h"
    #include "device/MK11DA5WS/MK11DA5WS_wdog.h"

#elif (defined(CPU_MK11DX128AVLK5) || defined(CPU_MK11DX256AVLK5) || defined(CPU_MK11DN512AVLK5) || \
    defined(CPU_MK11DX128AVMC5) || defined(CPU_MK11DX256AVMC5) || defined(CPU_MK11DN512AVMC5))

    #define MK11DA5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK11DA5/MK11DA5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK11DA5/MK11DA5_adc.h"
    #include "device/MK11DA5/MK11DA5_cau.h"
    #include "device/MK11DA5/MK11DA5_cmp.h"
    #include "device/MK11DA5/MK11DA5_cmt.h"
    #include "device/MK11DA5/MK11DA5_crc.h"
    #include "device/MK11DA5/MK11DA5_dac.h"
    #include "device/MK11DA5/MK11DA5_dma.h"
    #include "device/MK11DA5/MK11DA5_dmamux.h"
    #include "device/MK11DA5/MK11DA5_ewm.h"
    #include "device/MK11DA5/MK11DA5_fmc.h"
    #include "device/MK11DA5/MK11DA5_ftfl.h"
    #include "device/MK11DA5/MK11DA5_ftm.h"
    #include "device/MK11DA5/MK11DA5_gpio.h"
    #include "device/MK11DA5/MK11DA5_i2c.h"
    #include "device/MK11DA5/MK11DA5_i2s.h"
    #include "device/MK11DA5/MK11DA5_llwu.h"
    #include "device/MK11DA5/MK11DA5_lptmr.h"
    #include "device/MK11DA5/MK11DA5_mcg.h"
    #include "device/MK11DA5/MK11DA5_mcm.h"
    #include "device/MK11DA5/MK11DA5_nv.h"
    #include "device/MK11DA5/MK11DA5_osc.h"
    #include "device/MK11DA5/MK11DA5_pdb.h"
    #include "device/MK11DA5/MK11DA5_pit.h"
    #include "device/MK11DA5/MK11DA5_pmc.h"
    #include "device/MK11DA5/MK11DA5_port.h"
    #include "device/MK11DA5/MK11DA5_rcm.h"
    #include "device/MK11DA5/MK11DA5_rfsys.h"
    #include "device/MK11DA5/MK11DA5_rfvbat.h"
    #include "device/MK11DA5/MK11DA5_rng.h"
    #include "device/MK11DA5/MK11DA5_rtc.h"
    #include "device/MK11DA5/MK11DA5_sim.h"
    #include "device/MK11DA5/MK11DA5_smc.h"
    #include "device/MK11DA5/MK11DA5_spi.h"
    #include "device/MK11DA5/MK11DA5_uart.h"
    #include "device/MK11DA5/MK11DA5_vref.h"
    #include "device/MK11DA5/MK11DA5_wdog.h"

#elif (defined(CPU_MK11DX128VLK5WS) || defined(CPU_MK11DX256VLK5WS) || defined(CPU_MK11DN512VLK5WS) || \
    defined(CPU_MK11DX128VMC5WS) || defined(CPU_MK11DX256VMC5WS) || defined(CPU_MK11DN512VMC5WS))

    #define MK11D5WS_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK11D5WS/MK11D5WS.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK11D5WS/MK11D5WS_adc.h"
    #include "device/MK11D5WS/MK11D5WS_cau.h"
    #include "device/MK11D5WS/MK11D5WS_cmp.h"
    #include "device/MK11D5WS/MK11D5WS_cmt.h"
    #include "device/MK11D5WS/MK11D5WS_crc.h"
    #include "device/MK11D5WS/MK11D5WS_dac.h"
    #include "device/MK11D5WS/MK11D5WS_dma.h"
    #include "device/MK11D5WS/MK11D5WS_dmamux.h"
    #include "device/MK11D5WS/MK11D5WS_dry.h"
    #include "device/MK11D5WS/MK11D5WS_ewm.h"
    #include "device/MK11D5WS/MK11D5WS_fmc.h"
    #include "device/MK11D5WS/MK11D5WS_ftfl.h"
    #include "device/MK11D5WS/MK11D5WS_ftm.h"
    #include "device/MK11D5WS/MK11D5WS_gpio.h"
    #include "device/MK11D5WS/MK11D5WS_i2c.h"
    #include "device/MK11D5WS/MK11D5WS_i2s.h"
    #include "device/MK11D5WS/MK11D5WS_llwu.h"
    #include "device/MK11D5WS/MK11D5WS_lptmr.h"
    #include "device/MK11D5WS/MK11D5WS_mcg.h"
    #include "device/MK11D5WS/MK11D5WS_mcm.h"
    #include "device/MK11D5WS/MK11D5WS_nv.h"
    #include "device/MK11D5WS/MK11D5WS_osc.h"
    #include "device/MK11D5WS/MK11D5WS_pdb.h"
    #include "device/MK11D5WS/MK11D5WS_pit.h"
    #include "device/MK11D5WS/MK11D5WS_pmc.h"
    #include "device/MK11D5WS/MK11D5WS_port.h"
    #include "device/MK11D5WS/MK11D5WS_rcm.h"
    #include "device/MK11D5WS/MK11D5WS_rfsys.h"
    #include "device/MK11D5WS/MK11D5WS_rfvbat.h"
    #include "device/MK11D5WS/MK11D5WS_rng.h"
    #include "device/MK11D5WS/MK11D5WS_rtc.h"
    #include "device/MK11D5WS/MK11D5WS_sim.h"
    #include "device/MK11D5WS/MK11D5WS_smc.h"
    #include "device/MK11D5WS/MK11D5WS_spi.h"
    #include "device/MK11D5WS/MK11D5WS_uart.h"
    #include "device/MK11D5WS/MK11D5WS_vref.h"
    #include "device/MK11D5WS/MK11D5WS_wdog.h"

#elif (defined(CPU_MK11DX128VLK5) || defined(CPU_MK11DX256VLK5) || defined(CPU_MK11DN512VLK5) || \
    defined(CPU_MK11DX128VMC5) || defined(CPU_MK11DX256VMC5) || defined(CPU_MK11DN512VMC5))

    #define MK11D5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK11D5/MK11D5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK11D5/MK11D5_adc.h"
    #include "device/MK11D5/MK11D5_cau.h"
    #include "device/MK11D5/MK11D5_cmp.h"
    #include "device/MK11D5/MK11D5_cmt.h"
    #include "device/MK11D5/MK11D5_crc.h"
    #include "device/MK11D5/MK11D5_dac.h"
    #include "device/MK11D5/MK11D5_dma.h"
    #include "device/MK11D5/MK11D5_dmamux.h"
    #include "device/MK11D5/MK11D5_ewm.h"
    #include "device/MK11D5/MK11D5_fmc.h"
    #include "device/MK11D5/MK11D5_ftfl.h"
    #include "device/MK11D5/MK11D5_ftm.h"
    #include "device/MK11D5/MK11D5_gpio.h"
    #include "device/MK11D5/MK11D5_i2c.h"
    #include "device/MK11D5/MK11D5_i2s.h"
    #include "device/MK11D5/MK11D5_llwu.h"
    #include "device/MK11D5/MK11D5_lptmr.h"
    #include "device/MK11D5/MK11D5_mcg.h"
    #include "device/MK11D5/MK11D5_mcm.h"
    #include "device/MK11D5/MK11D5_nv.h"
    #include "device/MK11D5/MK11D5_osc.h"
    #include "device/MK11D5/MK11D5_pdb.h"
    #include "device/MK11D5/MK11D5_pit.h"
    #include "device/MK11D5/MK11D5_pmc.h"
    #include "device/MK11D5/MK11D5_port.h"
    #include "device/MK11D5/MK11D5_rcm.h"
    #include "device/MK11D5/MK11D5_rfsys.h"
    #include "device/MK11D5/MK11D5_rfvbat.h"
    #include "device/MK11D5/MK11D5_rng.h"
    #include "device/MK11D5/MK11D5_rtc.h"
    #include "device/MK11D5/MK11D5_sim.h"
    #include "device/MK11D5/MK11D5_smc.h"
    #include "device/MK11D5/MK11D5_spi.h"
    #include "device/MK11D5/MK11D5_uart.h"
    #include "device/MK11D5/MK11D5_vref.h"
    #include "device/MK11D5/MK11D5_wdog.h"

#elif (defined(CPU_MK12DX128VLH5) || defined(CPU_MK12DX256VLH5) || defined(CPU_MK12DN512VLH5) || \
    defined(CPU_MK12DX128VLK5) || defined(CPU_MK12DX256VLK5) || defined(CPU_MK12DN512VLK5) || \
    defined(CPU_MK12DX128VMC5) || defined(CPU_MK12DX256VMC5) || defined(CPU_MK12DN512VMC5) || \
    defined(CPU_MK12DX128VLF5) || defined(CPU_MK12DX256VLF5))

    #define MK12D5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK12D5/MK12D5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK12D5/MK12D5_adc.h"
    #include "device/MK12D5/MK12D5_cmp.h"
    #include "device/MK12D5/MK12D5_cmt.h"
    #include "device/MK12D5/MK12D5_crc.h"
    #include "device/MK12D5/MK12D5_dac.h"
    #include "device/MK12D5/MK12D5_dma.h"
    #include "device/MK12D5/MK12D5_dmamux.h"
    #include "device/MK12D5/MK12D5_ewm.h"
    #include "device/MK12D5/MK12D5_fmc.h"
    #include "device/MK12D5/MK12D5_ftfl.h"
    #include "device/MK12D5/MK12D5_ftm.h"
    #include "device/MK12D5/MK12D5_gpio.h"
    #include "device/MK12D5/MK12D5_i2c.h"
    #include "device/MK12D5/MK12D5_i2s.h"
    #include "device/MK12D5/MK12D5_llwu.h"
    #include "device/MK12D5/MK12D5_lptmr.h"
    #include "device/MK12D5/MK12D5_mcg.h"
    #include "device/MK12D5/MK12D5_mcm.h"
    #include "device/MK12D5/MK12D5_nv.h"
    #include "device/MK12D5/MK12D5_osc.h"
    #include "device/MK12D5/MK12D5_pdb.h"
    #include "device/MK12D5/MK12D5_pit.h"
    #include "device/MK12D5/MK12D5_pmc.h"
    #include "device/MK12D5/MK12D5_port.h"
    #include "device/MK12D5/MK12D5_rcm.h"
    #include "device/MK12D5/MK12D5_rfsys.h"
    #include "device/MK12D5/MK12D5_rfvbat.h"
    #include "device/MK12D5/MK12D5_rtc.h"
    #include "device/MK12D5/MK12D5_sim.h"
    #include "device/MK12D5/MK12D5_smc.h"
    #include "device/MK12D5/MK12D5_spi.h"
    #include "device/MK12D5/MK12D5_uart.h"
    #include "device/MK12D5/MK12D5_vref.h"
    #include "device/MK12D5/MK12D5_wdog.h"

#elif (defined(CPU_MK20DX128VMP5) || defined(CPU_MK20DN128VMP5) || defined(CPU_MK20DX64VMP5) || \
    defined(CPU_MK20DN64VMP5) || defined(CPU_MK20DX32VMP5) || defined(CPU_MK20DN32VMP5) || \
    defined(CPU_MK20DX128VLH5) || defined(CPU_MK20DN128VLH5) || defined(CPU_MK20DX64VLH5) || \
    defined(CPU_MK20DN64VLH5) || defined(CPU_MK20DX32VLH5) || defined(CPU_MK20DN32VLH5) || \
    defined(CPU_MK20DX128VFM5) || defined(CPU_MK20DN128VFM5) || defined(CPU_MK20DX64VFM5) || \
    defined(CPU_MK20DN64VFM5) || defined(CPU_MK20DX32VFM5) || defined(CPU_MK20DN32VFM5) || \
    defined(CPU_MK20DX128VFT5) || defined(CPU_MK20DN128VFT5) || defined(CPU_MK20DX64VFT5) || \
    defined(CPU_MK20DN64VFT5) || defined(CPU_MK20DX32VFT5) || defined(CPU_MK20DN32VFT5) || \
    defined(CPU_MK20DX128VLF5) || defined(CPU_MK20DN128VLF5) || defined(CPU_MK20DX64VLF5) || \
    defined(CPU_MK20DN64VLF5) || defined(CPU_MK20DX32VLF5) || defined(CPU_MK20DN32VLF5))

    #define MK20D5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK20D5/MK20D5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK20D5/MK20D5_adc.h"
    #include "device/MK20D5/MK20D5_cmp.h"
    #include "device/MK20D5/MK20D5_cmt.h"
    #include "device/MK20D5/MK20D5_crc.h"
    #include "device/MK20D5/MK20D5_dma.h"
    #include "device/MK20D5/MK20D5_dmamux.h"
    #include "device/MK20D5/MK20D5_ewm.h"
    #include "device/MK20D5/MK20D5_fmc.h"
    #include "device/MK20D5/MK20D5_ftfl.h"
    #include "device/MK20D5/MK20D5_ftm.h"
    #include "device/MK20D5/MK20D5_gpio.h"
    #include "device/MK20D5/MK20D5_i2c.h"
    #include "device/MK20D5/MK20D5_i2s.h"
    #include "device/MK20D5/MK20D5_llwu.h"
    #include "device/MK20D5/MK20D5_lptmr.h"
    #include "device/MK20D5/MK20D5_mcg.h"
    #include "device/MK20D5/MK20D5_nv.h"
    #include "device/MK20D5/MK20D5_osc.h"
    #include "device/MK20D5/MK20D5_pdb.h"
    #include "device/MK20D5/MK20D5_pit.h"
    #include "device/MK20D5/MK20D5_pmc.h"
    #include "device/MK20D5/MK20D5_port.h"
    #include "device/MK20D5/MK20D5_rcm.h"
    #include "device/MK20D5/MK20D5_rfsys.h"
    #include "device/MK20D5/MK20D5_rfvbat.h"
    #include "device/MK20D5/MK20D5_rtc.h"
    #include "device/MK20D5/MK20D5_sim.h"
    #include "device/MK20D5/MK20D5_smc.h"
    #include "device/MK20D5/MK20D5_spi.h"
    #include "device/MK20D5/MK20D5_tsi.h"
    #include "device/MK20D5/MK20D5_uart.h"
    #include "device/MK20D5/MK20D5_usb.h"
    #include "device/MK20D5/MK20D5_usbdcd.h"
    #include "device/MK20D5/MK20D5_vref.h"
    #include "device/MK20D5/MK20D5_wdog.h"

#elif (defined(CPU_MK21DX128AVLK5WS) || defined(CPU_MK21DX256AVLK5WS) || defined(CPU_MK21DN512AVLK5WS) || \
    defined(CPU_MK21DX128AVMC5WS) || defined(CPU_MK21DX256AVMC5WS) || defined(CPU_MK21DN512AVMC5WS))

    #define MK21DA5WS_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK21DA5WS/MK21DA5WS.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK21DA5WS/MK21DA5WS_adc.h"
    #include "device/MK21DA5WS/MK21DA5WS_cau.h"
    #include "device/MK21DA5WS/MK21DA5WS_cmp.h"
    #include "device/MK21DA5WS/MK21DA5WS_cmt.h"
    #include "device/MK21DA5WS/MK21DA5WS_crc.h"
    #include "device/MK21DA5WS/MK21DA5WS_dac.h"
    #include "device/MK21DA5WS/MK21DA5WS_dma.h"
    #include "device/MK21DA5WS/MK21DA5WS_dmamux.h"
    #include "device/MK21DA5WS/MK21DA5WS_dry.h"
    #include "device/MK21DA5WS/MK21DA5WS_ewm.h"
    #include "device/MK21DA5WS/MK21DA5WS_fmc.h"
    #include "device/MK21DA5WS/MK21DA5WS_ftfl.h"
    #include "device/MK21DA5WS/MK21DA5WS_ftm.h"
    #include "device/MK21DA5WS/MK21DA5WS_gpio.h"
    #include "device/MK21DA5WS/MK21DA5WS_i2c.h"
    #include "device/MK21DA5WS/MK21DA5WS_i2s.h"
    #include "device/MK21DA5WS/MK21DA5WS_llwu.h"
    #include "device/MK21DA5WS/MK21DA5WS_lptmr.h"
    #include "device/MK21DA5WS/MK21DA5WS_mcg.h"
    #include "device/MK21DA5WS/MK21DA5WS_mcm.h"
    #include "device/MK21DA5WS/MK21DA5WS_nv.h"
    #include "device/MK21DA5WS/MK21DA5WS_osc.h"
    #include "device/MK21DA5WS/MK21DA5WS_pdb.h"
    #include "device/MK21DA5WS/MK21DA5WS_pit.h"
    #include "device/MK21DA5WS/MK21DA5WS_pmc.h"
    #include "device/MK21DA5WS/MK21DA5WS_port.h"
    #include "device/MK21DA5WS/MK21DA5WS_rcm.h"
    #include "device/MK21DA5WS/MK21DA5WS_rfsys.h"
    #include "device/MK21DA5WS/MK21DA5WS_rfvbat.h"
    #include "device/MK21DA5WS/MK21DA5WS_rng.h"
    #include "device/MK21DA5WS/MK21DA5WS_rtc.h"
    #include "device/MK21DA5WS/MK21DA5WS_sim.h"
    #include "device/MK21DA5WS/MK21DA5WS_smc.h"
    #include "device/MK21DA5WS/MK21DA5WS_spi.h"
    #include "device/MK21DA5WS/MK21DA5WS_uart.h"
    #include "device/MK21DA5WS/MK21DA5WS_usb.h"
    #include "device/MK21DA5WS/MK21DA5WS_usbdcd.h"
    #include "device/MK21DA5WS/MK21DA5WS_vref.h"
    #include "device/MK21DA5WS/MK21DA5WS_wdog.h"

#elif (defined(CPU_MK21DX128AVLK5) || defined(CPU_MK21DX256AVLK5) || defined(CPU_MK21DN512AVLK5) || \
    defined(CPU_MK21DX128AVMC5) || defined(CPU_MK21DX256AVMC5) || defined(CPU_MK21DN512AVMC5))

    #define MK21DA5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK21DA5/MK21DA5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK21DA5/MK21DA5_adc.h"
    #include "device/MK21DA5/MK21DA5_cau.h"
    #include "device/MK21DA5/MK21DA5_cmp.h"
    #include "device/MK21DA5/MK21DA5_cmt.h"
    #include "device/MK21DA5/MK21DA5_crc.h"
    #include "device/MK21DA5/MK21DA5_dac.h"
    #include "device/MK21DA5/MK21DA5_dma.h"
    #include "device/MK21DA5/MK21DA5_dmamux.h"
    #include "device/MK21DA5/MK21DA5_ewm.h"
    #include "device/MK21DA5/MK21DA5_fmc.h"
    #include "device/MK21DA5/MK21DA5_ftfl.h"
    #include "device/MK21DA5/MK21DA5_ftm.h"
    #include "device/MK21DA5/MK21DA5_gpio.h"
    #include "device/MK21DA5/MK21DA5_i2c.h"
    #include "device/MK21DA5/MK21DA5_i2s.h"
    #include "device/MK21DA5/MK21DA5_llwu.h"
    #include "device/MK21DA5/MK21DA5_lptmr.h"
    #include "device/MK21DA5/MK21DA5_mcg.h"
    #include "device/MK21DA5/MK21DA5_mcm.h"
    #include "device/MK21DA5/MK21DA5_nv.h"
    #include "device/MK21DA5/MK21DA5_osc.h"
    #include "device/MK21DA5/MK21DA5_pdb.h"
    #include "device/MK21DA5/MK21DA5_pit.h"
    #include "device/MK21DA5/MK21DA5_pmc.h"
    #include "device/MK21DA5/MK21DA5_port.h"
    #include "device/MK21DA5/MK21DA5_rcm.h"
    #include "device/MK21DA5/MK21DA5_rfsys.h"
    #include "device/MK21DA5/MK21DA5_rfvbat.h"
    #include "device/MK21DA5/MK21DA5_rng.h"
    #include "device/MK21DA5/MK21DA5_rtc.h"
    #include "device/MK21DA5/MK21DA5_sim.h"
    #include "device/MK21DA5/MK21DA5_smc.h"
    #include "device/MK21DA5/MK21DA5_spi.h"
    #include "device/MK21DA5/MK21DA5_uart.h"
    #include "device/MK21DA5/MK21DA5_usb.h"
    #include "device/MK21DA5/MK21DA5_usbdcd.h"
    #include "device/MK21DA5/MK21DA5_vref.h"
    #include "device/MK21DA5/MK21DA5_wdog.h"

#elif (defined(CPU_MK21DX128VLK5WS) || defined(CPU_MK21DX256VLK5WS) || defined(CPU_MK21DN512VLK5WS) || \
    defined(CPU_MK21DX128VMC5WS) || defined(CPU_MK21DX256VMC5WS) || defined(CPU_MK21DN512VMC5WS))

    #define MK21D5WS_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK21D5WS/MK21D5WS.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK21D5WS/MK21D5WS_adc.h"
    #include "device/MK21D5WS/MK21D5WS_cau.h"
    #include "device/MK21D5WS/MK21D5WS_cmp.h"
    #include "device/MK21D5WS/MK21D5WS_cmt.h"
    #include "device/MK21D5WS/MK21D5WS_crc.h"
    #include "device/MK21D5WS/MK21D5WS_dac.h"
    #include "device/MK21D5WS/MK21D5WS_dma.h"
    #include "device/MK21D5WS/MK21D5WS_dmamux.h"
    #include "device/MK21D5WS/MK21D5WS_dry.h"
    #include "device/MK21D5WS/MK21D5WS_ewm.h"
    #include "device/MK21D5WS/MK21D5WS_fmc.h"
    #include "device/MK21D5WS/MK21D5WS_ftfl.h"
    #include "device/MK21D5WS/MK21D5WS_ftm.h"
    #include "device/MK21D5WS/MK21D5WS_gpio.h"
    #include "device/MK21D5WS/MK21D5WS_i2c.h"
    #include "device/MK21D5WS/MK21D5WS_i2s.h"
    #include "device/MK21D5WS/MK21D5WS_llwu.h"
    #include "device/MK21D5WS/MK21D5WS_lptmr.h"
    #include "device/MK21D5WS/MK21D5WS_mcg.h"
    #include "device/MK21D5WS/MK21D5WS_mcm.h"
    #include "device/MK21D5WS/MK21D5WS_nv.h"
    #include "device/MK21D5WS/MK21D5WS_osc.h"
    #include "device/MK21D5WS/MK21D5WS_pdb.h"
    #include "device/MK21D5WS/MK21D5WS_pit.h"
    #include "device/MK21D5WS/MK21D5WS_pmc.h"
    #include "device/MK21D5WS/MK21D5WS_port.h"
    #include "device/MK21D5WS/MK21D5WS_rcm.h"
    #include "device/MK21D5WS/MK21D5WS_rfsys.h"
    #include "device/MK21D5WS/MK21D5WS_rfvbat.h"
    #include "device/MK21D5WS/MK21D5WS_rng.h"
    #include "device/MK21D5WS/MK21D5WS_rtc.h"
    #include "device/MK21D5WS/MK21D5WS_sim.h"
    #include "device/MK21D5WS/MK21D5WS_smc.h"
    #include "device/MK21D5WS/MK21D5WS_spi.h"
    #include "device/MK21D5WS/MK21D5WS_uart.h"
    #include "device/MK21D5WS/MK21D5WS_usb.h"
    #include "device/MK21D5WS/MK21D5WS_usbdcd.h"
    #include "device/MK21D5WS/MK21D5WS_vref.h"
    #include "device/MK21D5WS/MK21D5WS_wdog.h"

#elif (defined(CPU_MK21DX128VLK5) || defined(CPU_MK21DX256VLK5) || defined(CPU_MK21DN512VLK5) || \
    defined(CPU_MK21DX128VMC5) || defined(CPU_MK21DX256VMC5) || defined(CPU_MK21DN512VMC5))

    #define MK21D5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK21D5/MK21D5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK21D5/MK21D5_adc.h"
    #include "device/MK21D5/MK21D5_cau.h"
    #include "device/MK21D5/MK21D5_cmp.h"
    #include "device/MK21D5/MK21D5_cmt.h"
    #include "device/MK21D5/MK21D5_crc.h"
    #include "device/MK21D5/MK21D5_dac.h"
    #include "device/MK21D5/MK21D5_dma.h"
    #include "device/MK21D5/MK21D5_dmamux.h"
    #include "device/MK21D5/MK21D5_ewm.h"
    #include "device/MK21D5/MK21D5_fmc.h"
    #include "device/MK21D5/MK21D5_ftfl.h"
    #include "device/MK21D5/MK21D5_ftm.h"
    #include "device/MK21D5/MK21D5_gpio.h"
    #include "device/MK21D5/MK21D5_i2c.h"
    #include "device/MK21D5/MK21D5_i2s.h"
    #include "device/MK21D5/MK21D5_llwu.h"
    #include "device/MK21D5/MK21D5_lptmr.h"
    #include "device/MK21D5/MK21D5_mcg.h"
    #include "device/MK21D5/MK21D5_mcm.h"
    #include "device/MK21D5/MK21D5_nv.h"
    #include "device/MK21D5/MK21D5_osc.h"
    #include "device/MK21D5/MK21D5_pdb.h"
    #include "device/MK21D5/MK21D5_pit.h"
    #include "device/MK21D5/MK21D5_pmc.h"
    #include "device/MK21D5/MK21D5_port.h"
    #include "device/MK21D5/MK21D5_rcm.h"
    #include "device/MK21D5/MK21D5_rfsys.h"
    #include "device/MK21D5/MK21D5_rfvbat.h"
    #include "device/MK21D5/MK21D5_rng.h"
    #include "device/MK21D5/MK21D5_rtc.h"
    #include "device/MK21D5/MK21D5_sim.h"
    #include "device/MK21D5/MK21D5_smc.h"
    #include "device/MK21D5/MK21D5_spi.h"
    #include "device/MK21D5/MK21D5_uart.h"
    #include "device/MK21D5/MK21D5_usb.h"
    #include "device/MK21D5/MK21D5_usbdcd.h"
    #include "device/MK21D5/MK21D5_vref.h"
    #include "device/MK21D5/MK21D5_wdog.h"

#elif (defined(CPU_MK22DX128VLH5) || defined(CPU_MK22DX256VLH5) || defined(CPU_MK22DN512VLH5) || \
    defined(CPU_MK22DX128VLK5) || defined(CPU_MK22DX256VLK5) || defined(CPU_MK22DN512VLK5) || \
    defined(CPU_MK22DX128VMC5) || defined(CPU_MK22DX256VMC5) || defined(CPU_MK22DN512VMC5) || \
    defined(CPU_MK22DX128VLF5) || defined(CPU_MK22DX256VLF5))

    #define MK22D5_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK22D5/MK22D5.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK22D5/MK22D5_adc.h"
    #include "device/MK22D5/MK22D5_cmp.h"
    #include "device/MK22D5/MK22D5_cmt.h"
    #include "device/MK22D5/MK22D5_crc.h"
    #include "device/MK22D5/MK22D5_dac.h"
    #include "device/MK22D5/MK22D5_dma.h"
    #include "device/MK22D5/MK22D5_dmamux.h"
    #include "device/MK22D5/MK22D5_ewm.h"
    #include "device/MK22D5/MK22D5_fmc.h"
    #include "device/MK22D5/MK22D5_ftfl.h"
    #include "device/MK22D5/MK22D5_ftm.h"
    #include "device/MK22D5/MK22D5_gpio.h"
    #include "device/MK22D5/MK22D5_i2c.h"
    #include "device/MK22D5/MK22D5_i2s.h"
    #include "device/MK22D5/MK22D5_llwu.h"
    #include "device/MK22D5/MK22D5_lptmr.h"
    #include "device/MK22D5/MK22D5_mcg.h"
    #include "device/MK22D5/MK22D5_mcm.h"
    #include "device/MK22D5/MK22D5_nv.h"
    #include "device/MK22D5/MK22D5_osc.h"
    #include "device/MK22D5/MK22D5_pdb.h"
    #include "device/MK22D5/MK22D5_pit.h"
    #include "device/MK22D5/MK22D5_pmc.h"
    #include "device/MK22D5/MK22D5_port.h"
    #include "device/MK22D5/MK22D5_rcm.h"
    #include "device/MK22D5/MK22D5_rfsys.h"
    #include "device/MK22D5/MK22D5_rfvbat.h"
    #include "device/MK22D5/MK22D5_rtc.h"
    #include "device/MK22D5/MK22D5_sim.h"
    #include "device/MK22D5/MK22D5_smc.h"
    #include "device/MK22D5/MK22D5_spi.h"
    #include "device/MK22D5/MK22D5_uart.h"
    #include "device/MK22D5/MK22D5_usb.h"
    #include "device/MK22D5/MK22D5_usbdcd.h"
    #include "device/MK22D5/MK22D5_vref.h"
    #include "device/MK22D5/MK22D5_wdog.h"

#elif (defined(CPU_MK22FN128VDC10) || defined(CPU_MK22FN128VLH10) || defined(CPU_MK22FN128VLL10) || \
    defined(CPU_MK22FN128VMP10))

    #define MK22F12810_SERIES

    #define K22F12810_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK22F12810/MK22F12810.h"
    /* Extension register definitions */
    #include "device/MK22F12810/MK22F12810_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK22F12810/MK22F12810_features.h"

#elif (defined(CPU_MK22FN256VDC12) || defined(CPU_MK22FN256VLH12) || defined(CPU_MK22FN256VLL12) || \
    defined(CPU_MK22FN256VMP12))

    #define MK22F25612_SERIES

    #define K22F25612_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK22F25612/MK22F25612.h"
    /* Extension register definitions */
    #include "device/MK22F25612/MK22F25612_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK22F25612/MK22F25612_features.h"


#elif (defined(CPU_MK22FN512VDC12) || defined(CPU_MK22FN512VLH12) || defined(CPU_MK22FN512VLL12))

    #define MK22F51212_SERIES

    #define K22F51212_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK22F51212/MK22F51212.h"
    /* Extension register definitions */
    #include "device/MK22F51212/MK22F51212_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK22F51212/MK22F51212_features.h"

#elif (defined(CPU_MK24FN1M0VDC12) || defined(CPU_MK24FN1M0VLL12) || defined(CPU_MK24FN1M0VLQ12))

    #define MK24F12_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK24F12/MK24F12.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK24F12/MK24F12_adc.h"
    #include "device/MK24F12/MK24F12_aips.h"
    #include "device/MK24F12/MK24F12_axbs.h"
    #include "device/MK24F12/MK24F12_can.h"
    #include "device/MK24F12/MK24F12_cau.h"
    #include "device/MK24F12/MK24F12_cmp.h"
    #include "device/MK24F12/MK24F12_cmt.h"
    #include "device/MK24F12/MK24F12_crc.h"
    #include "device/MK24F12/MK24F12_dac.h"
    #include "device/MK24F12/MK24F12_dma.h"
    #include "device/MK24F12/MK24F12_dmamux.h"
    #include "device/MK24F12/MK24F12_ewm.h"
    #include "device/MK24F12/MK24F12_fb.h"
    #include "device/MK24F12/MK24F12_fmc.h"
    #include "device/MK24F12/MK24F12_ftfe.h"
    #include "device/MK24F12/MK24F12_ftm.h"
    #include "device/MK24F12/MK24F12_gpio.h"
    #include "device/MK24F12/MK24F12_i2c.h"
    #include "device/MK24F12/MK24F12_i2s.h"
    #include "device/MK24F12/MK24F12_llwu.h"
    #include "device/MK24F12/MK24F12_lptmr.h"
    #include "device/MK24F12/MK24F12_mcg.h"
    #include "device/MK24F12/MK24F12_mcm.h"
    #include "device/MK24F12/MK24F12_mpu.h"
    #include "device/MK24F12/MK24F12_nv.h"
    #include "device/MK24F12/MK24F12_osc.h"
    #include "device/MK24F12/MK24F12_pdb.h"
    #include "device/MK24F12/MK24F12_pit.h"
    #include "device/MK24F12/MK24F12_pmc.h"
    #include "device/MK24F12/MK24F12_port.h"
    #include "device/MK24F12/MK24F12_rcm.h"
    #include "device/MK24F12/MK24F12_rfsys.h"
    #include "device/MK24F12/MK24F12_rfvbat.h"
    #include "device/MK24F12/MK24F12_rng.h"
    #include "device/MK24F12/MK24F12_rtc.h"
    #include "device/MK24F12/MK24F12_sdhc.h"
    #include "device/MK24F12/MK24F12_sim.h"
    #include "device/MK24F12/MK24F12_smc.h"
    #include "device/MK24F12/MK24F12_spi.h"
    #include "device/MK24F12/MK24F12_uart.h"
    #include "device/MK24F12/MK24F12_usb.h"
    #include "device/MK24F12/MK24F12_usbdcd.h"
    #include "device/MK24F12/MK24F12_vref.h"
    #include "device/MK24F12/MK24F12_wdog.h"

#elif (defined(CPU_MK24FN256VDC12))

    #define MK24F25612_SERIES

    #define K24F25612_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK24F25612/MK24F25612.h"
    /* Extension register definitions */
    #include "device/MK24F25612/MK24F25612_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK24F25612/MK24F25612_features.h"


#elif (defined(CPU_MK63FN1M0VLQ12) || defined(CPU_MK63FN1M0VMD12))

    #define MK63F12_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK63F12/MK63F12.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK63F12/MK63F12_adc.h"
    #include "device/MK63F12/MK63F12_aips.h"
    #include "device/MK63F12/MK63F12_axbs.h"
    #include "device/MK63F12/MK63F12_can.h"
    #include "device/MK63F12/MK63F12_cau.h"
    #include "device/MK63F12/MK63F12_cmp.h"
    #include "device/MK63F12/MK63F12_cmt.h"
    #include "device/MK63F12/MK63F12_crc.h"
    #include "device/MK63F12/MK63F12_dac.h"
    #include "device/MK63F12/MK63F12_dma.h"
    #include "device/MK63F12/MK63F12_dmamux.h"
    #include "device/MK63F12/MK63F12_enet.h"
    #include "device/MK63F12/MK63F12_ewm.h"
    #include "device/MK63F12/MK63F12_fb.h"
    #include "device/MK63F12/MK63F12_fmc.h"
    #include "device/MK63F12/MK63F12_ftfe.h"
    #include "device/MK63F12/MK63F12_ftm.h"
    #include "device/MK63F12/MK63F12_gpio.h"
    #include "device/MK63F12/MK63F12_i2c.h"
    #include "device/MK63F12/MK63F12_i2s.h"
    #include "device/MK63F12/MK63F12_llwu.h"
    #include "device/MK63F12/MK63F12_lptmr.h"
    #include "device/MK63F12/MK63F12_mcg.h"
    #include "device/MK63F12/MK63F12_mcm.h"
    #include "device/MK63F12/MK63F12_mpu.h"
    #include "device/MK63F12/MK63F12_nv.h"
    #include "device/MK63F12/MK63F12_osc.h"
    #include "device/MK63F12/MK63F12_pdb.h"
    #include "device/MK63F12/MK63F12_pit.h"
    #include "device/MK63F12/MK63F12_pmc.h"
    #include "device/MK63F12/MK63F12_port.h"
    #include "device/MK63F12/MK63F12_rcm.h"
    #include "device/MK63F12/MK63F12_rfsys.h"
    #include "device/MK63F12/MK63F12_rfvbat.h"
    #include "device/MK63F12/MK63F12_rng.h"
    #include "device/MK63F12/MK63F12_rtc.h"
    #include "device/MK63F12/MK63F12_sdhc.h"
    #include "device/MK63F12/MK63F12_sim.h"
    #include "device/MK63F12/MK63F12_smc.h"
    #include "device/MK63F12/MK63F12_spi.h"
    #include "device/MK63F12/MK63F12_uart.h"
    #include "device/MK63F12/MK63F12_usb.h"
    #include "device/MK63F12/MK63F12_usbdcd.h"
    #include "device/MK63F12/MK63F12_vref.h"
    #include "device/MK63F12/MK63F12_wdog.h"

#elif (defined(CPU_MK64FX512VDC12) || defined(CPU_MK64FN1M0VDC12) || defined(CPU_MK64FX512VLL12) || \
    defined(CPU_MK64FN1M0VLL12) || defined(CPU_MK64FX512VLQ12) || defined(CPU_MK64FN1M0VLQ12) || \
    defined(CPU_MK64FX512VMD12) || defined(CPU_MK64FN1M0VMD12))

    #define MK64F12_SERIES

    #define K64F12_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK64F12/MK64F12.h"
    /* Extension register definitions */
    #include "device/MK64F12/MK64F12_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK64F12/MK64F12_features.h"

#elif (defined(CPU_MK80FN256VLQR))

    #define MK80F256_SERIES

    #define K80F256_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK81F25615/MK81F25615.h"
    /* Extension register definitions */
    #include "device/MK81F25615/MK81F25615_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK81F25615/MK81F25615_features.h"

    // Temporary - for SIM SECKEY registers
    #include "device/MK81F25615/MK80F256_sim_seckey.h"

#elif (defined(CPU_MK65FN2M0CAC18) || defined(CPU_MK65FX1M0CAC18) || defined(CPU_MK65FN2M0VMI18) || \
    defined(CPU_MK65FX1M0VMI18))

    #define MK65F18_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK65F18/MK65F18.h"
    /* Extension register definitions */
    #include "device/MK65F18/MK65F18_extension.h"
    /* CPU specific feature definitions */
    #include "device/MK65F18/MK65F18_features.h"

    // Temporary - to fix interrupt vector numbers
    #include "device/MK65F18/MK65F18_INV.h"

#elif (defined(CPU_MK66FN2M0VLQ18) || defined(CPU_MK66FX1M0VLQ18) || defined(CPU_MK66FN2M0VMD18) || \
    defined(CPU_MK66FX1M0VMD18))

    #define MK66F18_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK66F18/MK66F18.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK66F18/MK66F18_adc.h"
    #include "device/MK66F18/MK66F18_aips.h"
    #include "device/MK66F18/MK66F18_axbs.h"
    #include "device/MK66F18/MK66F18_can.h"
    #include "device/MK66F18/MK66F18_cau.h"
    #include "device/MK66F18/MK66F18_cmp.h"
    #include "device/MK66F18/MK66F18_cmt.h"
    #include "device/MK66F18/MK66F18_crc.h"
    #include "device/MK66F18/MK66F18_dac.h"
    #include "device/MK66F18/MK66F18_dma.h"
    #include "device/MK66F18/MK66F18_dmamux.h"
    #include "device/MK66F18/MK66F18_enet.h"
    #include "device/MK66F18/MK66F18_ewm.h"
    #include "device/MK66F18/MK66F18_fb.h"
    #include "device/MK66F18/MK66F18_fmc.h"
    #include "device/MK66F18/MK66F18_ftfe.h"
    #include "device/MK66F18/MK66F18_ftm.h"
    #include "device/MK66F18/MK66F18_gpio.h"
    #include "device/MK66F18/MK66F18_i2c.h"
    #include "device/MK66F18/MK66F18_i2s.h"
    #include "device/MK66F18/MK66F18_llwu.h"
    #include "device/MK66F18/MK66F18_lmem.h"
    #include "device/MK66F18/MK66F18_lptmr.h"
    #include "device/MK66F18/MK66F18_lpuart.h"
    #include "device/MK66F18/MK66F18_mcg.h"
    #include "device/MK66F18/MK66F18_mcm.h"
    #include "device/MK66F18/MK66F18_mpu.h"
    #include "device/MK66F18/MK66F18_nv.h"
    #include "device/MK66F18/MK66F18_osc.h"
    #include "device/MK66F18/MK66F18_pdb.h"
    #include "device/MK66F18/MK66F18_pit.h"
    #include "device/MK66F18/MK66F18_pmc.h"
    #include "device/MK66F18/MK66F18_port.h"
    #include "device/MK66F18/MK66F18_rcm.h"
    #include "device/MK66F18/MK66F18_rfsys.h"
    #include "device/MK66F18/MK66F18_rfvbat.h"
    #include "device/MK66F18/MK66F18_rng.h"
    #include "device/MK66F18/MK66F18_rtc.h"
    #include "device/MK66F18/MK66F18_sdhc.h"
    #include "device/MK66F18/MK66F18_sdram.h"
    #include "device/MK66F18/MK66F18_sim.h"
    #include "device/MK66F18/MK66F18_smc.h"
    #include "device/MK66F18/MK66F18_spi.h"
    #include "device/MK66F18/MK66F18_tpm.h"
    #include "device/MK66F18/MK66F18_tsi.h"
    #include "device/MK66F18/MK66F18_uart.h"
    #include "device/MK66F18/MK66F18_usb.h"
    #include "device/MK66F18/MK66F18_usbdcd.h"
    #include "device/MK66F18/MK66F18_usbhs.h"
    #include "device/MK66F18/MK66F18_usbhsdcd.h"
    #include "device/MK66F18/MK66F18_usbphy.h"
    #include "device/MK66F18/MK66F18_vref.h"
    #include "device/MK66F18/MK66F18_wdog.h"

#elif (defined(CPU_MK70FN1M0VMF12) || defined(CPU_MK70FX512VMF12) || defined(CPU_MK70FN1M0VMF15) || \
    defined(CPU_MK70FX512VMF15) || defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FX512VMJ12) || \
    defined(CPU_MK70FN1M0VMJ15) || defined(CPU_MK70FX512VMJ15))

    #define MK70F12_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK70F12/MK70F12.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK70F12/MK70F12_adc.h"
    #include "device/MK70F12/MK70F12_aips.h"
    #include "device/MK70F12/MK70F12_axbs.h"
    #include "device/MK70F12/MK70F12_can.h"
    #include "device/MK70F12/MK70F12_cau.h"
    #include "device/MK70F12/MK70F12_cmp.h"
    #include "device/MK70F12/MK70F12_cmt.h"
    #include "device/MK70F12/MK70F12_crc.h"
    #include "device/MK70F12/MK70F12_dac.h"
    #include "device/MK70F12/MK70F12_ddr.h"
    #include "device/MK70F12/MK70F12_dma.h"
    #include "device/MK70F12/MK70F12_dmamux.h"
    #include "device/MK70F12/MK70F12_enet.h"
    #include "device/MK70F12/MK70F12_ewm.h"
    #include "device/MK70F12/MK70F12_fb.h"
    #include "device/MK70F12/MK70F12_fmc.h"
    #include "device/MK70F12/MK70F12_ftfe.h"
    #include "device/MK70F12/MK70F12_ftm.h"
    #include "device/MK70F12/MK70F12_gpio.h"
    #include "device/MK70F12/MK70F12_i2c.h"
    #include "device/MK70F12/MK70F12_i2s.h"
    #include "device/MK70F12/MK70F12_lcdc.h"
    #include "device/MK70F12/MK70F12_llwu.h"
    #include "device/MK70F12/MK70F12_lmem.h"
    #include "device/MK70F12/MK70F12_lptmr.h"
    #include "device/MK70F12/MK70F12_mcg.h"
    #include "device/MK70F12/MK70F12_mcm.h"
    #include "device/MK70F12/MK70F12_mpu.h"
    #include "device/MK70F12/MK70F12_nfc.h"
    #include "device/MK70F12/MK70F12_nv.h"
    #include "device/MK70F12/MK70F12_osc.h"
    #include "device/MK70F12/MK70F12_pdb.h"
    #include "device/MK70F12/MK70F12_pit.h"
    #include "device/MK70F12/MK70F12_pmc.h"
    #include "device/MK70F12/MK70F12_port.h"
    #include "device/MK70F12/MK70F12_rcm.h"
    #include "device/MK70F12/MK70F12_rfsys.h"
    #include "device/MK70F12/MK70F12_rfvbat.h"
    #include "device/MK70F12/MK70F12_rng.h"
    #include "device/MK70F12/MK70F12_rtc.h"
    #include "device/MK70F12/MK70F12_sdhc.h"
    #include "device/MK70F12/MK70F12_sim.h"
    #include "device/MK70F12/MK70F12_smc.h"
    #include "device/MK70F12/MK70F12_spi.h"
    #include "device/MK70F12/MK70F12_tsi.h"
    #include "device/MK70F12/MK70F12_uart.h"
    #include "device/MK70F12/MK70F12_usb.h"
    #include "device/MK70F12/MK70F12_usbdcd.h"
    #include "device/MK70F12/MK70F12_usbhs.h"
    #include "device/MK70F12/MK70F12_vref.h"
    #include "device/MK70F12/MK70F12_wdog.h"

#elif (defined(CPU_MK70FN1M0VMF12) || defined(CPU_MK70FX512VMF12) || defined(CPU_MK70FN1M0VMF15) || \
    defined(CPU_MK70FX512VMF15) || defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FX512VMJ12) || \
    defined(CPU_MK70FN1M0VMJ15) || defined(CPU_MK70FX512VMJ15))

    #define MK70F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK70F15/MK70F15.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MK70F15/MK70F15_adc.h"
    #include "device/MK70F15/MK70F15_aips.h"
    #include "device/MK70F15/MK70F15_axbs.h"
    #include "device/MK70F15/MK70F15_can.h"
    #include "device/MK70F15/MK70F15_cau.h"
    #include "device/MK70F15/MK70F15_cmp.h"
    #include "device/MK70F15/MK70F15_cmt.h"
    #include "device/MK70F15/MK70F15_crc.h"
    #include "device/MK70F15/MK70F15_dac.h"
    #include "device/MK70F15/MK70F15_ddr.h"
    #include "device/MK70F15/MK70F15_dma.h"
    #include "device/MK70F15/MK70F15_dmamux.h"
    #include "device/MK70F15/MK70F15_enet.h"
    #include "device/MK70F15/MK70F15_ewm.h"
    #include "device/MK70F15/MK70F15_fb.h"
    #include "device/MK70F15/MK70F15_fmc.h"
    #include "device/MK70F15/MK70F15_ftfe.h"
    #include "device/MK70F15/MK70F15_ftm.h"
    #include "device/MK70F15/MK70F15_gpio.h"
    #include "device/MK70F15/MK70F15_i2c.h"
    #include "device/MK70F15/MK70F15_i2s.h"
    #include "device/MK70F15/MK70F15_lcdc.h"
    #include "device/MK70F15/MK70F15_llwu.h"
    #include "device/MK70F15/MK70F15_lmem.h"
    #include "device/MK70F15/MK70F15_lptmr.h"
    #include "device/MK70F15/MK70F15_mcg.h"
    #include "device/MK70F15/MK70F15_mcm.h"
    #include "device/MK70F15/MK70F15_mpu.h"
    #include "device/MK70F15/MK70F15_nfc.h"
    #include "device/MK70F15/MK70F15_nv.h"
    #include "device/MK70F15/MK70F15_osc.h"
    #include "device/MK70F15/MK70F15_pdb.h"
    #include "device/MK70F15/MK70F15_pit.h"
    #include "device/MK70F15/MK70F15_pmc.h"
    #include "device/MK70F15/MK70F15_port.h"
    #include "device/MK70F15/MK70F15_rcm.h"
    #include "device/MK70F15/MK70F15_rfsys.h"
    #include "device/MK70F15/MK70F15_rfvbat.h"
    #include "device/MK70F15/MK70F15_rng.h"
    #include "device/MK70F15/MK70F15_rtc.h"
    #include "device/MK70F15/MK70F15_sdhc.h"
    #include "device/MK70F15/MK70F15_sim.h"
    #include "device/MK70F15/MK70F15_smc.h"
    #include "device/MK70F15/MK70F15_spi.h"
    #include "device/MK70F15/MK70F15_tsi.h"
    #include "device/MK70F15/MK70F15_uart.h"
    #include "device/MK70F15/MK70F15_usb.h"
    #include "device/MK70F15/MK70F15_usbdcd.h"
    #include "device/MK70F15/MK70F15_usbhs.h"
    #include "device/MK70F15/MK70F15_vref.h"
    #include "device/MK70F15/MK70F15_wdog.h"

#elif (defined(CPU_MKL02Z32CAF4) || defined(CPU_MKL02Z8VFG4) || defined(CPU_MKL02Z16VFG4) || \
    defined(CPU_MKL02Z32VFG4) || defined(CPU_MKL02Z16VFK4) || defined(CPU_MKL02Z32VFK4) || \
    defined(CPU_MKL02Z16VFM4) || defined(CPU_MKL02Z32VFM4))

    #define MKL02Z4_SERIES

    #define KL02Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL02Z4/MKL02Z4.h"
    /* Extension register definitions */
    #include "device/MKL02Z4/MKL02Z4_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL02Z4/MKL02Z4_features.h"

#elif (defined(CPU_MKL03Z32CAF4) || defined(CPU_MKL03Z8VFG4) || defined(CPU_MKL03Z16VFG4) || \
    defined(CPU_MKL03Z32VFG4) || defined(CPU_MKL03Z8VFK4) || defined(CPU_MKL03Z16VFK4) || \
    defined(CPU_MKL03Z32VFK4))

    #define KL03Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL03Z4/MKL03Z4.h"
    /* Extension register definitions */
    #include "device/MKL03Z4/MKL03Z4_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL03Z4/MKL03Z4_features.h"

#elif (defined(CPU_MKL05Z8VFK4) || defined(CPU_MKL05Z16VFK4) || defined(CPU_MKL05Z32VFK4) || \
    defined(CPU_MKL05Z8VLC4) || defined(CPU_MKL05Z16VLC4) || defined(CPU_MKL05Z32VLC4) || \
    defined(CPU_MKL05Z8VFM4) || defined(CPU_MKL05Z16VFM4) || defined(CPU_MKL05Z32VFM4) || \
    defined(CPU_MKL05Z16VLF4) || defined(CPU_MKL05Z32VLF4))

    #define MKL05Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL05Z4/MKL05Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL05Z4/MKL05Z4_adc.h"
    #include "device/MKL05Z4/MKL05Z4_cmp.h"
    #include "device/MKL05Z4/MKL05Z4_dac.h"
    #include "device/MKL05Z4/MKL05Z4_dma.h"
    #include "device/MKL05Z4/MKL05Z4_dmamux.h"
    #include "device/MKL05Z4/MKL05Z4_fgpio.h"
    #include "device/MKL05Z4/MKL05Z4_ftfa.h"
    #include "device/MKL05Z4/MKL05Z4_gpio.h"
    #include "device/MKL05Z4/MKL05Z4_i2c.h"
    #include "device/MKL05Z4/MKL05Z4_llwu.h"
    #include "device/MKL05Z4/MKL05Z4_lptmr.h"
    #include "device/MKL05Z4/MKL05Z4_mcg.h"
    #include "device/MKL05Z4/MKL05Z4_mcm.h"
    #include "device/MKL05Z4/MKL05Z4_mtb.h"
    #include "device/MKL05Z4/MKL05Z4_mtbdwt.h"
    #include "device/MKL05Z4/MKL05Z4_nv.h"
    #include "device/MKL05Z4/MKL05Z4_osc.h"
    #include "device/MKL05Z4/MKL05Z4_pit.h"
    #include "device/MKL05Z4/MKL05Z4_pmc.h"
    #include "device/MKL05Z4/MKL05Z4_port.h"
    #include "device/MKL05Z4/MKL05Z4_rcm.h"
    #include "device/MKL05Z4/MKL05Z4_rom.h"
    #include "device/MKL05Z4/MKL05Z4_rtc.h"
    #include "device/MKL05Z4/MKL05Z4_sim.h"
    #include "device/MKL05Z4/MKL05Z4_smc.h"
    #include "device/MKL05Z4/MKL05Z4_spi.h"
    #include "device/MKL05Z4/MKL05Z4_tpm.h"
    #include "device/MKL05Z4/MKL05Z4_tsi.h"
    #include "device/MKL05Z4/MKL05Z4_uart0.h"

#elif (defined(CPU_MKL13Z64VLH4) || defined(CPU_MKL13Z32VLH4))

    #define KL13Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL13Z4/MKL13Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL13Z4/MKL13Z4_crc.h"
    #include "device/MKL13Z4/MKL13Z4_ftfa.h"
    #include "device/MKL13Z4/MKL13Z4_gpio.h"
    #include "device/MKL13Z4/MKL13Z4_i2c.h"
    #include "device/MKL13Z4/MKL13Z4_lpuart.h"
    #include "device/MKL13Z4/MKL13Z4_mcg.h"
    #include "device/MKL13Z4/MKL13Z4_mcm.h"
    #include "device/MKL13Z4/MKL13Z4_port.h"
    #include "device/MKL13Z4/MKL13Z4_rcm.h"
    #include "device/MKL13Z4/MKL13Z4_sim.h"
    #include "device/MKL13Z4/MKL13Z4_smc.h"
    #include "device/MKL13Z4/MKL13Z4_spi.h"

#elif (defined(CPU_MKL16Z32VFM4) || defined(CPU_MKL16Z64VFM4) || defined(CPU_MKL16Z128VFM4) || \
    defined(CPU_MKL16Z32VFT4) || defined(CPU_MKL16Z64VFT4) || defined(CPU_MKL16Z128VFT4) || \
    defined(CPU_MKL16Z32VLH4) || defined(CPU_MKL16Z64VLH4) || defined(CPU_MKL16Z128VLH4))

    #define MKL16Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL16Z4/MKL16Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL16Z4/MKL16Z4_adc.h"
    #include "device/MKL16Z4/MKL16Z4_cmp.h"
    #include "device/MKL16Z4/MKL16Z4_dac.h"
    #include "device/MKL16Z4/MKL16Z4_dma.h"
    #include "device/MKL16Z4/MKL16Z4_dmamux.h"
    #include "device/MKL16Z4/MKL16Z4_fgpio.h"
    #include "device/MKL16Z4/MKL16Z4_ftfa.h"
    #include "device/MKL16Z4/MKL16Z4_gpio.h"
    #include "device/MKL16Z4/MKL16Z4_i2c.h"
    #include "device/MKL16Z4/MKL16Z4_i2s.h"
    #include "device/MKL16Z4/MKL16Z4_llwu.h"
    #include "device/MKL16Z4/MKL16Z4_lptmr.h"
    #include "device/MKL16Z4/MKL16Z4_mcg.h"
    #include "device/MKL16Z4/MKL16Z4_mcm.h"
    #include "device/MKL16Z4/MKL16Z4_mtb.h"
    #include "device/MKL16Z4/MKL16Z4_mtbdwt.h"
    #include "device/MKL16Z4/MKL16Z4_nv.h"
    #include "device/MKL16Z4/MKL16Z4_osc.h"
    #include "device/MKL16Z4/MKL16Z4_pit.h"
    #include "device/MKL16Z4/MKL16Z4_pmc.h"
    #include "device/MKL16Z4/MKL16Z4_port.h"
    #include "device/MKL16Z4/MKL16Z4_rcm.h"
    #include "device/MKL16Z4/MKL16Z4_rom.h"
    #include "device/MKL16Z4/MKL16Z4_rtc.h"
    #include "device/MKL16Z4/MKL16Z4_sim.h"
    #include "device/MKL16Z4/MKL16Z4_smc.h"
    #include "device/MKL16Z4/MKL16Z4_spi.h"
    #include "device/MKL16Z4/MKL16Z4_tpm.h"
    #include "device/MKL16Z4/MKL16Z4_tsi.h"
    #include "device/MKL16Z4/MKL16Z4_uart.h"
    #include "device/MKL16Z4/MKL16Z4_uart0.h"

#elif (defined(CPU_MKL17Z128VFM4) || defined(CPU_MKL17Z256VFM4) || defined(CPU_MKL17Z128VFT4) || \
    defined(CPU_MKL17Z256VFT4) || defined(CPU_MKL17Z128VMP4) || defined(CPU_MKL17Z256VMP4))

    #define MKL17Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL17Z4/MKL17Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL17Z4/MKL17Z4_adc.h"
    #include "device/MKL17Z4/MKL17Z4_cmp.h"
    #include "device/MKL17Z4/MKL17Z4_dac.h"
    #include "device/MKL17Z4/MKL17Z4_dma.h"
    #include "device/MKL17Z4/MKL17Z4_dmamux.h"
    #include "device/MKL17Z4/MKL17Z4_flexio.h"
    #include "device/MKL17Z4/MKL17Z4_ftfa.h"
    #include "device/MKL17Z4/MKL17Z4_gpio.h"
    #include "device/MKL17Z4/MKL17Z4_i2c.h"
    #include "device/MKL17Z4/MKL17Z4_i2s.h"
    #include "device/MKL17Z4/MKL17Z4_llwu.h"
    #include "device/MKL17Z4/MKL17Z4_lptmr.h"
    #include "device/MKL17Z4/MKL17Z4_lpuart.h"
    #include "device/MKL17Z4/MKL17Z4_mcg.h"
    #include "device/MKL17Z4/MKL17Z4_mcm.h"
    #include "device/MKL17Z4/MKL17Z4_mtb.h"
    #include "device/MKL17Z4/MKL17Z4_mtbdwt.h"
    #include "device/MKL17Z4/MKL17Z4_nv.h"
    #include "device/MKL17Z4/MKL17Z4_osc.h"
    #include "device/MKL17Z4/MKL17Z4_pit.h"
    #include "device/MKL17Z4/MKL17Z4_pmc.h"
    #include "device/MKL17Z4/MKL17Z4_port.h"
    #include "device/MKL17Z4/MKL17Z4_rcm.h"
    #include "device/MKL17Z4/MKL17Z4_rfsys.h"
    #include "device/MKL17Z4/MKL17Z4_rom.h"
    #include "device/MKL17Z4/MKL17Z4_rtc.h"
    #include "device/MKL17Z4/MKL17Z4_sim.h"
    #include "device/MKL17Z4/MKL17Z4_smc.h"
    #include "device/MKL17Z4/MKL17Z4_spi.h"
    #include "device/MKL17Z4/MKL17Z4_tpm.h"
    #include "device/MKL17Z4/MKL17Z4_uart.h"
    #include "device/MKL17Z4/MKL17Z4_vref.h"

#elif (defined(CPU_MKL25Z32VFM4) || defined(CPU_MKL25Z64VFM4) || defined(CPU_MKL25Z128VFM4) || \
    defined(CPU_MKL25Z32VFT4) || defined(CPU_MKL25Z64VFT4) || defined(CPU_MKL25Z128VFT4) || \
    defined(CPU_MKL25Z32VLH4) || defined(CPU_MKL25Z64VLH4) || defined(CPU_MKL25Z128VLH4) || \
    defined(CPU_MKL25Z32VLK4) || defined(CPU_MKL25Z64VLK4) || defined(CPU_MKL25Z128VLK4))

    #define KL25Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL25Z4/MKL25Z4.h"
    /* Extension register definitions */
    #include "device/MKL25Z4/MKL25Z4_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL25Z4/MKL25Z4_features.h"

#elif (defined(CPU_MKL26Z32VFM4) || defined(CPU_MKL26Z64VFM4) || defined(CPU_MKL26Z128VFM4) || \
    defined(CPU_MKL26Z32VFT4) || defined(CPU_MKL26Z64VFT4) || defined(CPU_MKL26Z128VFT4) || \
    defined(CPU_MKL26Z32VLH4) || defined(CPU_MKL26Z64VLH4) || defined(CPU_MKL26Z128VLH4) || \
    defined(CPU_MKL26Z256VLK4) || defined(CPU_MKL26Z128VLL4) || defined(CPU_MKL26Z256VLL4) || \
    defined(CPU_MKL26Z128VMC4) || defined(CPU_MKL26Z256VMC4))

    #define MKL26Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL26Z4/MKL26Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL26Z4/MKL26Z4_adc.h"
    #include "device/MKL26Z4/MKL26Z4_cmp.h"
    #include "device/MKL26Z4/MKL26Z4_dac.h"
    #include "device/MKL26Z4/MKL26Z4_dma.h"
    #include "device/MKL26Z4/MKL26Z4_dmamux.h"
    #include "device/MKL26Z4/MKL26Z4_fgpio.h"
    #include "device/MKL26Z4/MKL26Z4_ftfa.h"
    #include "device/MKL26Z4/MKL26Z4_gpio.h"
    #include "device/MKL26Z4/MKL26Z4_i2c.h"
    #include "device/MKL26Z4/MKL26Z4_i2s.h"
    #include "device/MKL26Z4/MKL26Z4_llwu.h"
    #include "device/MKL26Z4/MKL26Z4_lptmr.h"
    #include "device/MKL26Z4/MKL26Z4_mcg.h"
    #include "device/MKL26Z4/MKL26Z4_mcm.h"
    #include "device/MKL26Z4/MKL26Z4_mtb.h"
    #include "device/MKL26Z4/MKL26Z4_mtbdwt.h"
    #include "device/MKL26Z4/MKL26Z4_nv.h"
    #include "device/MKL26Z4/MKL26Z4_osc.h"
    #include "device/MKL26Z4/MKL26Z4_pit.h"
    #include "device/MKL26Z4/MKL26Z4_pmc.h"
    #include "device/MKL26Z4/MKL26Z4_port.h"
    #include "device/MKL26Z4/MKL26Z4_rcm.h"
    #include "device/MKL26Z4/MKL26Z4_rom.h"
    #include "device/MKL26Z4/MKL26Z4_rtc.h"
    #include "device/MKL26Z4/MKL26Z4_sim.h"
    #include "device/MKL26Z4/MKL26Z4_smc.h"
    #include "device/MKL26Z4/MKL26Z4_spi.h"
    #include "device/MKL26Z4/MKL26Z4_tpm.h"
    #include "device/MKL26Z4/MKL26Z4_tsi.h"
    #include "device/MKL26Z4/MKL26Z4_uart.h"
    #include "device/MKL26Z4/MKL26Z4_uart0.h"
    #include "device/MKL26Z4/MKL26Z4_usb.h"


#elif (defined(CPU_MKL27Z32VDA4) || defined(CPU_MKL27Z64VDA4) || defined(CPU_MKL27Z32VFM4) || \
    defined(CPU_MKL27Z64VFM4) || defined(CPU_MKL27Z32VFT4) || defined(CPU_MKL27Z64VFT4) || \
    defined(CPU_MKL27Z32VLH4) || defined(CPU_MKL27Z64VLH4) || defined(CPU_MKL27Z32VMP4) || \
    defined(CPU_MKL27Z64VMP4))

    #define KL27Z644_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL27Z644/MKL27Z644.h"
    /* Extension register definitions */
    #include "device/MKL27Z644/MKL27Z644_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL27Z644/MKL27Z644_features.h"

#elif (defined(CPU_MKL33Z32VFT4) || defined(CPU_MKL33Z64VFT4) || defined(CPU_MKL33Z32VLH4) || \
    defined(CPU_MKL33Z64VLH4) || defined(CPU_MKL33Z32VLK4) || defined(CPU_MKL33Z64VLK4) || \
    defined(CPU_MKL33Z32VMP4) || defined(CPU_MKL33Z64VMP4))

    #define KL33Z644_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL33Z644/MKL33Z644.h"
    /* Extension register definitions */
    #include "device/MKL33Z644/MKL33Z644_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL33Z644/MKL33Z644_features.h"


#elif (defined(CPU_MKL43Z128VLH4) || defined(CPU_MKL43Z256VLH4) || defined(CPU_MKL43Z128VMP4) || \
    defined(CPU_MKL43Z256VMP4))

    #define KL43Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL43Z4/MKL43Z4.h"
    /* Extension register definitions */
    #include "device/MKL43Z4/MKL43Z4_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL43Z4/MKL43Z4_features.h"

#elif (defined(CPU_MKL46Z128VLH4) || defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z128VLL4) || \
    defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z128VMC4) || defined(CPU_MKL46Z256VMC4))

    #define MKL46Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL46Z4/MKL46Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL46Z4/MKL46Z4_adc.h"
    #include "device/MKL46Z4/MKL46Z4_cmp.h"
    #include "device/MKL46Z4/MKL46Z4_dac.h"
    #include "device/MKL46Z4/MKL46Z4_dma.h"
    #include "device/MKL46Z4/MKL46Z4_dmamux.h"
    #include "device/MKL46Z4/MKL46Z4_fgpio.h"
    #include "device/MKL46Z4/MKL46Z4_ftfa.h"
    #include "device/MKL46Z4/MKL46Z4_gpio.h"
    #include "device/MKL46Z4/MKL46Z4_i2c.h"
    #include "device/MKL46Z4/MKL46Z4_i2s.h"
    #include "device/MKL46Z4/MKL46Z4_lcd.h"
    #include "device/MKL46Z4/MKL46Z4_llwu.h"
    #include "device/MKL46Z4/MKL46Z4_lptmr.h"
    #include "device/MKL46Z4/MKL46Z4_mcg.h"
    #include "device/MKL46Z4/MKL46Z4_mcm.h"
    #include "device/MKL46Z4/MKL46Z4_mtb.h"
    #include "device/MKL46Z4/MKL46Z4_mtbdwt.h"
    #include "device/MKL46Z4/MKL46Z4_nv.h"
    #include "device/MKL46Z4/MKL46Z4_osc.h"
    #include "device/MKL46Z4/MKL46Z4_pit.h"
    #include "device/MKL46Z4/MKL46Z4_pmc.h"
    #include "device/MKL46Z4/MKL46Z4_port.h"
    #include "device/MKL46Z4/MKL46Z4_rcm.h"
    #include "device/MKL46Z4/MKL46Z4_rom.h"
    #include "device/MKL46Z4/MKL46Z4_rtc.h"
    #include "device/MKL46Z4/MKL46Z4_sim.h"
    #include "device/MKL46Z4/MKL46Z4_smc.h"
    #include "device/MKL46Z4/MKL46Z4_spi.h"
    #include "device/MKL46Z4/MKL46Z4_tpm.h"
    #include "device/MKL46Z4/MKL46Z4_tsi.h"
    #include "device/MKL46Z4/MKL46Z4_uart.h"
    #include "device/MKL46Z4/MKL46Z4_uart0.h"
    #include "device/MKL46Z4/MKL46Z4_usb.h"

#elif (defined(CPU_MKV30F128VFM10) || defined(CPU_MKV30F64VFM10) || defined(CPU_MKV30F128VLF10) || \
    defined(CPU_MKV30F64VLF10) || defined(CPU_MKV30F128VLH10) || defined(CPU_MKV30F64VLH10))

    #define MKV30F12810_SERIES

    #define KV30F12810_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV30F12810/MKV30F12810.h"
    /* Extension register definitions */
    #include "device/MKV30F12810/MKV30F12810_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV30F12810/MKV30F12810_features.h"

#elif (defined(CPU_MKV31F128VLH10) || defined(CPU_MKV31F128VLL10))

    #define MKV31F12810_SERIES

    #define KV31F12810_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV31F12810/MKV31F12810.h"
    /* Extension register definitions */
    #include "device/MKV31F12810/MKV31F12810_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV31F12810/MKV31F12810_features.h"

#elif (defined(CPU_MKV31F256VLH12) || defined(CPU_MKV31F256VLL12))

    #define MKV31F25612_SERIES

    #define KV31F25612_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV31F25612/MKV31F25612.h"
    /* Extension register definitions */
    #include "device/MKV31F25612/MKV31F25612_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV31F25612/MKV31F25612_features.h"

#elif (defined(CPU_MKV31F512VLH12) || defined(CPU_MKV31F512VLL12))

    #define MKV31F51212_SERIES

    #define KV31F51212_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV31F51212/MKV31F51212.h"
    /* Extension register definitions */
    #include "device/MKV31F51212/MKV31F51212_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV31F51212/MKV31F51212_features.h"

#elif (defined(CPU_MKV40F128VLH15) || defined(CPU_MKV40F128VLL15) || defined(CPU_MKV40F256VLH15) || \
    defined(CPU_MKV40F256VLL15) || defined(CPU_MKV40F64VLH15))

    #define MKV40F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV40F15/MKV40F15.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKV40F15/MKV40F15_adc.h"
    #include "device/MKV40F15/MKV40F15_aoi.h"
    #include "device/MKV40F15/MKV40F15_can.h"
    #include "device/MKV40F15/MKV40F15_cmp.h"
    #include "device/MKV40F15/MKV40F15_crc.h"
    #include "device/MKV40F15/MKV40F15_dma.h"
    #include "device/MKV40F15/MKV40F15_dmamux.h"
    #include "device/MKV40F15/MKV40F15_enc.h"
    #include "device/MKV40F15/MKV40F15_ewm.h"
    #include "device/MKV40F15/MKV40F15_fmc.h"
    #include "device/MKV40F15/MKV40F15_ftfa.h"
    #include "device/MKV40F15/MKV40F15_ftm.h"
    #include "device/MKV40F15/MKV40F15_gpio.h"
    #include "device/MKV40F15/MKV40F15_i2c.h"
    #include "device/MKV40F15/MKV40F15_llwu.h"
    #include "device/MKV40F15/MKV40F15_lptmr.h"
    #include "device/MKV40F15/MKV40F15_mcg.h"
    #include "device/MKV40F15/MKV40F15_mcm.h"
    #include "device/MKV40F15/MKV40F15_nv.h"
    #include "device/MKV40F15/MKV40F15_osc.h"
    #include "device/MKV40F15/MKV40F15_pdb.h"
    #include "device/MKV40F15/MKV40F15_pit.h"
    #include "device/MKV40F15/MKV40F15_pmc.h"
    #include "device/MKV40F15/MKV40F15_port.h"
    #include "device/MKV40F15/MKV40F15_rcm.h"
    #include "device/MKV40F15/MKV40F15_sim.h"
    #include "device/MKV40F15/MKV40F15_smc.h"
    #include "device/MKV40F15/MKV40F15_spi.h"
    #include "device/MKV40F15/MKV40F15_uart.h"
    #include "device/MKV40F15/MKV40F15_wdog.h"
    #include "device/MKV40F15/MKV40F15_xbara.h"
    #include "device/MKV40F15/MKV40F15_xbarb.h"

#elif (defined(CPU_MKV43F128VLH15) || defined(CPU_MKV43F128VLL15) || defined(CPU_MKV43F64VLH15))

    #define MKV43F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV43F15/MKV43F15.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKV43F15/MKV43F15_adc.h"
    #include "device/MKV43F15/MKV43F15_aoi.h"
    #include "device/MKV43F15/MKV43F15_can.h"
    #include "device/MKV43F15/MKV43F15_cmp.h"
    #include "device/MKV43F15/MKV43F15_crc.h"
    #include "device/MKV43F15/MKV43F15_dma.h"
    #include "device/MKV43F15/MKV43F15_dmamux.h"
    #include "device/MKV43F15/MKV43F15_enc.h"
    #include "device/MKV43F15/MKV43F15_ewm.h"
    #include "device/MKV43F15/MKV43F15_fmc.h"
    #include "device/MKV43F15/MKV43F15_ftfa.h"
    #include "device/MKV43F15/MKV43F15_gpio.h"
    #include "device/MKV43F15/MKV43F15_i2c.h"
    #include "device/MKV43F15/MKV43F15_llwu.h"
    #include "device/MKV43F15/MKV43F15_lptmr.h"
    #include "device/MKV43F15/MKV43F15_mcg.h"
    #include "device/MKV43F15/MKV43F15_mcm.h"
    #include "device/MKV43F15/MKV43F15_nv.h"
    #include "device/MKV43F15/MKV43F15_osc.h"
    #include "device/MKV43F15/MKV43F15_pdb.h"
    #include "device/MKV43F15/MKV43F15_pit.h"
    #include "device/MKV43F15/MKV43F15_pmc.h"
    #include "device/MKV43F15/MKV43F15_port.h"
    #include "device/MKV43F15/MKV43F15_pwm.h"
    #include "device/MKV43F15/MKV43F15_rcm.h"
    #include "device/MKV43F15/MKV43F15_sim.h"
    #include "device/MKV43F15/MKV43F15_smc.h"
    #include "device/MKV43F15/MKV43F15_spi.h"
    #include "device/MKV43F15/MKV43F15_uart.h"
    #include "device/MKV43F15/MKV43F15_wdog.h"
    #include "device/MKV43F15/MKV43F15_xbara.h"
    #include "device/MKV43F15/MKV43F15_xbarb.h"

#elif (defined(CPU_MKV44F128VLH15) || defined(CPU_MKV44F128VLL15) || defined(CPU_MKV44F64VLH15))

    #define MKV44F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV44F15/MKV44F15.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKV44F15/MKV44F15_adc.h"
    #include "device/MKV44F15/MKV44F15_aoi.h"
    #include "device/MKV44F15/MKV44F15_can.h"
    #include "device/MKV44F15/MKV44F15_cmp.h"
    #include "device/MKV44F15/MKV44F15_crc.h"
    #include "device/MKV44F15/MKV44F15_dac.h"
    #include "device/MKV44F15/MKV44F15_dma.h"
    #include "device/MKV44F15/MKV44F15_dmamux.h"
    #include "device/MKV44F15/MKV44F15_enc.h"
    #include "device/MKV44F15/MKV44F15_ewm.h"
    #include "device/MKV44F15/MKV44F15_fmc.h"
    #include "device/MKV44F15/MKV44F15_ftfa.h"
    #include "device/MKV44F15/MKV44F15_gpio.h"
    #include "device/MKV44F15/MKV44F15_i2c.h"
    #include "device/MKV44F15/MKV44F15_llwu.h"
    #include "device/MKV44F15/MKV44F15_lptmr.h"
    #include "device/MKV44F15/MKV44F15_mcg.h"
    #include "device/MKV44F15/MKV44F15_mcm.h"
    #include "device/MKV44F15/MKV44F15_nv.h"
    #include "device/MKV44F15/MKV44F15_osc.h"
    #include "device/MKV44F15/MKV44F15_pdb.h"
    #include "device/MKV44F15/MKV44F15_pit.h"
    #include "device/MKV44F15/MKV44F15_pmc.h"
    #include "device/MKV44F15/MKV44F15_port.h"
    #include "device/MKV44F15/MKV44F15_pwm.h"
    #include "device/MKV44F15/MKV44F15_rcm.h"
    #include "device/MKV44F15/MKV44F15_sim.h"
    #include "device/MKV44F15/MKV44F15_smc.h"
    #include "device/MKV44F15/MKV44F15_spi.h"
    #include "device/MKV44F15/MKV44F15_uart.h"
    #include "device/MKV44F15/MKV44F15_wdog.h"
    #include "device/MKV44F15/MKV44F15_xbara.h"
    #include "device/MKV44F15/MKV44F15_xbarb.h"

#elif (defined(CPU_MKV45F128VLH15) || defined(CPU_MKV45F128VLL15) || defined(CPU_MKV45F256VLH15) || \
    defined(CPU_MKV45F256VLL15))

    #define MKV45F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV45F15/MKV45F15.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKV45F15/MKV45F15_adc.h"
    #include "device/MKV45F15/MKV45F15_aoi.h"
    #include "device/MKV45F15/MKV45F15_can.h"
    #include "device/MKV45F15/MKV45F15_cmp.h"
    #include "device/MKV45F15/MKV45F15_crc.h"
    #include "device/MKV45F15/MKV45F15_dma.h"
    #include "device/MKV45F15/MKV45F15_dmamux.h"
    #include "device/MKV45F15/MKV45F15_enc.h"
    #include "device/MKV45F15/MKV45F15_ewm.h"
    #include "device/MKV45F15/MKV45F15_fmc.h"
    #include "device/MKV45F15/MKV45F15_ftfa.h"
    #include "device/MKV45F15/MKV45F15_ftm.h"
    #include "device/MKV45F15/MKV45F15_gpio.h"
    #include "device/MKV45F15/MKV45F15_i2c.h"
    #include "device/MKV45F15/MKV45F15_llwu.h"
    #include "device/MKV45F15/MKV45F15_lptmr.h"
    #include "device/MKV45F15/MKV45F15_mcg.h"
    #include "device/MKV45F15/MKV45F15_mcm.h"
    #include "device/MKV45F15/MKV45F15_nv.h"
    #include "device/MKV45F15/MKV45F15_osc.h"
    #include "device/MKV45F15/MKV45F15_pdb.h"
    #include "device/MKV45F15/MKV45F15_pit.h"
    #include "device/MKV45F15/MKV45F15_pmc.h"
    #include "device/MKV45F15/MKV45F15_port.h"
    #include "device/MKV45F15/MKV45F15_pwm.h"
    #include "device/MKV45F15/MKV45F15_rcm.h"
    #include "device/MKV45F15/MKV45F15_sim.h"
    #include "device/MKV45F15/MKV45F15_smc.h"
    #include "device/MKV45F15/MKV45F15_spi.h"
    #include "device/MKV45F15/MKV45F15_uart.h"
    #include "device/MKV45F15/MKV45F15_wdog.h"
    #include "device/MKV45F15/MKV45F15_xbara.h"
    #include "device/MKV45F15/MKV45F15_xbarb.h"

#elif (defined(CPU_MKV46F128VLH15) || defined(CPU_MKV46F128VLL15) || defined(CPU_MKV46F256VLH15) || \
    defined(CPU_MKV46F256VLL15))

    #define MKV46F15_SERIES

    #define KV46F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV46F15/MKV46F15.h"
    /* Extension register definitions */
    #include "device/MKV46F15/MKV46F15_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV46F15/MKV46F15_features.h"

#elif (defined(CPU_MKL28T256VLH7) || defined(CPU_MKL28T512VLH7))
    #define MKL28T7_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL28T7/MKL28T7_CORE0.h"
    /* Extension register headers. */
    #include "device/MKL28T7/MKL28T7_CORE0_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKL28T7/MKL28T7_CORE0_features.h"

#elif (defined(CPU_PKE18F512VLH15) || defined(CPU_PKE18F512VLL15))
    #define PKE18F15_SERIES

    /* CMSIS-style register definitions */
    #include "device/PKE18F15/PKE18F15.h"
    /* Extension register definitions */
    #include "device/PKE18F15/PKE18F15_extension.h"
    /* CPU specific feature definitions */
    #include "device/PKE18F15/PKE18F15_features.h"

#elif (defined(CPU_MKV11Z128VLH7) || defined(CPU_MKV11Z128VLL7) || defined(CPU_MKV11Z128VLC7) || \
    defined(CPU_MKV11Z128VFM5))

    #define MKV11Z7_SERIES

    #define KV11Z7_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV11Z7/MKV11Z7.h"
    /* Extension register definitions */
    #include "device/MKV11Z7/MKV11Z7_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV11Z7/MKV11Z7_features.h"

#elif (defined(CPU_MK87PD4M0CMD36))
    #define MK87PD4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MK87PD4/MK87PD4.h"

    /* Extension register definitions */
    #include "device/MK87PD4/MK87PD4_registers.h"

#elif (defined(CPU_MKL81Z128VDC7) || defined(CPU_MKL81Z128VLK7) || defined(CPU_MKL81Z128VLL7) || \
	defined(CPU_MKL81Z128VLH7) || defined(CPU_MKL81Z128VMP7))
	#define MKL80Z7_SERIES

	#include "device/MKL80Z7/MKL80Z7.h"
	#include "device/MKL80Z7/MKL80Z7_extension.h"
	#include "device/MKL80Z7/MKL80Z7_features.h"

#elif (defined(CPU_MKV58F1M0VMD22) || defined(CPU_MKV58F1M0VLQ22) || defined(CPU_MKV58F1M0VLL22) || \
    defined(CPU_MKV56F1M0VMD22) || defined(CPU_MKV56F1M0VLQ22) || defined(CPU_MKV56F1M0VLL22) || \
    defined(CPU_MKV58F512VMD22) || defined(CPU_MKV58F512VLQ22) || defined(CPU_MKV58F512VLL22) || \
    defined(CPU_MKV56F512VMD22) || defined(CPU_MKV56F512VLQ22) || defined(CPU_MKV56F512VLL22))    

    #define KV58F22_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKV58F22/MKV58F22.h"
    /* Extension register definitions */
    #include "device/MKV58F22/MKV58F22_extension.h"
    /* CPU specific feature definitions */
    #include "device/MKV58F22/MKV58F22_features.h"

#else
    #error "No valid CPU defined!"
#endif

#endif /* __FSL_DEVICE_REGISTERS_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
