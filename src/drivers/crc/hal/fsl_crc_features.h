/*
** ###################################################################
**     Version:             rev. 1.0, 2014-05-14
**     Build:               b140715
**
**     Abstract:
**         Chip specific module features.
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
**     - rev. 1.0 (2014-05-14)
**         Customer release.
**
** ###################################################################
*/

#if !defined(__FSL_CRC_FEATURES_H__)
#define __FSL_CRC_FEATURES_H__

#if defined(CPU_MK02FN128VFM10) || defined(CPU_MK02FN64VFM10) || defined(CPU_MK02FN128VLF10) || defined(CPU_MK02FN64VLF10) || \
    defined(CPU_MK02FN128VLH10) || defined(CPU_MK02FN64VLH10) || defined(CPU_MK11DX128AVLK5) || defined(CPU_MK11DX256AVLK5) || \
    defined(CPU_MK11DN512AVLK5) || defined(CPU_MK11DX128AVMC5) || defined(CPU_MK11DX256AVMC5) || defined(CPU_MK11DN512AVMC5) || \
    defined(CPU_MK21DX128AVLK5) || defined(CPU_MK21DX256AVLK5) || defined(CPU_MK21DN512AVLK5) || defined(CPU_MK21DX128AVMC5) || \
    defined(CPU_MK21DX256AVMC5) || defined(CPU_MK21DN512AVMC5) || defined(CPU_MK21FX512AVLQ12) || defined(CPU_MK21FN1M0AVLQ12) || \
    defined(CPU_MK21FX512AVMC12) || defined(CPU_MK21FN1M0AVMC12) || defined(CPU_MK21FX512AVMD12) || defined(CPU_MK21FN1M0AVMD12) || \
    defined(CPU_MK22FN128VDC10) || defined(CPU_MK22FN128VLH10) || defined(CPU_MK22FN128VLL10) || defined(CPU_MK22FN128VMP10) || \
    defined(CPU_MK22FN256VDC12) || defined(CPU_MK22FN256VLH12) || defined(CPU_MK22FN256VLL12) || defined(CPU_MK22FN256VMP12) || \
    defined(CPU_MK22FN512VDC12) || defined(CPU_MK22FN512VLH12) || defined(CPU_MK22FN512VLL12) || defined(CPU_MK24FN1M0VDC12) || \
    defined(CPU_MK24FN1M0VLL12) || defined(CPU_MK24FN1M0VLQ12) || defined(CPU_MK24FN256VDC12) || defined(CPU_MK63FN1M0VLQ12) || \
    defined(CPU_MK63FN1M0VMD12) || defined(CPU_MK64FX512VDC12) || defined(CPU_MK64FN1M0VDC12) || defined(CPU_MK64FX512VLL12) || \
    defined(CPU_MK64FN1M0VLL12) || defined(CPU_MK64FX512VLQ12) || defined(CPU_MK64FN1M0VLQ12) || defined(CPU_MK64FX512VMD12) || \
    defined(CPU_MK64FN1M0VMD12) || defined(CPU_MK65FN2M0CAC18) || defined(CPU_MK65FX1M0CAC18) || defined(CPU_MK65FN2M0VMI18) || \
    defined(CPU_MK65FX1M0VMI18) || defined(CPU_MK66FN2M0VLQ18) || defined(CPU_MK66FX1M0VLQ18) || defined(CPU_MK66FN2M0VMD18) || \
    defined(CPU_MK66FX1M0VMD18) || defined(CPU_MKE02Z64VLC2) || defined(CPU_MKE02Z32VLC2) || defined(CPU_MKE02Z16VLC2) || \
    defined(CPU_SKEAZN64MLC2) || defined(CPU_SKEAZN32MLC2) || defined(CPU_SKEAZN16MLC2) || defined(CPU_MKE02Z64VLC4) || \
    defined(CPU_MKE02Z32VLC4) || defined(CPU_MKE02Z16VLC4) || defined(CPU_MKE02Z64VLD2) || defined(CPU_MKE02Z32VLD2) || \
    defined(CPU_MKE02Z16VLD2) || defined(CPU_SKEAZN64MLD2) || defined(CPU_SKEAZN32MLD2) || defined(CPU_SKEAZN16MLD2) || \
    defined(CPU_MKE02Z64VLD4) || defined(CPU_MKE02Z32VLD4) || defined(CPU_MKE02Z16VLD4) || defined(CPU_MKE02Z64VLH2) || \
    defined(CPU_MKE02Z64VQH2) || defined(CPU_MKE02Z32VLH2) || defined(CPU_MKE02Z32VQH2) || defined(CPU_SKEAZN64MLH2) || \
    defined(CPU_SKEAZN32MLH2) || defined(CPU_MKE02Z64VLH4) || defined(CPU_MKE02Z64VQH4) || defined(CPU_MKE02Z32VLH4) || \
    defined(CPU_MKE02Z32VQH4) || defined(CPU_MKE04Z128VLD4) || defined(CPU_MKE04Z64VLD4) || defined(CPU_MKE04Z128VLK4) || \
    defined(CPU_MKE04Z64VLK4) || defined(CPU_MKE04Z128VQH4) || defined(CPU_MKE04Z64VQH4) || defined(CPU_MKE04Z128VLH4) || \
    defined(CPU_MKE04Z64VLH4) || defined(CPU_MKE04Z8VFK4) || defined(CPU_SKEAZN8MFK) || defined(CPU_MKE04Z8VTG4) || \
    defined(CPU_SKEAZN8MTG) || defined(CPU_MKE04Z8VWJ4) || defined(CPU_MKE06Z128VLD4) || defined(CPU_MKE06Z64VLD4) || \
    defined(CPU_MKE06Z128VLK4) || defined(CPU_MKE06Z64VLK4) || defined(CPU_MKE06Z128VQH4) || defined(CPU_MKE06Z64VQH4) || \
    defined(CPU_MKE06Z128VLH4) || defined(CPU_MKE06Z64VLH4) || defined(CPU_MKV10Z16VFM7) || defined(CPU_MKV10Z16VLC7) || \
    defined(CPU_MKV10Z16VLF7) || defined(CPU_MKV10Z32VFM7) || defined(CPU_MKV10Z32VLC7) || defined(CPU_MKV10Z32VLF7) || \
    defined(CPU_MKV30F128VFM10) || defined(CPU_MKV30F64VFM10) || defined(CPU_MKV30F128VLF10) || defined(CPU_MKV30F64VLF10) || \
    defined(CPU_MKV30F128VLH10) || defined(CPU_MKV30F64VLH10) || defined(CPU_MKV31F128VLH10) || defined(CPU_MKV31F128VLL10) || \
    defined(CPU_MKV31F256VLH12) || defined(CPU_MKV31F256VLL12) || defined(CPU_MKV31F512VLH12) || defined(CPU_MKV31F512VLL12) || \
    defined(CPU_MKV40F128VLH15) || defined(CPU_MKV40F128VLL15) || defined(CPU_MKV40F256VLH15) || defined(CPU_MKV40F256VLL15) || \
    defined(CPU_MKV40F64VLH15) || defined(CPU_MKV43F128VLH15) || defined(CPU_MKV43F128VLL15) || defined(CPU_MKV43F64VLH15) || \
    defined(CPU_MKV44F128VLH15) || defined(CPU_MKV44F128VLL15) || defined(CPU_MKV44F64VLH15) || defined(CPU_MKV45F128VLH15) || \
    defined(CPU_MKV45F128VLL15) || defined(CPU_MKV45F256VLH15) || defined(CPU_MKV45F256VLL15) || defined(CPU_MKV46F128VLH15) || \
    defined(CPU_MKV46F128VLL15) || defined(CPU_MKV46F256VLH15) || defined(CPU_MKV46F256VLL15) || defined(CPU_SKEAZ128MLH) || \
    defined(CPU_SKEAZ64MLH) || defined(CPU_SKEAZ128MLK) || defined(CPU_SKEAZ64MLK) || \
    defined(CPU_MK80FN256VLQR) || \
    defined(CPU_MKL28T512VLH7) || \
    defined (CPU_PKE18F512VLH15) || defined (CPU_PKE18F512VLL15)
    /* @brief Has data register with name CRC */
    #define FSL_FEATURE_CRC_HAS_CRC_REG (0)
#elif defined(CPU_MK10DN512VLK10) || defined(CPU_MK10DN512VLL10) || defined(CPU_MK10DX128VLQ10) || defined(CPU_MK10DX256VLQ10) || \
    defined(CPU_MK10DN512VLQ10) || defined(CPU_MK10DN512VMC10) || defined(CPU_MK10DX128VMD10) || defined(CPU_MK10DX256VMD10) || \
    defined(CPU_MK10DN512VMD10) || defined(CPU_MK20DN512VLK10) || defined(CPU_MK20DN512VLL10) || defined(CPU_MK20DX128VLQ10) || \
    defined(CPU_MK20DX256VLQ10) || defined(CPU_MK20DN512VLQ10) || defined(CPU_MK20DX256VMC10) || defined(CPU_MK20DN512VMC10) || \
    defined(CPU_MK20DX128VMD10) || defined(CPU_MK20DX256VMD10) || defined(CPU_MK20DN512VMD10) || defined(CPU_MK20DX128VMP5) || \
    defined(CPU_MK20DN128VMP5) || defined(CPU_MK20DX64VMP5) || defined(CPU_MK20DN64VMP5) || defined(CPU_MK20DX32VMP5) || \
    defined(CPU_MK20DN32VMP5) || defined(CPU_MK20DX128VLH5) || defined(CPU_MK20DN128VLH5) || defined(CPU_MK20DX64VLH5) || \
    defined(CPU_MK20DN64VLH5) || defined(CPU_MK20DX32VLH5) || defined(CPU_MK20DN32VLH5) || defined(CPU_MK20DX128VFM5) || \
    defined(CPU_MK20DN128VFM5) || defined(CPU_MK20DX64VFM5) || defined(CPU_MK20DN64VFM5) || defined(CPU_MK20DX32VFM5) || \
    defined(CPU_MK20DN32VFM5) || defined(CPU_MK20DX128VFT5) || defined(CPU_MK20DN128VFT5) || defined(CPU_MK20DX64VFT5) || \
    defined(CPU_MK20DN64VFT5) || defined(CPU_MK20DX32VFT5) || defined(CPU_MK20DN32VFT5) || defined(CPU_MK20DX128VLF5) || \
    defined(CPU_MK20DN128VLF5) || defined(CPU_MK20DX64VLF5) || defined(CPU_MK20DN64VLF5) || defined(CPU_MK20DX32VLF5) || \
    defined(CPU_MK20DN32VLF5) || defined(CPU_MK30DN512VLK10) || defined(CPU_MK30DN512VLL10) || defined(CPU_MK30DX128VLQ10) || \
    defined(CPU_MK30DX256VLQ10) || defined(CPU_MK30DN512VLQ10) || defined(CPU_MK30DN512VMC10) || defined(CPU_MK30DX128VMD10) || \
    defined(CPU_MK30DX256VMD10) || defined(CPU_MK30DN512VMD10) || defined(CPU_MK40DN512VLK10) || defined(CPU_MK40DN512VLL10) || \
    defined(CPU_MK40DX128VLQ10) || defined(CPU_MK40DX256VLQ10) || defined(CPU_MK40DN512VLQ10) || defined(CPU_MK40DN512VMC10) || \
    defined(CPU_MK40DX128VMD10) || defined(CPU_MK40DX256VMD10) || defined(CPU_MK40DN512VMD10) || defined(CPU_MK50DX256CLL10) || \
    defined(CPU_MK50DN512CLL10) || defined(CPU_MK50DN512CLQ10) || defined(CPU_MK50DX256CMC10) || defined(CPU_MK50DN512CMC10) || \
    defined(CPU_MK50DN512CMD10) || defined(CPU_MK50DX256CMD10) || defined(CPU_MK50DX256CLK10) || defined(CPU_MK51DX256CLL10) || \
    defined(CPU_MK51DN512CLL10) || defined(CPU_MK51DN256CLQ10) || defined(CPU_MK51DN512CLQ10) || defined(CPU_MK51DX256CMC10) || \
    defined(CPU_MK51DN512CMC10) || defined(CPU_MK51DN256CMD10) || defined(CPU_MK51DN512CMD10) || defined(CPU_MK51DX256CLK10) || \
    defined(CPU_MK52DN512CLQ10) || defined(CPU_MK52DN512CMD10) || defined(CPU_MK53DN512CLQ10) || defined(CPU_MK53DX256CLQ10) || \
    defined(CPU_MK53DN512CMD10) || defined(CPU_MK53DX256CMD10) || defined(CPU_MK60DN256VLL10) || defined(CPU_MK60DX256VLL10) || \
    defined(CPU_MK60DN512VLL10) || defined(CPU_MK60DN256VLQ10) || defined(CPU_MK60DX256VLQ10) || defined(CPU_MK60DN512VLQ10) || \
    defined(CPU_MK60DN256VMC10) || defined(CPU_MK60DX256VMC10) || defined(CPU_MK60DN512VMC10) || defined(CPU_MK60DN256VMD10) || \
    defined(CPU_MK60DX256VMD10) || defined(CPU_MK60DN512VMD10) || defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FX512VMJ12) || \
    defined(CPU_MK70FN1M0VMJ15) || defined(CPU_MK70FX512VMJ15) || defined(CPU_MKL17Z32VLH4) || defined(CPU_MKL17Z64VLH4) || \
    defined(CPU_MKL27Z32VLH4) || defined(CPU_MKL27Z64VLH4)
    /* @brief Has data register with name CRC */
    #define FSL_FEATURE_CRC_HAS_CRC_REG (1)
#else
    #error "No valid CPU defined!"
#endif

#endif /* __FSL_CRC_FEATURES_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
