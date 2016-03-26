/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#ifndef __scuart_H__
#define __scuart_H__

#include "device/fsl_device_registers.h"
#include "fsl_platform_status.h"

/*!
 * @addtogroup scuart_driver
 * @{
 */

/////////////////////////////////////////////////////////////////////////////
// API
/////////////////////////////////////////////////////////////////////////////

//! @name SCUART Driver
//@{

//! @brief Type of callback for scuart data sink
typedef void (*scuart_data_sink_func_t)(uint8_t byte);

/*! @brief Reset SCUART peripheral
 *
 * @param uartch SCUART peripheral base address
 */
void scuart_reset (UART_Type * uartch);

/*! @brief Initialize SCUART peripheral
 *
 * @param uartch SCUART peripheral base address
 * @param uartclk SCUART peripheral clock frequency
 * @param baud SCUART baud rate
 * @param function data sink callback function
 *
 * @return An error code or kStatus_Success
 */
status_t scuart_init (UART_Type * uartch, int uartclk, int baud, scuart_data_sink_func_t function);

/*! @brief Send data from SCUART with format 8-N-1
 *
 * @param channel SCUART peripheral base address
 * @param ch data to send
 *
 */
void scuart_putchar (UART_Type * channel, char ch);

/*! @brief Shutdown SCUART peripheral
 *
 * @param uartch SCUART peripheral base address
 *
 */
void scuart_shutdown (UART_Type * uartch);

//@}

/*! @}*/

#endif /* __scuart_H__ */
