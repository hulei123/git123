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

#include "application_common.h"
#include "device/fsl_device_registers.h"
#include "lpuart/hal/fsl_lpuart_hal.h"
#include "target_config.h"
#include <assert.h>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////


void init_hardware(void)
{
    // Enable clocks to ports.
    SIM_SET_SCGC5(SIM, SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK);

    // Select the MCGIRCLK as UART0 clock source.

    SIM_BWR_SOPT2_LPUART0SRC(SIM, 3);

    // Update SystemCoreClock. Out of reset, the LIRC is enabled. FOPT bits set the OUTDIV1 value.
    uint32_t lirc = (MCG_BRD_C2_IRCS(MCG) == 1) ? kLIRC8M : kLIRC2M;
    SystemCoreClock = lirc / (SIM_BRD_CLKDIV1_OUTDIV1(SIM) + 1);
}

uint32_t get_uart_clock(unsigned int instance)
{
    uint32_t lirc = (MCG_BRD_C2_IRCS(MCG) == 1) ? kLIRC8M : kLIRC2M;
    return lirc >> MCG_BRD_SC_FCRDIV(MCG);
}

void init_term_uart(void)
{
    // Init pin mux for term uart.
    PORT_BWR_PCR_MUX(PORTB, 2, 2);   // UART0_RX is ALT2 for pin PTB2
    PORT_BWR_PCR_MUX(PORTB, 1, 2);   // UART0_TX is ALT2 for pin PTB1

    // Ungate the LPUART clock.
    SIM_SET_SCGC5(SIM, SIM_SCGC5_LPUART0_MASK);

    lpuart_hal_init(LPUART0, get_uart_clock(0), TERMINAL_BAUD);
}


int fputc(int ch, FILE *fp)
{
    while (!lpuart_hal_is_transmit_data_register_empty(LPUART0));
    lpuart_hal_putchar(LPUART0, ch);
    
    return ch;
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

