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

#include "device/fsl_device_registers.h"

#define LED1_OFFSET     3
#define LED2_OFFSET     4
#define LED3_OFFSET     4
#define LED4_OFFSET     5

#define LED1            (1)  //green
#define LED2            (2)  //yellow
#define LED3            (3)  //green
#define LED4            (4)  //yellow

#define LED1_PORT       (PORTC)
#define LED2_PORT       (PORTC)
#define LED3_PORT       (PORTD)
#define LED4_PORT       (PORTD)

#define LED1_GPIO       (GPIOC)
#define LED2_GPIO       (GPIOC)
#define LED3_GPIO       (GPIOD)
#define LED4_GPIO       (GPIOD)

#define MAX_LED_INDEX   (LED2)

static void init_hardware(void)
{    
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );
    
    // Enable the LED pins GPIO
    PORT_BWR_PCR_MUX(LED1_PORT, LED1_OFFSET, 1);
#if MAX_LED_INDEX >= LED2     
    PORT_BWR_PCR_MUX(LED2_PORT, LED2_OFFSET, 1);
#if MAX_LED_INDEX >= LED3     
    PORT_BWR_PCR_MUX(LED3_PORT, LED3_OFFSET, 1);
#if MAX_LED_INDEX >= LED4     
    PORT_BWR_PCR_MUX(LED4_PORT, LED4_OFFSET, 1);
#endif
#endif
#endif    
    
    // Set ports to outputs
    // Set ports to outputs
    LED1_GPIO->PDDR |= (1<<LED1_OFFSET);
#if MAX_LED_INDEX >= LED2     
    LED2_GPIO->PDDR |= (1<<LED2_OFFSET);
#if MAX_LED_INDEX >= LED3     
    LED3_GPIO->PDDR |= (1<<LED3_OFFSET);
#if MAX_LED_INDEX >= LED4     
    LED4_GPIO->PDDR |= (1<<LED4_OFFSET);
#endif
#endif
#endif    
}

static void led_toggle(uint32_t leds)
{
    if (leds & LED1)
    {
        LED1_GPIO->PTOR  |= (1 << LED1_OFFSET);
    }
#if MAX_LED_INDEX >= LED2    
    if (leds & LED2)
    {
        LED2_GPIO->PTOR  |= (1 << LED2_OFFSET);
    }
#if MAX_LED_INDEX >= LED3    
    if (leds & LED3)
    {
        LED3_GPIO->PTOR  |= (1 << LED3_OFFSET);
    }
#if MAX_LED_INDEX >= LED4    
    if (leds & LED4)
    {
        LED4_GPIO->PTOR  |= (1 << LED4_OFFSET);
    }
#endif // MAX_LED_INDEX >= LED4
#endif // MAX_LED_INDEX >= LED3
#endif // MAX_LED_INDEX >= LED2
}

void delay(void)
{
    volatile uint32_t delayTicks = 2000000;
    
    while(delayTicks--)
    {
        __ASM("nop");
    }
}


int main(void)
{
    init_hardware();
    
    uint32_t leds = LED1;
    while(1)
    {
        led_toggle(leds);
        delay();
        led_toggle(leds);
        
        leds <<= 1;
        if(leds > MAX_LED_INDEX)
        {
            leds = LED1;
        }
    }
}