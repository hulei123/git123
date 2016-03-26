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

#define LED1_OFFSET     0
#define LED2_OFFSET     1
#define LED3_OFFSET     2
#define LED4_OFFSET     3

#define LED1            (1)  //green
#define LED2            (2)  //yellow
#define LED3            (4)  //green
#define LED4            (8)  //yellow

#define LED1_GPIO       (PTE)
#define LED2_GPIO       (PTE)
#define LED3_GPIO       (PTE)

static void init_hardware(void)
{    
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );
    
#if defined(TOWER)
    // Enable the LED pins GPIO
    PORT_BWR_PCR_MUX(PORTD, 0, 1);
    PORT_BWR_PCR_MUX(PORTD, 1, 1);
    PORT_BWR_PCR_MUX(PORTD, 2, 1);
    PORT_BWR_PCR_MUX(PORTD, 3, 1);
    
    // Set ports to outputs
    PTD->PDDR |= (1<<LED1_OFFSET) | (1<<LED2_OFFSET) | (1<<LED3_OFFSET) | (1<<LED4_OFFSET);

#elif defined(FREEDOM)
    PORT_BWR_PCR_MUX(PORTB, 21, 1);             // Set pin PTB21 to GPIO function
    PORT_BWR_PCR_MUX(PORTB, 22, 1);             // Set pin PTB22 to GPIO function
    PORT_BWR_PCR_MUX(PORTE, 26, 1);             // Set pin PTE26 to GPIO function

    HW_GPIO_PDDR_SET(GPIOB_BASE, 1 << 21);           // Blue LED, Negative Logic (0=on, 1=off)
    HW_GPIO_PDDR_SET(GPIOB_BASE, 1 << 22);           // Red LED, Negative Logic (0=on, 1=off)
    HW_GPIO_PDDR_SET(GPIOE_BASE, 1 << 26);           // Green LED, Negative Logic (0=on, 1=off)

#endif // FREEDOM
}

#if defined(TOWER)

static void led_toggle(uint32_t leds)
{
    if (leds & LED1)
    {
        PTD->PTOR  |= (1 << LED1_OFFSET);
    }
    if (leds & LED2)
    {
        PTD->PTOR |= (1 << LED2_OFFSET);
    }
    if (leds & LED3)
    {
        PTD->PTOR |= (1<< LED3_OFFSET);
    }
    if (leds & LED4)
    {
        PTD->PTOR |= (1<< LED3_OFFSET);
    }
        
}

#elif defined(FREEDOM)

// RGB-LED Control: 1=on, 0=off, for each of the 3 colors
void RGB(int Red,int Green,int Blue)
{
    if (Red == 1)
        GPIO_CLR_PDOR(GPIOB, 1 << 22);
    else
        HW_GPIO_PDOR_SET(GPIOB_BASE, 1 << 22);
    
    if (Green == 1)
        GPIO_CLR_PDOR(GPIOD, 1 << 26);
    else
        HW_GPIO_PDOR_SET(GPIOD_BASE, 1 << 26);
    
    if (Blue == 1)
        GPIO_CLR_PDOR(GPIOB, 1 << 21);
    else
        HW_GPIO_PDOR_SET(GPIOB_BASE, 1 << 21);
}

#endif // FREEDOM

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
    
#if defined(TOWER)
    
    uint32_t leds = LED1;
    while(1)
    {
        led_toggle(leds);
        delay();
        led_toggle(leds);
        
        leds <<= 1;
        if(leds > LED4)
        {
            leds = LED1;
        }
    }
    
#elif defined(FREEDOM)

    RGB(0,0,0);                                  // Start with all LEDs off
    
    while(1)
    {
        RGB(1,0,0);
        delay();
        RGB(0,1,0);
        delay();
        RGB(0,0,1);
        delay();
    }    
    
#endif // FREEDOM
    
}