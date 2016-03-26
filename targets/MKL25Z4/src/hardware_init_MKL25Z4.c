/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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

#include "bootloader_common.h"
#include "bootloader/context.h"
#include "device/fsl_device_registers.h"
#include "drivers/uart/uart.h"
#include "drivers/systick/systick.h"
#include "drivers/watchdog/fsl_watchdog.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "smc/smc.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#ifdef FREEDOM
#define UART0_RX_GPIO_PIN_NUM 1  // PIN 1 in the PTA group
#define UART0_RX_ALT_MODE 2      // ALT mode for UART0 functionality for pin 1
#define UART0_RX_GPIO_ALT_MODE 1 // ALT mode for GPIO functionality for pin 1

#define UART0_TX_GPIO_PIN_NUM 2  // PIN 2 in the PTA group
#define UART0_TX_ALT_MODE 2      // ALT mode for UART0 TX functionality for pin 2
#else
#define UART0_RX_GPIO_PIN_NUM 15 // PIN 15 in the PTA group
#define UART0_RX_ALT_MODE 3      // ALT mode for UART0 functionality for pin 15
#define UART0_RX_GPIO_ALT_MODE 1 // ALT mdoe for GPIO functionality for pin 15

#define UART0_TX_GPIO_PIN_NUM 14 // PIN 14 in the PTA group
#define UART0_TX_ALT_MODE 3      // ALT mode for UART0 TX functionality for pin 14
#endif

#define PORT_IRQC_INTERRUPT_FALLING_EDGE 0xA
#define PORT_IRQC_INTERRUPT_DISABLE 0

#define BOOT_PIN_NUMBER     3
#define BOOT_PIN_PORT       PORTC
#define BOOT_PIN_GPIO       PTC
#define BOOT_PIN_ALT_MODE   1
#define BOOT_PIN_DEBOUNCE_READ_COUNT 500

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt. This only supports 1 pin
static pin_irq_callback_t s_pin_irq_func = 0;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/* This function is called for configurating pinmux for uart module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void uart_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
#if BL_CONFIG_UART      
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTA, UART0_RX_GPIO_PIN_NUM, 0);
                    PORT_BWR_PCR_MUX(PORTA, UART0_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    PORT_BWR_PCR_MUX(PORTA, UART0_RX_GPIO_PIN_NUM, UART0_RX_GPIO_ALT_MODE); // Set UART0_RX pin in GPIO mode
                    GPIO_CLR_PDDR(PTA, 1 << UART0_RX_GPIO_PIN_NUM);                    // Set UART0_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    PORT_BWR_PCR_MUX(PORTA, UART0_RX_GPIO_PIN_NUM, UART0_RX_ALT_MODE);   // Set UART0_RX pin to UART0_RX functionality
                    PORT_BWR_PCR_MUX(PORTA, UART0_TX_GPIO_PIN_NUM, UART0_TX_ALT_MODE);   // Set UART0_TX pin to UART0_TX functionality
                    break;
                default:
                    break;
            }
            break;
#endif // BL_CONFIG_UART            
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

/* This function is called for configurating pinmux for i2c module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void i2c_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
#if BL_CONFIG_I2C      
        case 0:
#ifdef FREEDOM
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTC, 8, 0);
                    PORT_BWR_PCR_MUX(PORTC, 9, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    PORT_BWR_PCR_MUX(PORTC, 8, 2);  // I2C0_SCL is ALT2 for pin PTC8
                    PORT_BWR_PCR_MUX(PORTC, 9, 2);  // I2C0_SDA is ALT2 for pin PTC9
                    break;
                default:
                    break;
            }
#else
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTE, 24, 0);
                    PORT_BWR_PCR_MUX(PORTE, 25, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    PORT_BWR_PCR_MUX(PORTE, 24, 5);  // I2C0_SCL is ALT5 for pin PTE24
                    PORT_BWR_PCR_MUX(PORTE, 25, 5);  // I2C0_SDA is ALT5 for pin PTE25
                    break;
                default:
                    break;
            }
#endif
            break;
#endif // BL_CONFIG_I2C            
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

/* This function is called for configurating pinmux for spi module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void spi_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
#if BL_CONFIG_SPI      
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTD, 0, 0);
                    PORT_BWR_PCR_MUX(PORTD, 1, 0);
                    PORT_BWR_PCR_MUX(PORTD, 2, 0);
                    PORT_BWR_PCR_MUX(PORTD, 3, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI0 on PTD0~3 (not available on 32-pin QFN package)
                    PORT_BWR_PCR_MUX(PORTD, 0, 2);  // SPI0_PCS0 is ALT2 for pin PTD0
                    PORT_BWR_PCR_MUX(PORTD, 1, 2);  // SPI0_SCK is ALT2 for pin PTD1
                    PORT_BWR_PCR_MUX(PORTD, 2, 2);  // SPI0_MOSI is ALT2 for pin PTD2
                    PORT_BWR_PCR_MUX(PORTD, 3, 2);  // SPI0_MISO is ALT2 for pin PTD3
                    break;
                default:
                    break;
            }
            break;
#endif // BL_CONFIG_SPI            
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

void init_hardware(void)
{
    exit_vlpr();
    
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );

    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK // set PLLFLLSEL to select the PLL for this clock source
                | SIM_SOPT2_UART0SRC(1);   // select the PLLFLLCLK as UART0 clock source

#if DEBUG
    // Enable the pins for the debug UART1
    PORT_BWR_PCR_MUX(PORTC, 3, 3);   // UART1_RX is PTC3 in ALT3
    PORT_BWR_PCR_MUX(PORTC, 4, 3);   // UART1_TX is PTC4 in ALT3
#endif
}

void deinit_hardware(void)
{
    SIM->SCGC5 &= (uint32_t)~( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );

    // Restore SIM_SOPTx related bits being changed
    SIM_CLR_SOPT1(SIM, SIM_SOPT1_USBREGEN_MASK);
    SIM_CLR_SOPT2(SIM, SIM_SOPT2_UART0SRC_MASK | SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK);
}

uint32_t get_bus_clock(void)
{
    return SystemCoreClock / (SIM_RD_CLKDIV1_OUTDIV4(SIM) + 1);
}

// Keep this function here to ensure compatibility, all usb related configuration
// is maintained by usb stack itself.
bool usb_clock_init(void)
{
    return true;
}


uint32_t get_uart_clock( unsigned int instance )
{
    switch(instance)
    {
        case 0:
            {
                uint32_t McgOutClk = SystemCoreClock * (SIM_BRD_CLKDIV1_OUTDIV1(SIM) + 1);
                
                // if PLL/2 is the UART0 clock
                if (MCG_BRD_S_PLLST(MCG))
                {
                    return McgOutClk / 2;
                }
                else
                {
                    return McgOutClk;
                }
            }
        case 1:
            {
                // UART1 always uses the system clock / OUTDIV4
                const uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT) + 1;
                return (SystemCoreClock / busClockDivider);
            }
        case 2:
            {
                // UART2 always uses the system clock / OUTDIV4
                uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT) + 1;
                return (SystemCoreClock / busClockDivider);
            }
        default:
            return 0;
    }
}

unsigned int read_autobaud_pin( unsigned int instance )
{
    switch(instance)
    {
        case 0:
            return (GPIO_RD_PDIR(PTA) >> UART0_RX_GPIO_PIN_NUM) & 1;
        case 1:
            return 0;
        case 2:
            return 0;
        default:
            return 0;
    }
}

bool is_boot_pin_asserted(void)
{
#if defined(BL_TARGET_FLASH) & !defined(FREEDOM)
    // Initialize boot pin for GPIO
    PORT_BWR_PCR_MUX(BOOT_PIN_PORT, BOOT_PIN_NUMBER, BOOT_PIN_ALT_MODE);
    // Set boot pin as an input
    GPIO_CLR_PDDR(BOOT_PIN_GPIO, 1 << BOOT_PIN_NUMBER);
    // Set boot pin pullup enabled, pullup select, filter enable
    PORT_SET_PCR(BOOT_PIN_PORT, BOOT_PIN_NUMBER, PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_PFE_MASK);

    unsigned int readCount = 0;

    // Sample the pin a number of times
    for (unsigned int i = 0; i < BOOT_PIN_DEBOUNCE_READ_COUNT; i++)
    {
        readCount += (GPIO_RD_PDIR(BOOT_PIN_GPIO) >> BOOT_PIN_NUMBER) & 1;
    }

    // boot pin is pulled high so we are measuring lows, make sure most of our measurements
    // registered as low
    return (readCount < (BOOT_PIN_DEBOUNCE_READ_COUNT/2));
#else
    // Boot pin for Flash only target
    return false;
#endif
}

//! @brief this is going to be used for autobaud IRQ handling
void PORTA_IRQHandler(void)
{
    // Check if the pin for UART0 is what triggered the PORT A interrupt
    if (PORT_RD_PCR_ISF(PORTA, UART0_RX_GPIO_PIN_NUM) && s_pin_irq_func)
    {
        s_pin_irq_func(0);
        PORT_WR_ISFR(PORTA, ~0U);
    }
    // else if would be added here for other UART RX pins, only supports UART0 currently
}

void enable_autobaud_pin_irq(unsigned int instance, pin_irq_callback_t func)
{
    switch(instance)
    {
        case 0:
            NVIC_SetPriority(PORTA_IRQn, 1);
            NVIC_EnableIRQ(PORTA_IRQn);
            // Only look for a falling edge for our interrupts
            PORT_BWR_PCR_IRQC(PORTA, UART0_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_FALLING_EDGE);
            s_pin_irq_func = func;
            break;
    }
}

void disable_autobaud_pin_irq(unsigned int instance)
{
    switch(instance)
    {
        case 0:
            NVIC_DisableIRQ(PORTA_IRQn);
            PORT_WR_PCR_IRQC(PORTA, UART0_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_DISABLE);
            s_pin_irq_func = 0;
            break;
    }
}

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void)
{
    uart_init(UART1, get_uart_clock(1), TERMINAL_BAUD, dummy_byte_callback);
}

void update_available_peripherals()
{
}


// @brief Initialize watchdog
void bootloader_watchdog_init(void)
{
    systick_init(SystemCoreClock / 64);
    systick_set_hook(bootloader_watchdog_service);
}

void bootloader_watchdog_service(void)
{
    lock_acquire();
    fsl_watchdog_service();
    lock_release();
}

void bootloader_watchdog_deinit(void)
{
    systick_shutdown();
}



#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    while (size--)
    {
        uart_putchar(UART1, *buf++);
    }

    return size;
}



#endif // __ICCARM__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

