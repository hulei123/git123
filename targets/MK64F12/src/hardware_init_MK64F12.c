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
#include "drivers/uart/scuart.h"
#include "smc/smc.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#if defined(TOWER)
#define UART1_RX_GPIO_PIN_NUM 3  // PIN 3 in the PTC group
#define UART1_RX_ALT_MODE 3       // ALT mode for UART1 functionality for pin 3
#define UART1_RX_GPIO_ALT_MODE 1  // ALT mdoe for GPIO functionality for pin 3

#define UART1_TX_GPIO_PIN_NUM 4  // PIN 3 in the PTC group
#define UART1_TX_ALT_MODE 3       // ALT mode for UART1 TX functionality for pin 4
#else
#define UART0_RX_GPIO_PIN_NUM 16  // PIN 16 in the PTB group
#define UART0_RX_ALT_MODE 3       // ALT mode for UART0 functionality for pin 16
#define UART0_RX_GPIO_ALT_MODE 1  // ALT mdoe for GPIO functionality for pin 16

#define UART0_TX_GPIO_PIN_NUM 17  // PIN 17 in the PTB group
#define UART0_TX_ALT_MODE 3       // ALT mode for UART0 TX functionality for pin 17
#endif  // defined TOWER

#define PORT_IRQC_INTERRUPT_FALLING_EDGE 0xA
#define PORT_IRQC_INTERRUPT_DISABLE 0

#define BOOT_PIN_NUMBER     6
#define BOOT_PIN_PORT       PORTC
#define BOOT_PIN_GPIO       PTC
#define BOOT_PIN_ALT_MODE   1
#define BOOT_PIN_DEBOUNCE_READ_COUNT 500

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt.
static pin_irq_callback_t s_pin_irq_func[UART_INSTANCE_COUNT] = {0};

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
#if BL_CONFIG_SCUART   
#if !defined(TOWER)      
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTB, UART0_RX_GPIO_PIN_NUM, 0);
                    PORT_BWR_PCR_MUX(PORTB, UART0_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    PORT_BWR_PCR_MUX(PORTB, UART0_RX_GPIO_PIN_NUM, UART0_RX_GPIO_ALT_MODE); // Set UART0_RX pin in GPIO mode
                    GPIO_CLR_PDDR(PTB, 1 << UART0_RX_GPIO_PIN_NUM);                    // Set UART0_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    PORT_BWR_PCR_MUX(PORTB, UART0_RX_GPIO_PIN_NUM, UART0_RX_ALT_MODE);   // Set UART0_RX pin to UART0_RX functionality
                    PORT_BWR_PCR_MUX(PORTB, UART0_TX_GPIO_PIN_NUM, UART0_TX_ALT_MODE);   // Set UART0_TX pin to UART0_TX functionality
                    break;
                default:
                    break;
            }
            break;
#endif // !defined TOWER            
        case 1:
#if defined(TOWER) 
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTC, UART1_RX_GPIO_PIN_NUM, 0);
                    PORT_BWR_PCR_MUX(PORTC, UART1_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    PORT_BWR_PCR_MUX(PORTC, UART1_RX_GPIO_PIN_NUM, UART1_RX_GPIO_ALT_MODE); // Set UART1_RX pin in GPIO mode
                    GPIO_CLR_PDDR(PTC, 1 << UART1_RX_GPIO_PIN_NUM);                    // Set UART1_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    PORT_BWR_PCR_MUX(PORTC, UART1_RX_GPIO_PIN_NUM, UART1_RX_ALT_MODE);   // Set UART1_RX pin to UART1_RX functionality
                    PORT_BWR_PCR_MUX(PORTC, UART1_TX_GPIO_PIN_NUM, UART1_TX_ALT_MODE);   // Set UART1_TX pin to UART1_TX functionality
                    break;
                default:
                    break;
            }
            break;
#endif   // defined TOWER
#endif // BL_CONFIG_SCUART            
        case 2:
        case 3:
        case 4:
        case 5:
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
        case 0:
            break;
#if BL_CONFIG_I2C            
        case 1:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTC, 10, 0);
                    PORT_BWR_PCR_MUX(PORTC, 11, 0);
                    PORT_BWR_PCR_ODE(PORTC, 10, 0);
                    PORT_BWR_PCR_ODE(PORTC, 11, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C1.
                    PORT_BWR_PCR_MUX(PORTC, 10, 2);  // I2C1_SCL is ALT2 for pin PTC10
                    PORT_BWR_PCR_ODE(PORTC, 10, 1);  // I2C1_SCL set for open drain
                    PORT_BWR_PCR_MUX(PORTC, 11, 2);  // I2C1_SDA is ALT2 for pin PTC11
                    PORT_BWR_PCR_ODE(PORTC, 11, 1);  // I2C1_SDA set for open drain
                    break;
                default:
                    break;
            }
            break;
#endif // BL_CONFIG_I2C            
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
#if BL_CONFIG_DSPI      
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
                    // Enable pins for SPI0 on PTD0~3
                    PORT_BWR_PCR_MUX(PORTD, 0, 2);  // SPI0_PCS0 is ALT2 for pin PTD0
                    PORT_BWR_PCR_MUX(PORTD, 1, 2);  // SPI0_SCK is ALT2 for pin PTD1
                    PORT_BWR_PCR_MUX(PORTD, 2, 2);  // SPI0_SOUT is ALT2 for pin PTD2
                    PORT_BWR_PCR_MUX(PORTD, 3, 2);  // SPI0_SIN is ALT2 for pin PTD3
                    break;
                default:
                    break;
            }
            break;
#endif // BL_CONFIG_DSPI            
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
    
    // Disable the MPU otherwise USB cannot access the bus
    MPU->CESR = 0;

    // Enable all the ports
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );
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
    SIM_CLR_SOPT2(SIM, SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK);
}

bool usb_clock_init(void)
{
    // Select PLL clock
    SIM_SOPT2 |= (SIM_SOPT2_USBSRC_MASK);

    // Enable USB-OTG IP clocking
    SIM_SCGC4 |= (SIM_SCGC4_USBOTG_MASK);

    // Configure enable USB regulator for device
    SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

    return true;
}

uint32_t get_bus_clock(void)
{
    uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1;
    return (SystemCoreClock / busClockDivider);
}

uint32_t get_uart_clock( unsigned int instance )
{
    switch(instance)
    {
        case 0:
        case 1:
            // UART0 and UART1 always use the system clock
            return SystemCoreClock;
        case 2:
        case 3:
        case 4:
        case 5:
            // UART2, UART3, UART4, and UART5 always use the bus clock
            return get_bus_clock();
        default:
            return 0;
    }
}

unsigned int read_autobaud_pin( unsigned int instance )
{
    switch(instance)
    {
#if !defined(TOWER)      
        case 0:
            return (GPIO_RD_PDIR(PTB) >> UART0_RX_GPIO_PIN_NUM) & 1;
#else 
        case 1:
            return (GPIO_RD_PDIR(PTC) >> UART1_RX_GPIO_PIN_NUM) & 1;
#endif // #if !defined(TOWER)
        case 2:
        case 3:
        case 4:
        case 5:
        default:
            return 0;
    }
}

bool is_boot_pin_asserted(void)
{
#ifdef BL_TARGET_FLASH
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

#if !defined(TOWER)
//! @brief this is going to be used for autobaud IRQ handling for UART0
void PORTB_IRQHandler(void)
{
    // Check if the pin for UART1 is what triggered the PORT C interrupt
    if (PORT_RD_PCR_ISF(PORTB, UART0_RX_GPIO_PIN_NUM) && s_pin_irq_func[0])
    {
        s_pin_irq_func[0](0);
        PORT_WR_ISFR(PORTB, ~0U);
    }
}
#endif 

#if defined(TOWER)
void PORTC_IRQHandler(void)
{
    // Check if the pin for UART1 is what triggered the PORT C interrupt
    if (PORT_RD_PCR_ISF(PORTC, UART1_RX_GPIO_PIN_NUM) && s_pin_irq_func[1])
    {
        s_pin_irq_func[1](1);
        PORT_WR_ISFR(PORTC, ~0U);
    }
}
#endif

void enable_autobaud_pin_irq(unsigned int instance, pin_irq_callback_t func)
{
    switch(instance)
    {
#if !defined(TOWER)      
        case 0:
            NVIC_SetPriority(PORTB_IRQn, 1);
            NVIC_EnableIRQ(PORTB_IRQn);
            // Only look for a falling edge for our interrupts
            PORT_BWR_PCR_IRQC(PORTB, UART0_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_FALLING_EDGE);
            s_pin_irq_func[0] = func;
            break;
#else 
        case 1:
            NVIC_SetPriority(PORTC_IRQn, 1);
            NVIC_EnableIRQ(PORTC_IRQn);
            // Only look for a falling edge for our interrupts
            PORT_BWR_PCR_IRQC(PORTC, UART1_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_FALLING_EDGE);
            s_pin_irq_func[1] = func;
            break;
#endif  // #if !defined(TOWER)
    }
}

void disable_autobaud_pin_irq(unsigned int instance)
{
    switch(instance)
    {
#if !defined(TOWER)      
        case 0:
            NVIC_DisableIRQ(PORTB_IRQn);
            PORT_BWR_PCR_IRQC(PORTB, UART0_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_DISABLE);
            s_pin_irq_func[0] = 0;
            break;
#else
        case 1:
            NVIC_DisableIRQ(PORTC_IRQn);
            PORT_BWR_PCR_IRQC(PORTC, UART1_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_DISABLE);
            s_pin_irq_func[1] = 0;
            break;
#endif
    }
}

void debug_init(void)
{
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

void update_available_peripherals()
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

