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
#include "utilities/kinetis_family.h"
#include "smc/smc.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#ifdef TOWER
#define UART2_RX_GPIO_PIN_NUM 17  // PTE17
#define UART2_RX_ALT_MODE 3      // ALT mode 3 for UART1 RX functionality
#define UART2_RX_GPIO_ALT_MODE 1 // ALT mode 1 for GPIO functionality

#define UART2_TX_GPIO_PIN_NUM 16  // PTE16
#define UART2_TX_ALT_MODE 3      // ALT mode 3 for UART1 TX functionality
#else
#define UART4_RX_GPIO_PIN_NUM 25  // PTE25
#define UART4_RX_ALT_MODE 3      // ALT mode 3 for UART1 RX functionality
#define UART4_RX_GPIO_ALT_MODE 1 // ALT mode 1 for GPIO functionality

#define UART4_TX_GPIO_PIN_NUM 24  // PTE24
#define UART4_TX_ALT_MODE 3      // ALT mode 3 for UART1 TX functionality
#endif // defined TOWER

#define PORT_IRQC_INTERRUPT_FALLING_EDGE 0xA
#define PORT_IRQC_INTERRUPT_DISABLE 0

#ifdef TOWER
#define BOOT_PIN_NUMBER     10
#define BOOT_PIN_PORT       PORTA
#define BOOT_PIN_GPIO       PTA
#define BOOT_PIN_ALT_MODE   1
#endif

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

/* This function is called for configuring pinmux for uart module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void uart_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
#if BL_CONFIG_SCUART  
#ifdef TOWER      
        case 2:
           switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTE, UART2_RX_GPIO_PIN_NUM, 0);
                    PORT_BWR_PCR_MUX(PORTE, UART2_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    PORT_BWR_PCR_MUX(PORTE, UART2_RX_GPIO_PIN_NUM, UART2_RX_GPIO_ALT_MODE); 
                    GPIO_CLR_PDDR(PTE, 1 << UART2_RX_GPIO_PIN_NUM);                      
                    break;
                case kPinmuxType_Peripheral:
                    PORT_BWR_PCR_MUX(PORTE, UART2_RX_GPIO_PIN_NUM, UART2_RX_ALT_MODE);  
                    PORT_BWR_PCR_MUX(PORTE, UART2_TX_GPIO_PIN_NUM, UART2_TX_ALT_MODE);  
                    break;      
                default:
                    break;    
            }
            break;  
#else // defined TOWER           
        case 4:
           switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTE, UART4_RX_GPIO_PIN_NUM, 0);
                    PORT_BWR_PCR_MUX(PORTE, UART4_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    PORT_BWR_PCR_MUX(PORTE, UART4_RX_GPIO_PIN_NUM, UART4_RX_GPIO_ALT_MODE); 
                    GPIO_CLR_PDDR(PTE, 1 << UART4_RX_GPIO_PIN_NUM);                      
                    break;
                case kPinmuxType_Peripheral:
                    PORT_BWR_PCR_MUX(PORTE, UART4_RX_GPIO_PIN_NUM, UART4_RX_ALT_MODE);  
                    PORT_BWR_PCR_MUX(PORTE, UART4_TX_GPIO_PIN_NUM, UART4_TX_ALT_MODE);  
                    break;      
                default:
                    break;    
            }
            break;  
#endif // defined TOWER           
#endif // BL_CONFIG_SCUART           
        default:
            break;
    }
}

/* This function is called for configuring pinmux for i2c module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void i2c_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
#if BL_CONFIG_I2C      
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTD, 8, 0);
                    PORT_BWR_PCR_ODE(PORTD, 8, 0);  // I2C0_SCL set for open drain
                    PORT_BWR_PCR_MUX(PORTD, 9, 0);
                    PORT_BWR_PCR_ODE(PORTD, 9, 0);  // I2C0_SDA set for open drain
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    PORT_BWR_PCR_MUX(PORTD, 8, 2);  // I2C0_SCL is ALT5 for pin PTE24
                    PORT_BWR_PCR_ODE(PORTD, 8, 1); // I2C0_SCL set for open drain
                    PORT_BWR_PCR_MUX(PORTD, 9, 2);  // I2C0_SDA is ALT5 for pin PTE25
                    PORT_BWR_PCR_ODE(PORTD, 9, 1); // I2C0_SDA set for open drain
                    break;
                default:
                    break;
            }          
            break;
#endif // BL_CONFIG_I2C            
        default:
            break;
    }
}

/* This function is called for configuring pinmux for spi module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void spi_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
#if BL_CONFIG_DSPI
        case 2:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    PORT_BWR_PCR_MUX(PORTD, 11, 0);
                    PORT_BWR_PCR_MUX(PORTD, 12, 0);
                    PORT_BWR_PCR_MUX(PORTD, 13, 0);
                    PORT_BWR_PCR_MUX(PORTD, 14, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI0 on PTD11~14
                    PORT_BWR_PCR_MUX(PORTD, 11, 2);  // SPI0_PCS0 is ALT2 for pin PTD11
                    PORT_BWR_PCR_MUX(PORTD, 12, 2);  // SPI0_SCK is ALT2 for pin PTD12
                    PORT_BWR_PCR_MUX(PORTD, 13, 2);  // SPI0_SOUT is ALT2 for pin PTD13
                    PORT_BWR_PCR_MUX(PORTD, 14, 2);  // SPI0_SIN is ALT2 for pin PTD14
                    break;
                default:
                    break;
            }
            break;
#endif // BL_CONFIG_DSPI
        default:
            break;
    }
}

void init_hardware(void)
{
    exit_vlpr();
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
    SIM_CLR_SOPT2(SIM, SIM_SOPT2_PLLFLLSEL_MASK);
}

// Note: Keep this function here to ensure compatibility, all usb related clock 
// configuration is maintained by USB stack itself.
bool usb_clock_init(void)
{
    return true;
}

uint32_t get_bus_clock(void)
{
    uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1;
    return (SystemCoreClock / busClockDivider);
}

// Return specified UART clock
// Note: LPUART0 is not supported here.
uint32_t get_uart_clock( unsigned int instance )
{
    switch(instance)
    {
    case 0:
    case 1:
        // UART0 and UART1 always use the system clock
        return SystemCoreClock;
    case 2:
        return get_bus_clock();
    case 3:
    case 4:
        // UART2-4 always use the bus clock.
        return get_bus_clock();
    default:
        return 0;
    }
}

unsigned int read_autobaud_pin( unsigned int instance )
{
    switch(instance)
    {
#ifdef TOWER      
         case 2:
            return (GPIO_RD_PDIR(PTE) >> UART2_RX_GPIO_PIN_NUM) & 1;
            break;  
#else            
        case 4:
            return (GPIO_RD_PDIR(PTE) >> UART4_RX_GPIO_PIN_NUM) & 1;
            break;    
#endif // Deifned TOWER            
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

//! @brief this is going to be used for autobaud IRQ handling for UART5
void PORTE_IRQHandler(void)
{
    // Check if the pin for UART5 is what triggered the PORT E interrupt
#ifdef TOWER  
    if (PORT_RD_PCR_ISF(PORTE, UART2_RX_GPIO_PIN_NUM) && s_pin_irq_func[0])      
    {
        s_pin_irq_func[0](2);  
        PORT_WR_ISFR(PORTE, ~0U);
    }  
#else    
    if (PORT_RD_PCR_ISF(PORTE, UART4_RX_GPIO_PIN_NUM) && s_pin_irq_func[0])      
    {
        s_pin_irq_func[0](4);  
        PORT_WR_ISFR(PORTE, ~0U);
    }
#endif // defined TOWER    
}

void enable_autobaud_pin_irq(unsigned int instance, pin_irq_callback_t func)
{
    switch(instance)
    {
#ifdef TOWER      
        case 2:
            NVIC_SetPriority(PORTE_IRQn, 1);
            NVIC_EnableIRQ(PORTE_IRQn);
            // Only look for a falling edge for our interrupts
            PORT_BWR_PCR_IRQC(PORTE, UART2_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_FALLING_EDGE);
            s_pin_irq_func[0] = func;
            break;   
#else            
        case 4:
            NVIC_SetPriority(PORTE_IRQn, 1);
            NVIC_EnableIRQ(PORTE_IRQn);
            // Only look for a falling edge for our interrupts
            PORT_BWR_PCR_IRQC(PORTE, UART4_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_FALLING_EDGE);
            s_pin_irq_func[0] = func;
            break;  
#endif // defiend TOWER            
    }
}

void disable_autobaud_pin_irq(unsigned int instance)
{
    switch(instance)
    {
#ifdef TOWER      
        case 2:
            NVIC_DisableIRQ(PORTE_IRQn);
            PORT_BWR_PCR_IRQC(PORTE, UART2_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_DISABLE);
            s_pin_irq_func[0] = 0;
            break;  
#else            
        case 4:
            NVIC_DisableIRQ(PORTE_IRQn);
            PORT_BWR_PCR_IRQC(PORTE, UART4_RX_GPIO_PIN_NUM, PORT_IRQC_INTERRUPT_DISABLE);
            s_pin_irq_func[0] = 0;
            break;
#endif // defined TOWER    
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

