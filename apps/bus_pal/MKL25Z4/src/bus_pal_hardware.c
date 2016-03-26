/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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


#include "bus_pal_hardware.h"
#include "fpga_clock_registers.h"
#include "fsl_device_registers.h"
#include "i2c/master/fsl_i2c_master_driver.h"
#include "microseconds/microseconds.h"
#include "spi/fsl_spi_master_driver.h"
#include "uart/uart0.h"

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief init uart functions.
 */
static void init_uarts(void);

/*!
 * @brief dspi initialization.
 */
static void init_spi(void);

/*!
 * @brief i2c initialization.
 */
static void init_i2c(uint32_t instance);

/*!
 * @brief i2c de-initialization.
 */
static void deinit_i2c(uint32_t instance);

/*!
 * @brief uart rx callback function.
 */
static void uart_rx_callback(uint8_t byte);

/*!
 * @brief get PORT base address function.
 */
static PORT_Type * getPortBaseAddrFromAscii(uint8_t port);

/*!
 * @brief get GPIO base address function.
 */
static GPIO_Type * getGpioBaseAddrFromAscii(uint8_t port);

/*!
 * @brief fpga write clock reg function.
 */
static void write_fpga_clock_reg(uint8_t reg, uint8_t val);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Variable for SPI setup information
static spi_device_t s_spiInfo =
{
    .busFrequencyKHz = 100,
    .busClockMHz = 0,
    .polarity = kSpiClockPolarity_ActiveLow,
    .phase = kSpiClockPhase_SecondEdge,
    .direction = kSpiMsbFirst
};

//! @brief Variable for I2C setup information
static i2c_master_state_t s_i2cMasterTarget;
//! @brief Variable for I2C setup information for the FPGA
static i2c_master_state_t s_i2cMasterFPGA;

//! @brief Variable for I2C setup information
static i2c_device_t s_i2cDevice =
{
    .address = 0x10, //!< The slave's 7-bit address
    .baudRate_kbps = 100
};

//! @brief Variable for I2C setup information
static i2c_device_t s_i2cFPGADevice =
{
    .address = CY22393_ADDR, //!< The slave's 7-bit address
    .baudRate_kbps = 400
};

//! @brief Variable for host data receiving
static uint8_t* s_rxData;
static uint32_t s_bytesRx;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*FUNCTION**********************************************************************
 *
 * Function Name : get_bus_clock
 * Description   : Gets bus clock
 *
 *END**************************************************************************/
uint32_t get_bus_clock(void)
{
    uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT) + 1;
    return (SystemCoreClock / busClockDivider);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : init_hardware
 * Description   : hardware initialization
 *
 *END**************************************************************************/
void init_hardware(void)
{
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );

    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK // set PLLFLLSEL to select the PLL for this clock source
               | SIM_SOPT2_UART0SRC(1); // select the PLLFLLCLK as UART0 clock source

    // Enable the pins for the selected UART
    // Enable the UART_TXD function on PTA1
    PORTA->PCR[1] = PORT_PCR_MUX(0x2);
    // Enable the UART_TXD function on PTA2
    PORTA->PCR[2] = PORT_PCR_MUX(0x2);

    // Enable pins for UART1.
    PORT_BWR_PCR_MUX(PORTC, 3, 3);   // UART1_RX is ALT3 for pin PTC3
    PORT_BWR_PCR_MUX(PORTC, 4, 3);   // UART1_TX is ALT3 for pin PTC4

    // Enable pins for I2C0.
    PORT_BWR_PCR_MUX(PORTC, 8, 2);  // I2C0_SCL is ALT2 for pin PTC8
    PORT_BWR_PCR_MUX(PORTC, 9, 2);  // I2C0_SDA is ALT2 for pin PTC9

    // Enable pins for I2C1.
    PORT_BWR_PCR_MUX(PORTE, 0, 6);  // I2C1_SDA is ALT6 for pin PTE0
    PORT_BWR_PCR_MUX(PORTE, 1, 6);  // I2C1_SCL is ALT6 for pin PTE1

    // Enable pins for SPI0 on PTD0~3 (not available on 32-pin QFN package)
    PORT_BWR_PCR_MUX(PORTD, 0, 2);  // SPI0_PCS0 is ALT2 for pin PTD0
    PORT_BWR_PCR_MUX(PORTD, 1, 2);  // SPI0_SCK is ALT2 for pin PTD1
    PORT_BWR_PCR_MUX(PORTD, 2, 2);  // SPI0_MOSI is ALT2 for pin PTD2
    PORT_BWR_PCR_MUX(PORTD, 3, 2);  // SPI0_MISO is ALT2 for pin PTD3

    // Ungate clocks to the UART modules
    SIM_SET_SCGC4(SIM, SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK | SIM_SCGC4_UART2_MASK);

    microseconds_init();

    init_uarts();
    init_spi();
    init_i2c(I2C0_IDX);
    init_i2c(I2C1_IDX);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_uarts
 * Description   : Initialize UART ports
 *
 *END**************************************************************************/
static void init_uarts(void)
{
    // UART0 uses PLL Clock
    uart0_init(GetSystemMCGPLLClock(), 57600, uart_rx_callback);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : write_fpga_clock_reg
 * Description   : fpga clock reg write function
 *
 *END**************************************************************************/
void write_fpga_clock_reg(uint8_t reg, uint8_t val)
{
    uint8_t packet[2] = {reg, val};

    I2C_DRV_MasterSendDataBlocking(I2C0_IDX, &s_i2cFPGADevice, NULL, 0, packet, 2, 1000);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : set_fpga_clock
 * Description   : fpga clock set function
 *
 *END**************************************************************************/
void set_fpga_clock(uint32_t clock)
{
    uint32_t i;
    uint32_t P, bestP;
    uint32_t Q, bestQ;
    uint32_t realClock;
    uint32_t diff, min_diff;
    uint32_t PHigh, P0;
    uint32_t pumpVal;
    uint32_t vcoClock;
    uint32_t postDiv;

    // Get the closest we can to the max VCO clock
    vcoClock = clock * (CY22393_MAX_VCO_CLK / clock);
    // Get the post divider we will need after setting to this high clock
    postDiv = vcoClock / clock;

    // The post div value cannot be above 31 for VCO clocks above 333MHZ
    if (postDiv > CY22393_POSTDIV_MAX_VAL)
    {
        uint32_t correction = postDiv - CY22393_POSTDIV_MAX_VAL;
        vcoClock = CY22393_MAX_VCO_CLK - (clock * correction);
        postDiv = CY22393_POSTDIV_MAX_VAL;
    }

    /* http://www.cypress.com/?id=4&rID=27709 */
    /* In all for loops, if min_diff = 0, exit for loop */
    /* F = (Ref * (P/Q)) / postDiv */
    /* Ref / Q must be >= 250khz */
    /* Q range can be between 2 and 257 */
    /* P range can be between 16 and 1600 */
    /* find combination of p0, p, and q resulting in clock closest to the requested value */
    min_diff = ~0;
    bestP = 0;
    bestQ = 0;

    for (Q = 2; ((CY22393_REF_CLK / Q) >= CY22393_MIN_REF_DIV_Q) && (Q < 257) && min_diff; Q++)
    {
        for (P = 16; (P <= 1600) && min_diff; P++)
        {
            realClock = (CY22393_REF_CLK / Q ) * P;

            if (vcoClock >= realClock)
            {
                diff = vcoClock - realClock;
            }
            else
            {
                diff = realClock - vcoClock;
            }

            if (min_diff > diff)
            {
                /* a better match found */
                min_diff = diff;
                bestP = P;
                bestQ = Q;
            }

            // Since we are just increasing our multiplier in this loop if its past our desired clock
            // we can break to start increasing the quotient
            if (realClock > vcoClock)
            {
                break;
            }

        }
    }

    P0 = bestP & 1;
    PHigh = (bestP / 2) - 3;
    bestQ -= 2;

    if ((bestP >= 16) && (bestP <= 231))
    {
        pumpVal = 0;
    }
    else if ((bestP >= 232) && (bestP <= 626))
    {
        pumpVal = 1;
    }
    else if ((bestP >= 627) && (bestP <= 834))
    {
        pumpVal = 2;
    }
    else if ((bestP >= 835) && (bestP <= 1043))
    {
        pumpVal = 3;
    }
    else
    {
        pumpVal = 4;
    }

    // Clear any existing values
    for ( i = CY22393_REG_LOW; i <= CY22393_REG_HIGH; i++)
    {
        write_fpga_clock_reg(i, 0);
    }

    // Disable PLL3
    write_fpga_clock_reg(CY22393_REG_PLL3E, 0);

    // Enable Clock A
    write_fpga_clock_reg(CY22393_REG_CLKA_DIVIDE, 1);

    // Set the CLK A post divider
    write_fpga_clock_reg(CY22393_REG_CLKA_DIVIDE, postDiv);

    // Disable the other clock outputs
    write_fpga_clock_reg(CY22393_REG_CLKB_DIVIDE, CY22393_DIVIDE_OFF); // clkb
    write_fpga_clock_reg(CY22393_REG_CLKC_DIVIDE, CY22393_DIVIDE_OFF); // clkc
    write_fpga_clock_reg(CY22393_REG_CLKD_DIVIDE, CY22393_DIVIDE_OFF); // clkd

    // Set all clock sources from PLL3
    write_fpga_clock_reg(CY22393_REG_SOURCE, 0xFF);

    // Set All clocks ACAdj to nominal (b01) with pulldowns enabled and xbuf output enable
    write_fpga_clock_reg(CY22393_REG_AC, 0x5C);
    // Set all clocks to use nominal drive strength
    write_fpga_clock_reg(CY22393_REG_DC, CY22393_DC);

    write_fpga_clock_reg(CY22393_REG_PLL3P, PHigh);
    write_fpga_clock_reg(CY22393_REG_PLL3Q, bestQ);

    // B6 enables PLL3
    // B5:B3 is the pump value
    // B2 is for P0 value
    // B1:0 is for B9:8 of P
    write_fpga_clock_reg(CY22393_REG_PLL3E, (1 << 6) |
                                            (pumpVal << 3) |
                                            (P0 << 2) |
                                            ((PHigh & 0x300) >> 8) );
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_spi
 * Description   : spi init function
 *
 *END**************************************************************************/

void init_spi(void)
{
    s_spiInfo.busClockMHz = get_bus_clock();
    spi_master_init(SPI0_IDX, &s_spiInfo);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_i2c
 * Description   : I2C init function
 *
 *END**************************************************************************/

void init_i2c(uint32_t instance)
{
    I2C_DRV_MasterInit(instance, &s_i2cMasterTarget);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : deinit_i2c
 * Description   : I2C de-init function
 *
 *END**************************************************************************/
void deinit_i2c(uint32_t instance)
{
    I2C_DRV_MasterDeinit(instance);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : host_start_command_rx
 * Description   : receiving host start command process
 *
 *END**************************************************************************/
void host_start_command_rx(uint8_t * dest, uint32_t length)
{
    s_rxData = dest;
    s_bytesRx = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : host_stop_command_rx
 * Description   : receiving host stop command process
 *
 *END**************************************************************************/
void host_stop_command_rx(void)
{
   s_rxData = 0;
   s_bytesRx = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_bytes_received_from_host
 * Description   : receiving host get bytes command process
 *
 *END**************************************************************************/
uint32_t get_bytes_received_from_host(void)
{
    return s_bytesRx;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : write_bytes_to_host
 * Description   : sending host bytes command process
 *
 *END**************************************************************************/
void write_bytes_to_host(uint8_t * src, uint32_t length)
{
    unsigned int i;

    for (i = 0; i < length; i++)
    {
        uart0_putchar(src[i]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_i2c_address
 * Description   : i2c config address process
 *
 *END**************************************************************************/
void configure_i2c_address(uint8_t address)
{
    s_i2cDevice.address = address;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_i2c_speed
 * Description   : i2c config speed process
 *
 *END**************************************************************************/
void configure_i2c_speed(unsigned int speedkhz)
{
    s_i2cDevice.baudRate_kbps = speedkhz;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : send_spi_data
 * Description   : spi send data proces
 *
 *END**************************************************************************/
void send_spi_data(uint8_t * src, uint32_t writeLength)
{
    spi_master_transfer(SPI0_IDX, &s_spiInfo, src, 0, writeLength, kSyncWaitForever);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_spi_data
 * Description   : spi receiving data process
 *
 *END**************************************************************************/
void receive_spi_data(uint8_t * dest, uint32_t readLength)
{
    spi_master_transfer(SPI0_IDX, &s_spiInfo, 0, dest, readLength, kSyncWaitForever);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : configure_spi_speed
 * Description   : spi config speed process
 *
 *END**************************************************************************/
void configure_spi_speed(unsigned int speedkhz)
{
    s_spiInfo.busFrequencyKHz = speedkhz;
    spi_master_configure_bus(SPI0_IDX, &s_spiInfo);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_spi_settings
 * Description   : spi config settings process
 *
 *END**************************************************************************/
void configure_spi_settings(spi_clock_polarity_t polarity, spi_clock_phase_t phase, spi_shift_direction_t direction)
{
    s_spiInfo.polarity = polarity;
    s_spiInfo.phase = phase;
    s_spiInfo.direction = direction;

    spi_master_configure_bus(SPI0_IDX, &s_spiInfo);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : send_i2c_data
 * Description   : i2c sending data process
 *
 *END**************************************************************************/
status_t send_i2c_data(uint8_t * src, uint32_t writeLength)
{
    i2c_status_t status = I2C_DRV_MasterSendDataBlocking(I2C0_IDX, &s_i2cDevice, NULL, 0, src, writeLength, 5000);

    if ((status == kStatus_I2C_Timeout) ||
        (status == kStatus_I2C_BusBusy) ||
        (status == kStatus_I2C_ReceivedNak))
    {
        deinit_i2c(I2C0_IDX);
        init_i2c(I2C0_IDX);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_i2c_data
 * Description   : i2c receiving data process
 *
 *END**************************************************************************/
status_t receive_i2c_data(uint8_t * dest, uint32_t readLength)
{
    i2c_status_t status = I2C_DRV_MasterReceiveDataBlocking(I2C0_IDX, &s_i2cDevice, NULL, 0, dest, readLength, 5000);

    if ((status == kStatus_I2C_Timeout) || (status == kStatus_I2C_BusBusy))
    {
        deinit_i2c(I2C0_IDX);
        init_i2c(I2C0_IDX);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_rx_callback
 * Description   : uart callback function
 *
 *END**************************************************************************/
void uart_rx_callback(uint8_t byte)
{
    if (s_rxData)
    {
        s_rxData[s_bytesRx++] = byte;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : getPortBaseAddrFromAscii
 * Description   : PORT get base address function
 *
 *END**************************************************************************/
PORT_Type * getPortBaseAddrFromAscii(uint8_t port)
{
    if ((port >= 'a') && (port <='e'))
    {
        port = port - 'a';
    }
    else if ((port >= 'A') && (port <='E'))
    {
        port = port - 'A';
    }

    switch(port)
    {
        default:
        case PORTA_IDX:
            return PORTA;
        case PORTB_IDX:
            return PORTB;
        case PORTC_IDX:
            return PORTC;
        case PORTD_IDX:
            return PORTD;
        case PORTE_IDX:
            return PORTE;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : getGpioBaseAddrFromAscii
 * Description   : GPIO get base address function
 *
 *END**************************************************************************/
GPIO_Type * getGpioBaseAddrFromAscii(uint8_t port)
{
    if ((port >= 'a') && (port <='e'))
    {
        port = port - 'a';
    }
    else if ((port >= 'A') && (port <='E'))
    {
        port = port - 'A';
    }

    switch(port)
    {
        default:
        case GPIOA_IDX:
            return GPIOA;
        case GPIOB_IDX:
            return GPIOB;
        case GPIOC_IDX:
            return GPIOC;
        case GPIOD_IDX:
            return GPIOD;
        case GPIOE_IDX:
            return GPIOE;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_gpio
 * Description   : GPIO config processing
 *
 *END**************************************************************************/
void configure_gpio(uint8_t port, uint8_t pinNum, uint8_t muxVal)
{
    PORT_Type * realPort = getPortBaseAddrFromAscii(port);
    PORT_BWR_PCR_MUX(realPort, pinNum, muxVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : set_gpio
 * Description   : GPIO set up function
 *
 *END**************************************************************************/
void set_gpio(uint8_t port, uint8_t pinNum, uint8_t level)
{
    GPIO_Type * realPort = getGpioBaseAddrFromAscii(port);

    GPIO_SET_PDDR(realPort, 1 << pinNum); // Set pin as an output

    if (level)
    {
        GPIO_WR_PSOR(realPort, 1 << pinNum);
    }
    else
    {
        GPIO_WR_PCOR(realPort, 1 << pinNum);
    }
}

#if __ICCARM__
/*FUNCTION**********************************************************************
 *
 * Function Name : __write
 * Description   : ICCARM write function implementation
 *
 *END**************************************************************************/
size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

