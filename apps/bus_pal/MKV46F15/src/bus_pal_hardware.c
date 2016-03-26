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
#include "fsl_dspi_master_driver.h"
#include "flexcan/fsl_flexcan_driver.h"
#include "uart/scuart.h"

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief init suart functions.
 */
static void init_scuarts(void);

/*!
 * @brief dspi initialization.
 */
static void init_dspi(void);

/*!
 * @brief flexcan initialization.
 */
static void init_flexcan(void);

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

#define FLEXCAN_USE_OSC_8M_CLOCK     1

#if FLEXCAN_USE_OSC_8M_CLOCK
/*!
 * @brief flexCAN timing table (8Mhz)contains propseg, pseg1, pseg2, pre_divider, rjw
 */
flexcan_time_segment_t bit_rate_table[] = {
    { 0, 2, 2,  7, 2},  /* 125 kHz */
    { 0, 2, 2,  3, 2},  /* 250 kHz */
    { 2, 1, 1,  1, 1},  /* 500 kHz */
    { 2, 2, 2,  0, 2},  /* 750 kHz */
    { 2, 1, 1,  0, 1}   /* 1   MHz */
};

#else
/*!
 * @brief flexCAN timing table (20.9715Mhz)contains propseg, pseg1, pseg2, pre_divider, rjw
 */
flexcan_time_segment_t bit_rate_table_backup[] = {
    { 5, 5, 7,  7, 3},  /* 125 Khz */
    { 5, 5, 7,  3, 3},  /* 250 Khz */
    { 5, 5, 7,  1, 3},  /* 500 Khz */
    { 3, 3, 5,  1, 3},  /* 750 KHz */
    { 5, 5, 7,  0, 3}   /* 1   MHz */
};
flexcan_time_segment_t bit_rate_table[] = {
    { 3, 7, 7,  7, 3},  /* 125 kHz */
    { 3, 7, 7,  3, 3},  /* 250 kHz */
    { 3, 7, 7,  1, 3},  /* 500 kHz */
    { 5, 3, 3,  1, 3},  /* 750 kHz */
    { 7, 5, 5,  0, 3}   /* 1   MHz */
};
#endif

/*!
 * @brief flexCAN common information structure
 */
typedef struct _flexcan_transfer_info {
    flexcan_state_t     state;          //!< state
    uint32_t            rx_id;          //!< rx id
    uint32_t            tx_id;          //!< tx id
    uint32_t            rx_mailbox_num; //!< rx mb number
    uint32_t            tx_mailbox_num; //!< tx mb number
    flexcan_data_info_t rx_info;        //!< tx info
    flexcan_data_info_t tx_info;        //!< tx info
    uint8_t             rx_buf[64];     //!< rx buffer 
    uint8_t             rx_buf_write_index;   //!< rx buffer write index 
    uint8_t             rx_buf_read_index;    //!< rx buffer read index 
} flexcan_transfer_info_t;
    
/*!
 * @brief flexCAN instance used for bus pal
 */
uint8_t flexcanInstance = 0;

//! @brief Global state for the FlexCAN peripheral interface.
flexcan_transfer_info_t s_flexcanInfo;

//! @brief FlexCAN config data
flexcan_info_t s_flexcanConfig =
{
    .data_sink = receive_can_data,
};

/*!
 * @brief DSPI variables
 */
static dspi_master_state_t g_dspiState;
static dspi_device_t g_dspiDevice;
uint32_t    g_calculatedBaudRate;

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
 * Function Name : get_fast_peripheral_clock
 * Description   : fast peripheral clock
 *
 *END**************************************************************************/
uint32_t get_fast_peripheral_clock(void)
{
    uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1;
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

    // Enable pins for UART1 on PTE0 - PTE1.
    PORT_BWR_PCR_MUX(PORTE, 1, 3);   // UART1_RX is ALT3 for pin PTC3
    PORT_BWR_PCR_MUX(PORTE, 0, 3);   // UART1_TX is ALT3 for pin PTC4

    // Enable pins for I2C0 on PTC14 - PTC15.
    PORT_BWR_PCR_MUX(PORTC, 14, 3);  // I2C0_SCL is ALT3 for pin PTC14
    PORT_BWR_PCR_ODE(PORTC, 14, 1);  // I2C0_SCL set for open drain
    PORT_BWR_PCR_MUX(PORTC, 15, 3);  // I2C0_SDA is ALT3 for pin PTC15
    PORT_BWR_PCR_ODE(PORTC, 15, 1);  // I2C0_SDA set for open drain

    // Enable pins for SPI0 on PTA14 - PTA17. 
    PORT_BWR_PCR_MUX(PORTA, 14, 2);  // SPI0_PCS0 is ALT2 for pin PTA14
    PORT_BWR_PCR_MUX(PORTA, 15, 2);  // SPI0_SCK  is ALT2 for pin PTA15
    PORT_BWR_PCR_MUX(PORTA, 16, 2);  // SPI0_SOUT is ALT2 for pin PTA16
    PORT_BWR_PCR_MUX(PORTA, 17, 2);  // SPI0_SIN  is ALT2 for pin PTA17

    // Enable pins for FLEXCAN0 on PTA12 - PTA13. 
    PORT_BWR_PCR_MUX(PORTA, 12, 2);  // FLEXCAN0_TX is ALT2 for pin PTA12
    PORT_BWR_PCR_MUX(PORTA, 13, 2);  // FLEXCAN0_RX is ALT2 for pin PTA13
  
    // Ungate clocks to the UART modules
    SIM_SET_SCGC4(SIM, SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK);

    microseconds_init();

    init_scuarts();
    init_dspi();
    init_flexcan();
    init_i2c(I2C0_IDX);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_scuarts
 * Description   : Initialize UART ports
 *
 *END**************************************************************************/
static void init_scuarts(void)
{
    // UART1 
    scuart_init(UART1, get_fast_peripheral_clock(), 57600, uart_rx_callback);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_dspi
 * Description   : Initialize dspi
 *
 *END**************************************************************************/
static void init_dspi(void)
{
    dspi_master_user_config_t dspiConfig;

    dspiConfig.isChipSelectContinuous = false;
    dspiConfig.isSckContinuous = false;
    dspiConfig.pcsPolarity = kDspiPcs_ActiveLow;
    dspiConfig.whichCtar = kDspiCtar1;
    dspiConfig.whichPcs = kDspiPcs0;

    DSPI_DRV_MasterInit(0, &g_dspiState, &dspiConfig);

    g_dspiDevice.dataBusConfig.bitsPerFrame = 8;
    g_dspiDevice.dataBusConfig.clkPhase = kDspiClockPhase_SecondEdge;
    g_dspiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveLow;
    g_dspiDevice.dataBusConfig.direction = kDspiMsbFirst;
    g_dspiDevice.bitsPerSec = 1000000;

    DSPI_DRV_MasterConfigureBus(0, &g_dspiDevice, &g_calculatedBaudRate);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_flexcan
 * Description   : Initialize flexCAN
 *
 *END**************************************************************************/
static void init_flexcan(void)
{
    flexcan_user_config_t flexcan1_data;
    uint32_t i;
    
    flexcan1_data.max_num_mb = 16;
    flexcan1_data.num_id_filters = kFlexCanRxFifoIDFilters_8;
    flexcan1_data.is_rx_fifo_needed = false; //disable fifo here
    
    /* Select mailbox number */
    s_flexcanInfo.rx_mailbox_num = 8;
    s_flexcanInfo.tx_mailbox_num = 9;

    /* make bootloader as node 'b' for testing */
    s_flexcanInfo.rx_id = 0x123;
    s_flexcanInfo.tx_id = 0x321;

    s_flexcanInfo.rx_buf_write_index = 0;    
    s_flexcanInfo.rx_buf_read_index = 0;    
    
#if FLEXCAN_USE_OSC_8M_CLOCK
    FLEXCAN_DRV_Init(flexcanInstance, &flexcan1_data, true, &s_flexcanInfo.state, 
                     &s_flexcanConfig, kFlexCanClkSource_Osc);
#else
    FLEXCAN_DRV_Init(flexcanInstance, &flexcan1_data, true, &s_flexcanInfo.state, 
                     &s_flexcanConfig, kFlexCanClkSource_Ipbus);
#endif
      
    FLEXCAN_DRV_SetBitrate(flexcanInstance, &bit_rate_table[0]); /* 125KHz */
    
    FLEXCAN_DRV_SetMaskType(flexcanInstance, kFlexCanRxMask_Global);

    FLEXCAN_DRV_SetRxMbGlobalMask(flexcanInstance, kFlexCanMbId_Std, 0xffffffff);
    
    for (i = 0; i < flexcan1_data.max_num_mb; i++)
    {
        FLEXCAN_DRV_SetRxIndividualMask(flexcanInstance, kFlexCanMbId_Std, i, 0xffffffff);    
    }

    // FlexCAN reveive config
    s_flexcanInfo.rx_info.msg_id_type = kFlexCanMbId_Std;
    s_flexcanInfo.rx_info.data_length = 8;

    // Configure RX MB fields
    FLEXCAN_DRV_ConfigRxMb(flexcanInstance, s_flexcanInfo.rx_mailbox_num, &s_flexcanInfo.rx_info, s_flexcanInfo.rx_id);

    FLEXCAN_DRV_EnableMbInt(flexcanInstance, s_flexcanInfo.rx_mailbox_num);    
    
    // FlexCAN transfer config
    s_flexcanInfo.tx_info.msg_id_type = kFlexCanMbId_Std;
    s_flexcanInfo.tx_info.data_length = 8;

    FLEXCAN_DRV_ConfigTxMb(flexcanInstance, s_flexcanInfo.tx_mailbox_num, &s_flexcanInfo.tx_info, s_flexcanInfo.tx_id);
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
        scuart_putchar(UART1, src[i]);
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
    DSPI_DRV_MasterTransferBlocking(0, NULL, src, NULL, writeLength, kSyncWaitForever);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_spi_data
 * Description   : spi receiving data process
 *
 *END**************************************************************************/
void receive_spi_data(uint8_t * dest, uint32_t readLength)
{
    DSPI_DRV_MasterTransferBlocking(0, NULL, NULL, dest, readLength, kSyncWaitForever);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_spi_speed
 * Description   : spi config speed process
 *
 *END**************************************************************************/
void configure_spi_speed(unsigned int speedkhz)
{
    g_dspiDevice.bitsPerSec = speedkhz*1000;

    DSPI_DRV_MasterConfigureBus(0, &g_dspiDevice, &g_calculatedBaudRate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_spi_settings
 * Description   : spi config settings process
 *
 *END**************************************************************************/
void configure_spi_settings(dspi_clock_polarity_t polarity, dspi_clock_phase_t phase, dspi_shift_direction_t direction)
{
    g_dspiDevice.dataBusConfig.clkPhase = phase;
    g_dspiDevice.dataBusConfig.clkPolarity = polarity;
    g_dspiDevice.dataBusConfig.direction = direction;

    DSPI_DRV_MasterConfigureBus(0, &g_dspiDevice, &g_calculatedBaudRate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_can_txid
 * Description   : flexCAN config tx id
 *
 *END**************************************************************************/
void configure_can_txid(unsigned int txid)
{
    s_flexcanInfo.tx_id = txid & 0x7ff;  // support 11 bit std id
    FLEXCAN_DRV_ConfigTxMb(flexcanInstance, s_flexcanInfo.tx_mailbox_num, &s_flexcanInfo.tx_info, s_flexcanInfo.tx_id);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_can_rxid
 * Description   : flexCAN config rx id
 *
 *END**************************************************************************/
void configure_can_rxid(unsigned int rxid)
{
    s_flexcanInfo.rx_id = rxid & 0x7ff;  // support 11 bit std id
    FLEXCAN_DRV_ConfigRxMb(flexcanInstance, s_flexcanInfo.rx_mailbox_num, &s_flexcanInfo.rx_info, s_flexcanInfo.rx_id);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_can_speed
 * Description   : flexCAN config speed process
 *
 *END**************************************************************************/
void configure_can_speed(unsigned int speed)
{
    if (speed < 5)
    {
        FLEXCAN_DRV_SetBitrate(flexcanInstance, &bit_rate_table[speed]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : send_can_data
 * Description   :  flexCAN sending data process
 *
 *END**************************************************************************/
void send_can_data(uint8_t * src, uint32_t writeLength)
{
    uint32_t sentCnt = 0;
    uint8_t *sendPtr = src;

    while (sentCnt < writeLength)
    {
        if ((writeLength - sentCnt) <= 8)
        {
            // number of bytes to be sent
            s_flexcanInfo.tx_info.data_length = writeLength - sentCnt; 
            sentCnt += writeLength - sentCnt;
        }
        else 
        {
            // number of bytes to be sent
            s_flexcanInfo.tx_info.data_length = 8;
            sentCnt += 8;
        }
        
        FLEXCAN_DRV_Send(flexcanInstance, s_flexcanInfo.tx_mailbox_num, &s_flexcanInfo.tx_info, s_flexcanInfo.tx_id,
                                (uint8_t *)sendPtr, 1000);
        
        sendPtr += s_flexcanInfo.tx_info.data_length;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_can_data
 * Description   : flexCAN receiving data process
 *
 *END**************************************************************************/
void receive_can_data(uint8_t data, uint32_t instance)
{
      s_flexcanInfo.rx_buf[s_flexcanInfo.rx_buf_write_index++] = data;
      s_flexcanInfo.rx_buf_write_index &= 0x3f;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : reset_can_buffer
 * Description   : flexCAN reset buffer process
 *
 *END**************************************************************************/
void reset_can_buffer(void)
{
      s_flexcanInfo.rx_buf_write_index = 0;
      s_flexcanInfo.rx_buf_read_index = 0;     
}

/*FUNCTION**********************************************************************
 *
 * Function Name : read_can_data
 * Description   : flexCAN read data process
 *
 *END**************************************************************************/
void read_can_data(uint8_t * dest, uint32_t readLength)
{
    uint8_t received_cnt = 0;
    uint64_t timeoutTicks = microseconds_get_ticks() + 20875*500;  // 5ms time out 
    
    while ((received_cnt < readLength) && (microseconds_get_ticks() < timeoutTicks))
    {
        if (s_flexcanInfo.rx_buf_read_index != s_flexcanInfo.rx_buf_write_index)
        {
            dest[received_cnt++] = s_flexcanInfo.rx_buf[s_flexcanInfo.rx_buf_read_index++];
            s_flexcanInfo.rx_buf_read_index &= 0x3f;
        }
    }
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

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_IRQ_Busoff_Handler
 * Description   : flexCAN busoff interrupt handler
 *
 *END**************************************************************************/
void FLEXCAN_DRV_IRQ_Busoff_Handler(uint8_t instance)
{
    FLEXCAN_HAL_ClearErrIntStatus((CAN_Type *)g_flexcanBaseAddr[instance]);  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_IRQ_Error_Handler
 * Description   : flexCAN general error interrupt handler
 *
 *END**************************************************************************/
void FLEXCAN_DRV_IRQ_Error_Handler(uint8_t instance)
{
    FLEXCAN_HAL_ClearErrIntStatus((CAN_Type *)g_flexcanBaseAddr[instance]);  
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

