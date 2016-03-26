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

#include "utilities/fsl_assert.h"
#include "bootloader/context.h"
#include "bootloader_common.h"
#include "packet/command_packet.h"
#include "flexcan/fsl_flexcan_driver.h"
#include "device/fsl_device_registers.h"
#include "packet/serial_packet.h"

#if BL_CONFIG_CAN

//! @addtogroup flexcan_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief flexCAN global information structure.
typedef struct _flexcan_transfer_info {
    flexcan_state_t     state;          //!< state
    flexcan_data_info_t rx_info;        //!< tx info
    flexcan_data_info_t tx_info;        //!< tx info
    int8_t              baudrate;       //!< baudrate (index)
    flexcan_operation_modes_t mode;
    bool                baudrateDetect; //!< auto baudrate detection
    uint16_t            txId;
    uint16_t            rxId;
} flexcan_transfer_info_t;

//! @brief flexCAN enums.
#define FLEXCAN_RX_MB            8
#define FLEXCAN_TX_MB            9
#define FLEXCAN_DATA_LENGTH      8
#define FLEXCAN_MAX_MB           16
#define FLEXCAN_MAX_SPEED        5

#define FLEXCAN_DEFAULT_RX_ID    0x321                  // default rxid 801
#define FLEXCAN_DEFAULT_TX_ID    0x123                  // default txid 291
#define FLEXCAN_DEFAULT_SPEED    4

// config1 8 bit
#define FLEXCAN_SPEED_MASK       0x0F                   // bit[3:0]
#define FLEXCAN_SPEED_SPEC_MASK  0x08                   // bit[3:3]
#define FLEXCAN_SPEED_INDEX_MASK 0x07                   // bit[2:0]
#define FLEXCAN_CLKSEL_MASK      0x80                   // bit[7:7]

#define FLEXCAN_PROPSEG_MASK     0x70                   // bit[6:4]
#define FLEXCAN_PROPSEG_SHIFT    4

// config2 16 bit
#define FLEXCAN_PRESCALER_MASK   0xFF00                 // bit[15:8]
#define FLEXCAN_PRESCALER_SHIFT  8
#define FLEXCAN_PSEG1_MASK       0x00E0                 // bit[7:5]
#define FLEXCAN_PSEG1_SHIFT      5
#define FLEXCAN_PSEG2_MASK       0x001C                 // bit[4:2]
#define FLEXCAN_PSEG2_SHIFT      2
#define FLEXCAN_RJW_MASK         0x0003                 // bit[1:0]
#define FLEXCAN_RJW_SHIFT        0

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @brief flexCAN poll for activity function
static bool flexcan_poll_for_activity(const peripheral_descriptor_t * self);
//! @brief flexCAN init function
static status_t flexcan_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function);
//! @brief flexCAN shutdown function
static void flexcan_full_shutdown(const peripheral_descriptor_t * self);
//! @brief flexCAN receiving first byte data sink function
static void flexcan_initial_data_sink(uint8_t sink_byte, uint32_t instance);
//! @brief flexCAN receiving data sink function
static void flexcan_data_sink(uint8_t sink_byte, uint32_t instance);
//! @brief flexCAN internal init function
static void flexcan_peripheral_init(uint32_t instance);
//! @brief flexCAN sending data function
static status_t flexcan_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief FlexCAN bit timing parameters calculated by using the method outlined in AN1798, section 4.1.
 */

/*!
 * @brief The table contains propseg, pseg1, pseg2, pre_divider, and rjw.
 */
#ifdef  CPU_MKV46F256VLL15
/* 8Mhz osc clock based */
const flexcan_time_segment_t bit_rate_table_8Mhz[FLEXCAN_MAX_SPEED] = {
    { 0, 2, 2,  7, 2},  /* 125 kHz */
    { 0, 2, 2,  3, 2},  /* 250 kHz */
    { 2, 1, 1,  1, 1},  /* 500 kHz */
    { 2, 2, 2,  0, 2},  /* 750 kHz */
    { 2, 1, 1,  0, 1}   /* 1   MHz */
};

/* 20.9715Mhz internal clock based */
const flexcan_time_segment_t bit_rate_table[] = {
    { 3, 7, 7,  7, 3},  /* 125 kHz */
    { 3, 7, 7,  3, 3},  /* 250 kHz */
    { 3, 7, 7,  1, 3},  /* 500 kHz */
    { 5, 3, 3,  1, 3},  /* 750 kHz */
    { 7, 5, 5,  0, 3}   /* 1   MHz */
};
#endif

#ifdef  CPU_PKE18F512VLH15
/* 24Mhz clock based (ROM version) */
const flexcan_time_segment_t bit_rate_table[FLEXCAN_MAX_SPEED] = {
    { 0, 4, 4, 15, 3},  /* 125 kHz */
    { 0, 4, 4,  7, 3},  /* 250 kHz */
    { 2, 3, 3,  3, 3},  /* 500 kHz */
    { 2, 1, 1,  3, 1},  /* 750 kHz */
    { 4, 2, 2,  1, 2}   /* 1   MHz */
};

/* 12Mhz clock based (FPGA version only) */
const flexcan_time_segment_t bit_rate_table_12Mhz[FLEXCAN_MAX_SPEED] = {
    { 0, 4, 4,  7, 3},  /* 125 kHz */
    { 0, 4, 4,  3, 3},  /* 250 kHz */
    { 2, 3, 3,  1, 3},  /* 500 kHz */
    { 2, 1, 1,  1, 1},  /* 750 kHz */
    { 4, 2, 2,  0, 2}   /* 1   MHz */
};

/* 6Mhz clock based (FPGA version only) */
const flexcan_time_segment_t bit_rate_table_6Mhz[FLEXCAN_MAX_SPEED] = {
    { 0, 4, 4,  3, 3},  /* 125 kHz */
    { 0, 4, 4,  1, 3},  /* 250 kHz */
    { 2, 3, 3,  0, 3},  /* 500 kHz */
    { 2, 1, 1,  0, 1},  /* 750 kHz */
    { 1, 0, 1,  0, 0}   /* 1   MHz */
};

/* 8Mhz clock based (real hardware version) */
const flexcan_time_segment_t bit_rate_table_8Mhz[FLEXCAN_MAX_SPEED] = {
    { 0, 2, 2,  7, 2},  /* 125 kHz */
    { 0, 2, 2,  3, 2},  /* 250 kHz */
    { 2, 1, 1,  1, 1},  /* 500 kHz */
    { 2, 2, 2,  0, 2},  /* 750 kHz */
    { 2, 1, 1,  0, 1}   /* 1   MHz */
};
#endif  // CPU_PKE18F512VLH15

#ifdef CPU_MKV11Z128VLH7 //MKV11Z7_SERIES
/* 10Mhz osc clock based */
const flexcan_time_segment_t bit_rate_table_10Mhz[FLEXCAN_MAX_SPEED] = {
    { 0, 3, 3,  7, 3},  /* 125 kHz */
    { 0, 3, 3,  3, 3},  /* 250 kHz */
    { 2, 2, 2,  1, 2},  /* 500 kHz */
    { 3, 3, 3,  0, 3},  /* 750 kHz */
    { 4, 1, 1,  0, 1}   /* 1   MHz */
};

/* 20.9715Mhz internal clock based */
const flexcan_time_segment_t bit_rate_table[] = {
    { 3, 7, 7,  7, 3},  /* 125 kHz */
    { 3, 7, 7,  3, 3},  /* 250 kHz */
    { 3, 7, 7,  1, 3},  /* 500 kHz */
    { 5, 3, 3,  1, 3},  /* 750 kHz */
    { 7, 5, 5,  0, 3}   /* 1   MHz */
};
#endif

/*!
 * @brief flexCAN control interface information
 */
const peripheral_control_interface_t g_flexcanControlInterface = {
    .pollForActivity = flexcan_poll_for_activity,
    .init = flexcan_full_init,
    .shutdown = flexcan_full_shutdown,
    .pump = 0
};

/*!
 * @brief flexCAN byte interface information
 */
const peripheral_byte_inteface_t g_flexcanByteInterface = {
    .init = NULL,
    .write = flexcan_write
};

//! @brief Global state for the FLEXCAN peripheral interface.
flexcan_transfer_info_t s_flexcanInfo;
static bool s_flexcanActivity[1] = {false};

/*!
 * @brief flexCAN config information
 */
flexcan_info_t s_flexcanConfig =
{
    .data_sink = flexcan_initial_data_sink,
    .on_error =  flexcan_peripheral_init
};

/*!
 * @brief flexCAN receiving data call back function
 */
static serial_byte_receive_func_t s_flexcan_app_data_sink_callback;
extern void * g_flexcanStatePtr[CAN_INSTANCE_COUNT];

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_poll_for_activity
 * Description   : Polling for FlexCAN activities
 *
 *END**************************************************************************/
bool flexcan_poll_for_activity(const peripheral_descriptor_t * self)
{
    return s_flexcanActivity[self->instance];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_initial_data_sink
 * Description   : Receiving first byte data sink function
 *
 *END**************************************************************************/
void flexcan_initial_data_sink(uint8_t sink_byte, uint32_t instance)
{
    if (sink_byte == kFramingPacketStartByte)
    {  
        s_flexcanActivity[instance] = true;
        FLEXCAN_DRV_SetDataSinkFunc(instance, flexcan_data_sink);
        s_flexcan_app_data_sink_callback(sink_byte);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_data_sink
 * Description   : Receiving data sink function
 *
 *END**************************************************************************/
void flexcan_data_sink(uint8_t sink_byte, uint32_t instance)
{
    s_flexcan_app_data_sink_callback(sink_byte);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_peripheral_init
 * Description   : Internal init function
 *
 *END**************************************************************************/
void flexcan_peripheral_init(uint32_t instance)
{
    flexcan_user_config_t flexcan1_data;
    uint8_t canConfig1;
    uint16_t canConfig2;
    uint16_t canId;
    uint8_t i;
    
    flexcan_time_segment_t bit_rate_table1;
    flexcan1_data.max_num_mb = FLEXCAN_MAX_MB;
    flexcan1_data.num_id_filters = kFlexCanRxFifoIDFilters_8;
    flexcan1_data.is_rx_fifo_needed = false; //disable fifo here

    // Read the can txId. If it is not set, i.e. 0xff,
    canId = g_bootloaderContext.propertyInterface->store->configurationData.canTxId;
    if (canId != 0xffff)
    {
        s_flexcanInfo.txId = canId & 0x7ff;  // support 11bit std id format
    }
    else
    {
        s_flexcanInfo.txId = FLEXCAN_DEFAULT_TX_ID;
    }

    // Read the can rxId. If it is not set, i.e. 0xff,
    canId = g_bootloaderContext.propertyInterface->store->configurationData.canRxId;
    if (canId != 0xffff)
    {
        s_flexcanInfo.rxId = canId & 0x7ff;  // support 11bit std id format
    }
    else
    {
        s_flexcanInfo.rxId = FLEXCAN_DEFAULT_RX_ID;
    }

    // Read the can config1. If it is not set, i.e. 0xff,
    canConfig1 = g_bootloaderContext.propertyInterface->store->configurationData.canConfig1;

    // check baud rate config
    if ((canConfig1 & FLEXCAN_SPEED_MASK) == 0x0f)
    {
        // not specified, go with auto detection feature enabled
        s_flexcanInfo.baudrateDetect = true;

        // select default baud rate 1M
        s_flexcanInfo.baudrate = FLEXCAN_MAX_SPEED - 1;
    }
    else
    {
        // baud rate config specified
        s_flexcanInfo.baudrateDetect = false;

        if (canConfig1 & FLEXCAN_SPEED_SPEC_MASK)
        {
            // specified baud rate setting directly
            s_flexcanInfo.baudrate = FLEXCAN_MAX_SPEED;

            // get config data from BCA area
            canConfig2 = g_bootloaderContext.propertyInterface->store->configurationData.canConfig2;

            // prepare specified config data
            bit_rate_table1.pre_divider = (canConfig2 & FLEXCAN_PRESCALER_MASK) >> FLEXCAN_PRESCALER_SHIFT;
            bit_rate_table1.pseg1 = (canConfig2 & FLEXCAN_PSEG1_MASK) >> FLEXCAN_PSEG1_SHIFT;
            bit_rate_table1.pseg2 = (canConfig2 & FLEXCAN_PSEG2_MASK) >> FLEXCAN_PSEG2_SHIFT;
            bit_rate_table1.rjw = canConfig2 & FLEXCAN_RJW_MASK;
            bit_rate_table1.propseg = (canConfig1 & FLEXCAN_PROPSEG_MASK) >> FLEXCAN_PROPSEG_SHIFT;
        }
        else
        {
            // specified speed index, baud rate settings from bootloader
            s_flexcanInfo.baudrate = canConfig1 & FLEXCAN_SPEED_INDEX_MASK;

            if (s_flexcanInfo.baudrate >= FLEXCAN_MAX_SPEED)
            {
                s_flexcanInfo.baudrate = FLEXCAN_MAX_SPEED - 1;
            }
        }

    }

    // also need to get clock selection config data
    FLEXCAN_DRV_Init(instance, &flexcan1_data, true, &s_flexcanInfo.state,
                     &s_flexcanConfig, kFlexCanClkSource_Ipbus);

    if (s_flexcanInfo.baudrate == FLEXCAN_MAX_SPEED)
    {
        // specified baud rate settings directly, need to get other config data
        s_flexcanInfo.baudrate = FLEXCAN_MAX_SPEED - 1;

        // using provided settings
        FLEXCAN_DRV_SetBitrate(instance, &bit_rate_table1);
    }
    else
    {
        // using setting table
        FLEXCAN_DRV_SetBitrate(instance, &bit_rate_table[s_flexcanInfo.baudrate]);
    }

    // if auto detection enabled, then go to listen mode first
    if (s_flexcanInfo.baudrateDetect)
    {
        s_flexcanInfo.mode = kFlexCanListenOnlyMode;
        FLEXCAN_DRV_EnableOperationMode(instance, kFlexCanListenOnlyMode);
    }

    // rx mb mask
    FLEXCAN_DRV_SetRxMbGlobalMask(instance, kFlexCanMbId_Std, 0xffffffff);

    for (i = 0; i < flexcan1_data.max_num_mb; i++)
    {
        FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMbId_Std, i, 0xffffffff);    
    }
    
    // FlexCAN reveive config
    s_flexcanInfo.rx_info.msg_id_type = kFlexCanMbId_Std;
    s_flexcanInfo.rx_info.data_length = FLEXCAN_DATA_LENGTH;

    // Configure RX MB fields
    FLEXCAN_DRV_ConfigRxMb(instance, FLEXCAN_RX_MB, &s_flexcanInfo.rx_info, s_flexcanInfo.rxId);

    FLEXCAN_DRV_EnableMbInt(instance, FLEXCAN_RX_MB);

    // FlexCAN transfer config
    s_flexcanInfo.tx_info.msg_id_type = kFlexCanMbId_Std;
    s_flexcanInfo.tx_info.data_length = FLEXCAN_DATA_LENGTH;

    FLEXCAN_DRV_ConfigTxMb(instance, FLEXCAN_TX_MB, &s_flexcanInfo.tx_info, s_flexcanInfo.txId);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_full_init
 * Description   : full init function
 *
 *END**************************************************************************/
status_t flexcan_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function)
{
    s_flexcan_app_data_sink_callback = function;

    // Configure selected pin as flexcan peripheral interface
    self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);

    flexcan_peripheral_init(self->instance);
    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_full_shutdown
 * Description   : full shutdown function
 *
 *END**************************************************************************/
void flexcan_full_shutdown(const peripheral_descriptor_t * self)
{
    FLEXCAN_DRV_Deinit(self->instance);

    // Restore selected pin to default state to reduce IDD.
    self->pinmuxConfig(self->instance, kPinmuxType_Default);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_write
 * Description   : Send data function
 *
 *END**************************************************************************/
status_t flexcan_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount)
{
    uint32_t sentCnt = 0;
    uint8_t *sendPtr = (uint8_t *)buffer;

    while (sentCnt < byteCount)
    {
        if ((byteCount - sentCnt) <= 8)
        {
            s_flexcanInfo.tx_info.data_length = byteCount - sentCnt; // number of bytes to be sent
            sentCnt += byteCount - sentCnt;
        }
        else
        {
            s_flexcanInfo.tx_info.data_length = 8; // number of bytes to be sent
            sentCnt += 8;
        }

        FLEXCAN_DRV_Send(self->instance, FLEXCAN_TX_MB, &s_flexcanInfo.tx_info, s_flexcanInfo.txId, (uint8_t *)sendPtr, 1000);
        sendPtr += s_flexcanInfo.tx_info.data_length;
    }

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_IRQ_Busoff_Handler
 * Description   : Busoff interrupt handler
 *
 *END**************************************************************************/
void FLEXCAN_DRV_IRQ_Busoff_Handler(uint8_t instance)
{
    FLEXCAN_HAL_ClearErrIntStatus((CAN_Type *)g_flexcanBaseAddr[instance]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_IRQ_Error_Handler
 * Description   : General error interrupt handler including auto baud rate
 *                 detection processing
 *
 *END**************************************************************************/
void FLEXCAN_DRV_IRQ_Error_Handler(uint8_t instance)
{
    CAN_Type * canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    if (s_flexcanInfo.baudrateDetect)
    {
        uint32_t status = FLEXCAN_HAL_GetErrStatus(canBaseAddr);

        if (s_flexcanInfo.mode == kFlexCanListenOnlyMode)
        {
            if ((status & (CAN_ESR1_RX_MASK | CAN_ESR1_BIT0ERR_MASK)) &&
               (!(status & (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_BIT1ERR_MASK))))
            {
                // we found the right baud rate
                FLEXCAN_HAL_DisableOperationMode(canBaseAddr, kFlexCanListenOnlyMode);
                s_flexcanInfo.mode = kFlexCanNormalMode;
            }
            else
            {
                s_flexcanInfo.baudrate--;

                if (s_flexcanInfo.baudrate < 0)
                {
                    s_flexcanInfo.baudrate = FLEXCAN_MAX_SPEED - 1;
                }

                FLEXCAN_DRV_SetBitrate(instance, &bit_rate_table[s_flexcanInfo.baudrate]);
            }
        }
        else if (status & (CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_BIT1ERR_MASK))
        {
            FLEXCAN_DRV_SoftReset(instance);
            flexcan_peripheral_init(instance);
        }
    }

    FLEXCAN_HAL_ClearErrIntStatus(canBaseAddr);
}

#ifdef MKV11Z7_SERIES
/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_CAN_IRQ_Handler
 * Description   : General error interrupt handler including auto baud rate
 *                 detection processing for KV11
 *
 *END**************************************************************************/
void FLEXCAN_DRV_CAN_IRQ_Handler(uint8_t instance)
{
    CAN_Type * canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    uint32_t status = CAN_RD_ESR1(canBaseAddr);
    uint32_t ctrl1 = CAN_RD_CTRL1(canBaseAddr);

    //Bus off interrupt
    if ((ctrl1 & CAN_CTRL1_BOFFMSK_MASK) && (status & CAN_ESR1_BOFFINT_MASK))
    {
        FLEXCAN_DRV_IRQ_Busoff_Handler(instance);
    }
    //Error interrupt
    else if ((ctrl1 & CAN_CTRL1_ERRMSK_MASK) && (status & CAN_ESR1_ERRINT_MASK))
    {
        if (s_flexcanInfo.baudrateDetect)
        {
            if (s_flexcanInfo.mode == kFlexCanListenOnlyMode)
            {
                if ((status & (CAN_ESR1_RX_MASK | CAN_ESR1_BIT0ERR_MASK)) &&
                   (!(status & (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_BIT1ERR_MASK))))
                {
                    // we found the right baud rate
                    FLEXCAN_HAL_DisableOperationMode(canBaseAddr, kFlexCanListenOnlyMode);
                    s_flexcanInfo.mode = kFlexCanNormalMode;
                }
                else
                {
                    s_flexcanInfo.baudrate--;

                    if (s_flexcanInfo.baudrate < 0)
                    {
                        s_flexcanInfo.baudrate = FLEXCAN_MAX_SPEED - 1;
                    }

                    FLEXCAN_DRV_SetBitrate(instance, &bit_rate_table[s_flexcanInfo.baudrate]);
                }
            }
            else
            {
                FLEXCAN_DRV_SoftReset(instance);
                flexcan_peripheral_init(instance);
            }
        }
        FLEXCAN_HAL_ClearErrIntStatus(canBaseAddr);
    }
    //wake up interrupt
    else if (CAN_RD_MCR_WAKMSK(canBaseAddr) && (status & CAN_ESR1_WAKINT_MASK))
    {
        FLEXCAN_DRV_IRQHandler(instance);
    }
    else
    {
        FLEXCAN_DRV_IRQHandler(instance);
    }
}
#endif //MKV11Z7_SERIES

//! @}

#endif // BL_CONFIG_CAN

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
