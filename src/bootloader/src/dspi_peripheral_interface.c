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
#include "dspi/fsl_dspi_slave_driver.h"
#include "device/fsl_device_registers.h"
#include "packet/serial_packet.h"

#if BL_CONFIG_DSPI

//! @addtogroup dspi_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Synchronization state between DSPI ISR and read/write functions.
typedef struct _dspi_transfer_info {
    const uint8_t * writeData;            //!< The applications data to write
    volatile uint32_t bytesToTransfer;    //!< The total number of bytes to be transmitted
} dspi_transfer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @brief DSPI slave poll for activity function
static bool dspi_poll_for_activity(const peripheral_descriptor_t * self);
//! @brief DSPI slave init function
static status_t dspi_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function);
//! @brief DSPI slave shutdown function
static void dspi_full_shutdown(const peripheral_descriptor_t * self);
//! @brief DSPI slave sending data function
static void dspi_data_source(uint8_t * source_byte, uint32_t instance);
//! @brief DSPI slave receiving data function
static void dspi_data_sink(uint8_t sink_byte, uint32_t instance);
//! @brief DSPI slave receiving first byte data function
static void dspi_initial_data_sink(uint8_t sink_byte, uint32_t instance);
//! @brief DSPI slave writing data function
static status_t dspi_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief DSPI slave control interface information
 */
const peripheral_control_interface_t g_dspiControlInterface = {
    .pollForActivity = dspi_poll_for_activity,
    .init = dspi_full_init,
    .shutdown = dspi_full_shutdown,
    .pump = 0
};

/*!
 * @brief DSPI slave byte interface information
 */
const peripheral_byte_inteface_t g_dspiByteInterface = {
    .init = NULL,
    .write = dspi_write
};

//! @brief Global state for the DSPI slave peripheral interface.
static dspi_transfer_info_t s_dspiInfo = {
    .writeData = 0,
    .bytesToTransfer = 0
};

//! @brief Global state for the DSPI slave peripheral interface.
static dspi_slave_state_t s_dspiState[SPI_INSTANCE_COUNT];

//! @brief Flag for DSPI detecting device activity
static bool s_dspiActivity[SPI_INSTANCE_COUNT] = {false};

/*!
 * @brief DSPI slave receiving data call back function
 */
static serial_byte_receive_func_t s_dspi_app_data_sink_callback;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_poll_for_activity
 * Description   : Polling for DSPI slave activities
 *
 *END**************************************************************************/
bool dspi_poll_for_activity(const peripheral_descriptor_t * self)
{
    return s_dspiActivity[self->instance];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_data_source
 * Description   : DSPI slave sending data function
 *
 *END**************************************************************************/
void dspi_data_source(uint8_t * source_byte, uint32_t instance)
{
    assert(source_byte);

    if (s_dspiInfo.bytesToTransfer)
    {
        *source_byte = *s_dspiInfo.writeData++;
        s_dspiInfo.bytesToTransfer--;
    }
    else
    {
        *source_byte = 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_initial_data_sink
 * Description   : DSPI slave receiving first byte data function
 *
 *END**************************************************************************/
void dspi_initial_data_sink(uint8_t sink_byte, uint32_t instance)
{
    if (sink_byte == kFramingPacketStartByte)
    {
        s_dspiActivity[instance] = true;
        s_dspiState[instance].callbacks.dataSink = dspi_data_sink;
        s_dspi_app_data_sink_callback(sink_byte);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_data_sink
 * Description   : DSPI slave receiving data function
 *
 *END**************************************************************************/
void dspi_data_sink(uint8_t sink_byte, uint32_t instance)
{
     s_dspi_app_data_sink_callback(sink_byte);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_full_init
 * Description   : DSPI slave full init function
 *
 *END**************************************************************************/
status_t dspi_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function)
{
    s_dspi_app_data_sink_callback = function;

    dspi_slave_user_config_t config = {
            .callbacks = {
                    .dataSource = dspi_data_source,
                    .dataSink = dspi_initial_data_sink,
                    0
                },
            .dataConfig = {
                .bitsPerFrame = 8,
                .clkPolarity = kDspiClockPolarity_ActiveLow,
                .clkPhase = kDspiClockPhase_SecondEdge,
                .direction = kDspiMsbFirst
            }
    };
    // Configure selected pin as spi peripheral interface
    self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);
    dspi_slave_init(self->instance, &config, &s_dspiState[self->instance]);

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_full_shutdown
 * Description   : DSPI slave full shutdown function
 *
 *END**************************************************************************/
void dspi_full_shutdown(const peripheral_descriptor_t * self)
{
    dspi_slave_shutdown(&s_dspiState[self->instance]);
    // Restore selected pin to default state to reduce IDD.
    self->pinmuxConfig(self->instance, kPinmuxType_Default);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_write
 * Description   : DSPI slave writing data function
 *
 *END**************************************************************************/
status_t dspi_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount)
{
    s_dspiInfo.writeData = buffer;
    s_dspiInfo.bytesToTransfer = byteCount;

    while (s_dspiInfo.bytesToTransfer);

    return kStatus_Success;
}

//! @}

#endif // BL_CONFIG_DSPI

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
