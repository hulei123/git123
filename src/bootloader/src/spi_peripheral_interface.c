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
#include "spi/fsl_spi_slave_driver.h"
#include "device/fsl_device_registers.h"
#include "packet/serial_packet.h"

#if BL_CONFIG_SPI
//! @addtogroup spi_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Synchronization state between SPI ISR and read/write functions.
typedef struct _spi_transfer_info {
    const uint8_t * writeData;            //!< The applications data to write
    volatile uint32_t bytesToTransfer;    //!< The total number of bytes to be transmitted
} spi_transfer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @brief SPI slave poll for activity function
static bool spi_poll_for_activity(const peripheral_descriptor_t * self);
//! @brief SPI slave init function
static status_t spi_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function);
//! @brief SPI slave shutdown function
static void spi_full_shutdown(const peripheral_descriptor_t * self);
//! @brief SPI slave sending data function
static void spi_data_source(uint8_t * source_byte, uint16_t instance);
//! @brief SPI slave receiving first byte data function
static void spi_initial_data_sink(uint8_t sink_byte, uint16_t instance);
//! @brief SPI slave receiving data function
static void spi_data_sink(uint8_t sink_byte, uint16_t instance);
//! @brief SPI slave writing data function
static status_t spi_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief SPI slave control interface information
 */
const peripheral_control_interface_t g_spiControlInterface = {
    .pollForActivity = spi_poll_for_activity,
    .init = spi_full_init,
    .shutdown = spi_full_shutdown,
    .pump = 0
};

/*!
 * @brief SPI slave byte interface information
 */
const peripheral_byte_inteface_t g_spiByteInterface = {
    .init = NULL,
    .write = spi_write
};

//! @brief Global state for the SPI slave peripheral interface.
static spi_transfer_info_t s_spiInfo = {
    .writeData = 0,
    .bytesToTransfer = 0
};

//! @brief Flag for detecting device activity
static bool s_spiActivity[SPI_INSTANCE_COUNT] = {false};

/*!
 * @brief SPI slave receiving data call back function
 */
static serial_byte_receive_func_t s_spi_app_data_sink_callback;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_poll_for_activity
 * Description   : Polling for SPI slave activities
 *
 *END**************************************************************************/
bool spi_poll_for_activity(const peripheral_descriptor_t * self)
{
    return s_spiActivity[self->instance];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_data_source
 * Description   : SPI slave sending data function
 *
 *END**************************************************************************/
void spi_data_source(uint8_t * source_byte, uint16_t instance)
{
    assert(source_byte);

    if (s_spiInfo.bytesToTransfer)
    {
        *source_byte = *s_spiInfo.writeData++;
        s_spiInfo.bytesToTransfer--;
    }
    else
    {
        *source_byte = 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_initial_data_sink
 * Description   : SPI slave receiving first byte data function
 *
 *END**************************************************************************/
void spi_initial_data_sink(uint8_t sink_byte, uint16_t instance)
{
    if (sink_byte == kFramingPacketStartByte)
    {
        s_spiActivity[instance] = true;
        spi_slave_set_data_sink_func(instance, spi_data_sink);
        s_spi_app_data_sink_callback(sink_byte);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_data_sink
 * Description   : SPI slave receiving data function
 *
 *END**************************************************************************/
void spi_data_sink(uint8_t sink_byte, uint16_t instance)
{
    s_spi_app_data_sink_callback(sink_byte);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_full_init
 * Description   : SPI slave full init function
 *
 *END**************************************************************************/
status_t spi_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function)
{
    s_spi_app_data_sink_callback = function;

    spi_slave_config_t config = {
            .callbacks = {
                    .dataSource = spi_data_source,
                    .dataSink = spi_initial_data_sink
                },
            .phase = kSpiClockPhase_SecondEdge,
            .polarity = kSpiClockPolarity_ActiveLow,
            .direction = kSpiMsbFirst
    };
    // Configure selected pin as spi peripheral interface
    self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);
    spi_slave_init(self->instance, &config);

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_full_shutdown
 * Description   : SPI slave full shutdown function
 *
 *END**************************************************************************/
void spi_full_shutdown(const peripheral_descriptor_t * self)
{
    spi_slave_shutdown(self->instance);
    // Restore selected pin to default state to reduce IDD.
    self->pinmuxConfig(self->instance, kPinmuxType_Default);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : spi_write
 * Description   : SPI slave writing data function
 *
 *END**************************************************************************/
status_t spi_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount)
{
    s_spiInfo.writeData = buffer;
    s_spiInfo.bytesToTransfer = byteCount;

    while (s_spiInfo.bytesToTransfer);

    return kStatus_Success;
}

//! @}

#endif // BL_CONFIG_SPI
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
