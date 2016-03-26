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
#include "i2c/slave/fsl_i2c_slave_driver.h"
#include "device/fsl_device_registers.h"
#include "packet/serial_packet.h"

#if BL_CONFIG_I2C
//! @addtogroup i2c_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// Allow the default to be overridden via the bootloader_config.h file.
#if !defined(BL_DEFAULT_I2C_SLAVE_ADDRESS)
//! @brief Default I2C slave address in 7-bit format.
#define BL_DEFAULT_I2C_SLAVE_ADDRESS    (0x10)
//! @brief Secondary I2C slave address in 7-bit format.
#define BL_SECONDARY_I2C_SLAVE_ADDRESS  (0x12)
#endif // BL_DEFAULT_I2C_SLAVE_ADDRESS

//! @brief Default width of glitches to filter in nanoseconds.
enum
{
    //! Width of glitches to filter in nanoseconds.
    kI2CGlitchFilterWidth_ns = 50
};

//! @brief Synchronization state between I2C ISR and read/write functions.
typedef struct _i2c_transfer_info {
    const uint8_t * writeData;         //!< The applications data to write
    volatile uint32_t bytesToTransfer; //!< The total number of bytes to be transmitted
} i2c_transfer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @brief I2C slave poll for activity function
static bool i2c_poll_for_activity(const peripheral_descriptor_t * self);
//! @brief I2C slave init function
static status_t i2c_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function);
//! @brief I2C slave shutdown function
static void i2c_full_shutdown(const peripheral_descriptor_t * self);
//! @brief I2C slave sending data function
static void i2c_data_source(uint8_t * source_byte);
//! @brief I2C slave receiving first byte data function
static void i2c_initial_data_sink(uint8_t sink_byte, uint32_t instance);
//! @brief I2C slave receiving data function
static void i2c_data_sink(uint8_t sink_byte, uint32_t instance);
//! @brief I2C slave internal init function
static void i2c_peripheral_init(uint32_t instance);
//! @brief I2C slave writing data function
static status_t i2c_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief I2C slave control interface information
 */
const peripheral_control_interface_t g_i2cControlInterface = {
    .pollForActivity = i2c_poll_for_activity,
    .init = i2c_full_init,
    .shutdown = i2c_full_shutdown,
    .pump = 0
};

/*!
 * @brief I2C slave byte interface information
 */
const peripheral_byte_inteface_t g_i2cByteInterface = {
    .init = NULL,
    .write = i2c_write
};

//! @brief Global state for the I2C slave peripheral interface.
static i2c_transfer_info_t s_i2cInfo = {
    .writeData = 0,
    .bytesToTransfer = 0
};

//! @brief Global state for the I2C slave peripheral interface.
static bool s_i2cActivity[I2C_INSTANCE_COUNT] = {false};

/*!
 * @brief I2C slave config information
 */
static i2c_slave_info_t s_i2cSlaveConfig =
{
    .slaveAddress = BL_DEFAULT_I2C_SLAVE_ADDRESS,
    .data_source = i2c_data_source,
    .data_sink = i2c_initial_data_sink,
    .on_error =  i2c_peripheral_init
};

/*!
 * @brief I2C slave receiving data call back function
 */
static serial_byte_receive_func_t s_i2c_app_data_sink_callback;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_poll_for_activity
 * Description   : Polling for I2C slave activities
 *
 *END**************************************************************************/
bool i2c_poll_for_activity(const peripheral_descriptor_t * self)
{
    return s_i2cActivity[self->instance];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_data_source
 * Description   : I2C slave sending data function
 *
 *END**************************************************************************/
void i2c_data_source(uint8_t * source_byte)
{
    assert(source_byte);

    if (s_i2cInfo.bytesToTransfer)
    {
        *source_byte = *s_i2cInfo.writeData++;
        s_i2cInfo.bytesToTransfer--;
    }
    else
    {
        *source_byte = 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_data_sink
 * Description   : I2C slave receiving data function
 *
 *END**************************************************************************/
void i2c_data_sink(uint8_t sink_byte, uint32_t instance)
{
    s_i2c_app_data_sink_callback(sink_byte);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_initial_data_sink
 * Description   : I2C slave receiving first byte data function
 *
 *END**************************************************************************/
void i2c_initial_data_sink(uint8_t sink_byte, uint32_t instance)
{
    if (sink_byte == kFramingPacketStartByte)
    {
        s_i2cActivity[instance] = true;
        i2c_slave_set_data_sink_func(instance, i2c_data_sink);
        s_i2c_app_data_sink_callback(sink_byte);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_peripheral_init
 * Description   : I2C slave Internal init function
 *
 *END**************************************************************************/
void i2c_peripheral_init(uint32_t instance)
{
    i2c_slave_init(instance, &s_i2cSlaveConfig);
    i2c_set_glitch_filter_width(instance, get_bus_clock(), kI2CGlitchFilterWidth_ns);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_full_init
 * Description   : I2C slave full init function
 *
 *END**************************************************************************/
status_t i2c_full_init(const peripheral_descriptor_t * self, serial_byte_receive_func_t function)
{
    s_i2c_app_data_sink_callback = function;

    // Read the address from the configuration field. If it is not set, i.e. 0xff,
    // use the default address.
    uint8_t slaveAddress = g_bootloaderContext.propertyInterface->store->configurationData.i2cSlaveAddress;
    if (slaveAddress != 0xff)
    {
        s_i2cSlaveConfig.slaveAddress = slaveAddress;
    }
#if BL_FEATURE_SECONDARY_I2C_SLAVE_ADDRESS
    else if (is_secondary_i2c_slave_address_enabled())
    {
        s_i2cSlaveConfig.slaveAddress = BL_SECONDARY_I2C_SLAVE_ADDRESS;
    }
#endif // BL_FEATURE_SECONDARY_I2C_SLAVE_ADDRESS
        
    // Configure selected pin as i2c peripheral interface
    self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);

    i2c_peripheral_init(self->instance);

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_full_shutdown
 * Description   : I2C slave full shutdown function
 *
 *END**************************************************************************/
void i2c_full_shutdown(const peripheral_descriptor_t * self)
{
    i2c_slave_deinit(self->instance);
    // Restore selected pin to default state to reduce IDD.
    self->pinmuxConfig(self->instance, kPinmuxType_Default);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_write
 * Description   : I2C slave writing data function
 *
 *END**************************************************************************/
status_t i2c_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount)
{
    s_i2cInfo.writeData = buffer;
    s_i2cInfo.bytesToTransfer = byteCount;

    while (s_i2cInfo.bytesToTransfer);

    return kStatus_Success;
}

//! @}

#endif // BL_CONFIG_I2C
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
