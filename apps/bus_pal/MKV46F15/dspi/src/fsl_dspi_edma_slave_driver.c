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

#include <string.h>
#include <assert.h>
#include "fsl_dspi_edma_slave_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Flags of DSPI slave event.
 *
 * DSPI event used to notify user that it finishes the task.
 */
typedef enum _dspi_edma_event_flags {
    kDspiEdmaTransferDone = 0x01,         /*!< Transferring done flag */
} dspi_edma_event_flag_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for DSPI instances. */
extern const uint32_t g_dspiBaseAddr[];

/*! @brief Table to save DSPI IRQ enum numbers defined in CMSIS header file. */
extern const IRQn_Type g_dspiIrqId[SPI_INSTANCE_COUNT];

/* Pointer to runtime state structure.*/
extern void * g_dspiStatePtr[SPI_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief DSPI receive done callback function
 * @details This function is called when receiving is done.
 *
 * @param param pointer to parameter
 * @param status current status of eDMA channel
 */
static void DSPI_DRV_EdmaRxCallback(void *param, edma_chn_status_t status);

/*!
 * @brief DSPI transmit done callback function
 * @details This function is called when transmitting is done.
 *
 * @param param pointer to parameter
 * @param status current status of eDMA channel
 */
static void DSPI_DRV_EdmaTxCallback(void *param, edma_chn_status_t status);

/*!
 * @brief Finish the current DSPI transfer
 * @details This function stop the DSPI transfer
 *
 * @param instance The instance of DSPI hardware
 */
static void DSPI_DRV_EdmaCompleteTransfer(uint32_t instance);

/*!
 * @brief Start slave transferring.
 * @details This function make starting the transfer.
 *
 * @param instance The instance number of DSPI peripheral
 * @param sendBuffer The pointer to transmit buffer
 * @param receiveBuffer The pointer to receive buffer
 * @param transferByteCount The transfer size
 */
static void DSPI_DRV_EdmaSlaveStartTransfer(uint32_t instance,
                                            const uint8_t *sendBuffer,
                                            uint8_t *receiveBuffer,
                                            uint32_t transferByteCount);

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveInit
 * Description   : Initialize a DSPI slave instance.
 * Un-gate DSPI's clock, setup the default value and initializes the required
 * resources.
 *
 *END**************************************************************************/
dspi_status_t DSPI_DRV_EdmaSlaveInit(uint32_t instance,
                                     dspi_edma_slave_state_t * dspiState,
                                     const dspi_edma_slave_user_config_t * slaveConfig)
{
    dspi_status_t result = kStatus_DSPI_Success;
    dma_request_source_t dspiTxEdmaRequest = kDmaRequestMux0Disable;
    dma_request_source_t dspiRxEdmaRequest = kDmaRequestMux0Disable;
    uint32_t baseAddr = g_dspiBaseAddr[instance];

    // Check the instance, eDMA only support SPI0
    if (instance != 0)
    {
        return kStatus_DSPI_OutOfRange;
    }

    // Check parameter pointer is not NULL
    if ((dspiState == NULL) || (slaveConfig == NULL))
    {
        return kStatus_DSPI_InvalidParameter;
    }

    // Check DSPI slave instance is already initialized
    if (g_dspiStatePtr[instance])
    {
        return kStatus_DSPI_Initialized;
    }

    // Check if bits/frame is 3 bytes, eDMA driver does not support 3bytes copy at a time.
    if ((16 < slaveConfig->dataConfig.bitsPerFrame) && (slaveConfig->dataConfig.bitsPerFrame <= 24))
    {
        return kStatus_DSPI_OutOfRange;
    }

    // Check bits/frame number
    if (slaveConfig->dataConfig.bitsPerFrame > 32)
    {
        return kStatus_DSPI_OutOfRange;
    }

    // Clear the run-time state struct for this instance.
    memset(dspiState, 0, sizeof(* dspiState));

    // Initial default value slave state structure
    dspiState->status = kStatus_DSPI_Success;
    dspiState->errorCount = 0;
    dspiState->dummyPattern = slaveConfig->dummyPattern;
    dspiState->remainingSendByteCount = 0;
    dspiState->remainingReceiveByteCount = 0;
    dspiState->isTransferInProgress = false;
    dspiState->extraReceiveByte = 0;

    if (kStatus_OSA_Success != OSA_EventCreate(&dspiState->event, kEventAutoClear))
    {
        // Create event error
        dspiState->status = kStatus_DSPI_Error;
        return kStatus_DSPI_Error;
    }

    // configure the run-time state struct with the nubmer of bits/frame
    dspiState->bitsPerFrame = slaveConfig->dataConfig.bitsPerFrame;

    // Enable clock for DSPI
    CLOCK_SYS_EnableSpiClock(instance);

    // Reset the DSPI module, which also disables the DSPI module
    DSPI_HAL_Init(baseAddr);

    // Set to slave mode.
    DSPI_HAL_SetMasterSlaveMode(baseAddr, kDspiSlave);

    // Set slave data format
    result = DSPI_HAL_SetDataFormat(baseAddr, kDspiCtar0, &slaveConfig->dataConfig);

    // Enable fifo operation (regardless of FIFO depth)
    DSPI_HAL_SetFifoCmd(baseAddr, true, true);

    // flush the fifos
    DSPI_HAL_SetFlushFifoCmd(baseAddr, true, true);

    switch (instance)
    {
        case 0:
            // SPI0
            dspiRxEdmaRequest = kDmaRequestMux0SPI0Rx;
            dspiTxEdmaRequest = kDmaRequestMux0SPI0Tx;
            break;
        case 1:
            // SPI1
            break;
        default :
            break;
    }

    // This channel transfers data from RX FIFO to receiveBuffer
    if (kEDMAInvalidChannel == EDMA_DRV_RequestChannel(kEDMAAnyChannel,
                                                       dspiRxEdmaRequest,
                                                       &dspiState->edmaRxChannel))
    {
        dspiState->status = kStatus_DSPI_Error;
        return kStatus_DSPI_DMAChannelInvalid;
    }

    // This channel transfers data from transmitBuffer to TX FIFO
    if (kEDMAInvalidChannel == EDMA_DRV_RequestChannel(kEDMAAnyChannel,
                                                       dspiTxEdmaRequest,
                                                       &dspiState->edmaTxChannel))
    {
        dspiState->status = kStatus_DSPI_Error;
        return kStatus_DSPI_DMAChannelInvalid;
    }

    // Configure IRQ state structure, so irq handler can point to the correct state structure
    g_dspiStatePtr[instance] = dspiState;

    // Enable the interrupt
    INT_SYS_EnableIRQ(g_dspiIrqId[instance]);

    // DSPI module enable
    DSPI_HAL_Enable(baseAddr);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveDeinit
 * Description   : Shutdown a DSPI instance.
 * Resets the DSPI peripheral, disables the interrupt to the core, and gates its clock.
 *
 *END**************************************************************************/
void DSPI_DRV_EdmaSlaveDeinit(uint32_t instance)
{
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];

    // Validate function parameters
    assert(instance < SPI_INSTANCE_COUNT);

    if (!dspiState)
    {
        return;
    }

    // disable the interrupt
    INT_SYS_DisableIRQ(g_dspiIrqId[instance]);

    // Stop the transfer process in the slave
    DSPI_HAL_StopTransfer(baseAddr);

    // Wait until the DSPI run status signals that is has halted before shutting
    // down the module and before gating off the DSPI clock source.  Otherwise, if the DSPI
    // is shut down before it has halted it's internal processes, it may be left in an unknown
    // state.
    // Note that if the master slave select is still asserted, the run status will never clear.
    // Hence, ensure before shutting down the slave that the master has de-asserted the slave
    // select signal (it should be high if slave select active low or it should be low if
    // slave select is active high).
    while((DSPI_HAL_GetStatusFlag(baseAddr, kDspiTxAndRxStatus))) { }

    // Restore the module to defaults then power it down. This also disables the DSPI module.
    DSPI_HAL_Init(baseAddr);

    // Gate the clock for DSPI.
    CLOCK_SYS_DisableSpiClock(instance);

    // Destroy event
    OSA_EventDestroy(&dspiState->event);

    EDMA_DRV_ReleaseChannel(&dspiState->edmaTxChannel);
    EDMA_DRV_ReleaseChannel(&dspiState->edmaRxChannel);

    dspiState->status = kStatus_DSPI_NonInit;

    // Clear state pointer.
    g_dspiStatePtr[instance] = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveStartTransfer
 * Description   : Starts transfer data on SPI bus using eDMA and non-blocking call
 * Start DSPI transfering, update transmit/receive information into slave state structure
 *
 *END**************************************************************************/
static void DSPI_DRV_EdmaSlaveStartTransfer(uint32_t instance,
                                            const uint8_t *sendBuffer,
                                            uint8_t *receiveBuffer,
                                            uint32_t transferByteCount)
{
    uint32_t dmaInstance, dmaChannel;
    uint32_t majorIteration;
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    uint8_t nBytes = dspiState->bitsPerFrame / 8;

    // Calculate number of bytes in frame
    if (dspiState->bitsPerFrame % 8 != 0)
    {
        nBytes ++;
    }

    // Stop the transfer first
    DSPI_HAL_StopTransfer(baseAddr);

    // Reset the transfer counter to 0.
    DSPI_HAL_PresetTransferCount(baseAddr, 0);

    // flush the fifos
    DSPI_HAL_SetFlushFifoCmd(baseAddr, true, true);

    // Clear DSPI flags
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxAndRxStatus);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiEndOfQueue);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxFifoUnderflow);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxFifoFillRequest);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiRxFifoOverflow);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiRxFifoDrainRequest);

    if (sendBuffer)
    {
        // Configure for transmit

        dmaInstance = VIRTUAL_CHN_TO_EDMA_MODULE_REGBASE(dspiState->edmaTxChannel.channel);
        dmaChannel = VIRTUAL_CHN_TO_EDMA_CHN(dspiState->edmaTxChannel.channel);
        EDMA_HAL_ClearDoneStatusFlag(dmaInstance, (edma_channel_indicator_t)dmaChannel);
        EDMA_HAL_HTCDClearReg(dmaInstance, dmaChannel);

        // Source addr, TX Send buffer
        EDMA_HAL_HTCDSetSrcAddr(dmaInstance, dmaChannel,(uint32_t)(sendBuffer));

        // Source address adjust last: don't increment source address
        EDMA_HAL_HTCDSetSrcLastAdjust(dmaInstance, dmaChannel, 0);


        // Destination is the TX FIFO
        EDMA_HAL_HTCDSetDestAddr(dmaInstance, dmaChannel,
                                 DSPI_HAL_GetSlavePushrRegAddr(baseAddr));

        // Dest addr offset don't increment as it is a FIFO
        EDMA_HAL_HTCDSetDestOffset(dmaInstance, dmaChannel, 0);

        // No adjustment needed for destination addr
        EDMA_HAL_HTCDSetDestLastAdjust(dmaInstance, dmaChannel, 0);

        //* The source and destination attributes (bit size) depends on bits/frame setting
        if (dspiState->bitsPerFrame <= 8)
        {
            // Source addr offset is 1 as send buffer pointer is incremented 1 bytes for each write
            EDMA_HAL_HTCDSetSrcOffset(dmaInstance, dmaChannel, 1);

            // Destination size is always one byte, source size varies depending on bits/frame
            EDMA_HAL_HTCDSetAttribute(dmaInstance, dmaChannel,
                                      kEDMAModuloDisable, kEDMAModuloDisable,
                                      kEDMATransferSize_1Bytes, kEDMATransferSize_1Bytes);

            // Transfer 1 byte from RX FIFO to receive buffer
            EDMA_HAL_HTCDSetNbytes(dmaInstance, dmaChannel, 1);

            // Adjust majorIteration to 1 byte per transfer
            majorIteration = transferByteCount;
            dspiState->remainingSendByteCount = transferByteCount;
        }
        // Source size is two bytes
        else if (dspiState->bitsPerFrame <= 16)
        {
            // Source addr offset is 2 as send buffer pointer is incremented 2 bytes for each write
            EDMA_HAL_HTCDSetSrcOffset(dmaInstance, dmaChannel, 2);

            // Destination size is always one byte, source size varies depending on bits/frame
            EDMA_HAL_HTCDSetAttribute(dmaInstance, dmaChannel,
                                      kEDMAModuloDisable, kEDMAModuloDisable,
                                      kEDMATransferSize_2Bytes, kEDMATransferSize_2Bytes);

            // Transfer 2 bytes from transmit buffer to TX FIFO
            EDMA_HAL_HTCDSetNbytes(dmaInstance, dmaChannel, 2);

            // Adjust majorIteration to 2 bytes per transfer
            majorIteration = transferByteCount/2;

            // Adding one more byte if size is odd
            if(transferByteCount % 2 != 0)
            {
                majorIteration ++;
            }
            dspiState->remainingSendByteCount = majorIteration * 2;
        }
        // 3 bytes/frame does not support because eDMA can not copy 3bytes a time.

        // Source size 4 bytes (32-bit)
        else
        {
            // Source addr offset is 4 as send buffer pointer is incremented 4 bytes for each write
            EDMA_HAL_HTCDSetSrcOffset(dmaInstance, dmaChannel, 4);
            // Destination size is always one byte, source size varies depending on bits/frame

            EDMA_HAL_HTCDSetAttribute(dmaInstance, dmaChannel,
                                      kEDMAModuloDisable, kEDMAModuloDisable,
                                      kEDMATransferSize_4Bytes, kEDMATransferSize_4Bytes);

            // Transfer 4 bytes from transmit buffer to TX FIFO */
            EDMA_HAL_HTCDSetNbytes(dmaInstance, dmaChannel, 4);

            // Adjust majorIteration to 4 bytes per transfer
            majorIteration = transferByteCount/4;

            // Adding byte(s) if size is odd
            if(transferByteCount % 4 != 0)
            {
                majorIteration ++;
            }
            dspiState->remainingSendByteCount = majorIteration * 4;
        }
        // Configure CITER and BITER fields and clear the ELINK field (disable channel linking)
        EDMA_HAL_HTCDSetChannelMinorLink(dmaInstance, dmaChannel, 0, false);
        EDMA_HAL_HTCDSetMajorCount(dmaInstance, dmaChannel, majorIteration);

        // Now that the TCD was set up, enable the DSPI Peripheral Hardware request for the
        // TX FIFO
        EDMA_HAL_SetDmaRequestCmd(dmaInstance, (edma_channel_indicator_t)dmaChannel, true);

        // Set up eDMA callback
        // Due to MISRA 11.1 rule:
        // Conversions shall not be performed between a pointer to a function
        // and any type other than an integral type.
        // We first have to typecast the callback function pointer as a uint32_t before typecasting
        // as a void pointer.
        EDMA_DRV_InstallCallback(&dspiState->edmaTxChannel,
                                 DSPI_DRV_EdmaTxCallback,(void *)instance);

        // Enable interrupt
        EDMA_HAL_HTCDSetIntCmd(dmaInstance, dmaChannel, true);

        // Change DSPI status
        dspiState->sendBuffer = sendBuffer;
    }
    else
    {
        // Write known data (zeros) if no source buffer passed in
        DSPI_HAL_WriteDataSlavemode(baseAddr, dspiState->dummyPattern);

        // Transmission FIFO fill request interrupt disable
        DSPI_HAL_SetTxFifoFillDmaIntMode(baseAddr, kDspiGenerateIntReq, false);
    }

    if ((receiveBuffer) && (transferByteCount/nBytes > 0))
    {
        // Configure for receive
        dmaInstance = VIRTUAL_CHN_TO_EDMA_MODULE_REGBASE(dspiState->edmaRxChannel.channel);
        dmaChannel = VIRTUAL_CHN_TO_EDMA_CHN(dspiState->edmaRxChannel.channel);
        EDMA_HAL_ClearDoneStatusFlag(dmaInstance, (edma_channel_indicator_t)dmaChannel);
        EDMA_HAL_HTCDClearReg(dmaInstance, dmaChannel);

        // Source addr, RX FIFO
        EDMA_HAL_HTCDSetSrcAddr(dmaInstance, dmaChannel,
                                DSPI_HAL_GetPoprRegAddr(baseAddr));

        // Source addr offset is 0 as source addr never increments
        EDMA_HAL_HTCDSetSrcOffset(dmaInstance, dmaChannel, 0);

        // Source address adjust last: don't increment source address, it is constant
        EDMA_HAL_HTCDSetSrcLastAdjust(dmaInstance, dmaChannel, 0);

        // Destination is the receive buffer
        EDMA_HAL_HTCDSetDestAddr(dmaInstance, dmaChannel, (uint32_t)(receiveBuffer));

        // No adjustment needed for destination addr for most bits/frame. This field gets
        // updated for the special case of 24-bits/frame
        EDMA_HAL_HTCDSetDestLastAdjust(dmaInstance, dmaChannel, 0);

        // The source and destination attributes (bit size) depends on bits/frame setting
        if (dspiState->bitsPerFrame <= 8)
        {
            // Dest addr offset, always increment to the next byte
            EDMA_HAL_HTCDSetDestOffset(dmaInstance, dmaChannel, 1);

            // Destination size is always one byte, source size varies depending on bits/frame
            EDMA_HAL_HTCDSetAttribute(dmaInstance, dmaChannel,
                                      kEDMAModuloDisable, kEDMAModuloDisable,
                                      kEDMATransferSize_1Bytes, kEDMATransferSize_1Bytes);

            // Transfer 1 byte from RX FIFO to receive buffer
            EDMA_HAL_HTCDSetNbytes(dmaInstance, dmaChannel, 1);

            // Adjust majorIteration to 1 byte per transfer
            majorIteration = transferByteCount;

            // Update remaining receive byte count
            dspiState->remainingReceiveByteCount = transferByteCount;
        }
        else if (dspiState->bitsPerFrame <= 16)
        // Source size is two bytes
        {
            // Dest addr offset, always increment to the next byte
            EDMA_HAL_HTCDSetDestOffset(dmaInstance, dmaChannel, 2);

            // Destination size is always one byte, source size varies depending on bits/frame
            EDMA_HAL_HTCDSetAttribute(dmaInstance, dmaChannel,
                                      kEDMAModuloDisable, kEDMAModuloDisable,
                                      kEDMATransferSize_2Bytes, kEDMATransferSize_2Bytes);

            // Transfer 2 bytes from RX FIFO to receive buffer
            EDMA_HAL_HTCDSetNbytes(dmaInstance, dmaChannel, 2);

            // Adjust majorIteration to 2 bytes per transfer
            majorIteration = transferByteCount/2;

            // Update remaining receive byte count
            dspiState->remainingReceiveByteCount = majorIteration * 2;

            // Calculate the receive extra bytes
            dspiState->extraReceiveByte = transferByteCount - dspiState->remainingReceiveByteCount;
        }
        else
        // Source size 4 bytes (32-bit)
        {
            // Dest addr offset, always increment to the next byte
            EDMA_HAL_HTCDSetDestOffset(dmaInstance, dmaChannel, 4);

            // Destination size is always one byte, source size varies depending on bits/frame
            EDMA_HAL_HTCDSetAttribute(dmaInstance, dmaChannel,
                                      kEDMAModuloDisable, kEDMAModuloDisable,
                                      kEDMATransferSize_4Bytes, kEDMATransferSize_4Bytes);

            // Transfer 4 bytes from RX FIFO to receive buffer
            EDMA_HAL_HTCDSetNbytes(dmaInstance, dmaChannel, 4);

            // Adjust majorIteration to 4 bytes per transfer
            majorIteration = transferByteCount/4;

            // Update remaining receive byte count
            dspiState->remainingReceiveByteCount = majorIteration * 4;

            // Calculate the receive extra bytes
            dspiState->extraReceiveByte = transferByteCount - dspiState->remainingReceiveByteCount;
        }

        // Configure CITER and BITER fields and clear the ELINK field (disable channel linking)
        EDMA_HAL_HTCDSetChannelMinorLink(dmaInstance, dmaChannel, 0, false);
        EDMA_HAL_HTCDSetMajorCount(dmaInstance, dmaChannel, majorIteration);

        // Now that the TCD was set up, enable the DSPI Peripheral Hardware request for the
        // RX FIFO
        EDMA_HAL_SetDmaRequestCmd(dmaInstance, (edma_channel_indicator_t)dmaChannel, true);

        // Due to MISRA 11.1 rule:
        // Conversions shall not be performed between a pointer to a function
        // and any type other than an integral type.
        // We first have to typecast the callback function pointer as a uint32_t before typecasting
        // as a void pointer.
        EDMA_DRV_InstallCallback(&dspiState->edmaRxChannel,
                                 DSPI_DRV_EdmaRxCallback,(void *)instance);

        // Enable interrupt
        EDMA_HAL_HTCDSetIntCmd(dmaInstance, dmaChannel, true);

        // Change DSPI state
        dspiState->receiveBuffer = receiveBuffer;
    }
    else if (receiveBuffer)
    {
        // Change DSPI state
        dspiState->receiveBuffer = receiveBuffer;

        // Calculate the receive extra bytes
        dspiState->extraReceiveByte = transferByteCount;

        // Slave receive length is less than bits/frame number of bytes
        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);
        DSPI_HAL_SetIntMode(baseAddr, kDspiTxComplete, true);
    }
    else
    {
        // Reception FIFO fill request interrupt disable
        DSPI_HAL_SetRxFifoDrainDmaIntMode(baseAddr, kDspiGenerateIntReq, false);
    }


    if((sendBuffer) && (dspiState->remainingSendByteCount > 0))
    {
        // Check if number of send bytes is smaller than FIFO deepth, enable transmit completed
        // interrupt to know when transferring is done
        if (dspiState->remainingSendByteCount <= (FSL_FEATURE_DSPI_FIFO_SIZEn(instance) * nBytes))
        {
            DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);
            DSPI_HAL_SetIntMode(baseAddr, kDspiTxComplete, true);
        }
        else
        {
            DSPI_HAL_SetIntMode(baseAddr, kDspiTxComplete, false);
        }
        // Enable the Transmit FIFO Fill Request as a DMA request
        DSPI_HAL_SetTxFifoFillDmaIntMode(baseAddr, kDspiGenerateDmaReq, true);
    }
    if((receiveBuffer) && (dspiState->remainingReceiveByteCount > 0))
    {
        // Enable the Receive FIFO Drain Request as a DMA request
        DSPI_HAL_SetRxFifoDrainDmaIntMode(baseAddr, kDspiGenerateDmaReq, true);
    }

    // Update state
    dspiState->isTransferInProgress = true;
    dspiState->status = kStatus_DSPI_Busy;

    // Start DSPI transfers, set to running state
    DSPI_HAL_StartTransfer(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveAbortTransfer
 * Description   : Abort tranfer
 * Abort data transfer, using after function DSPI_DRV_EdmaSlaveTransfer() to abort
 * transfering data.
 *
 *END**************************************************************************/
dspi_status_t DSPI_DRV_EdmaSlaveAbortTransfer(uint32_t instance)
{
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];

    // Check driver initialized
    if (!dspiState)
    {
        return kStatus_DSPI_NonInit;
    }

    // Check current status
    if (!dspiState->isTransferInProgress)
    {
        return kStatus_DSPI_NoTransferInProgress;
    }

    // Force complete the transfer
    DSPI_DRV_EdmaCompleteTransfer(instance);

    return kStatus_DSPI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaCompleteTransfer
 * Description   : Finish the transfer
 * Called when transfer is finished
 *
 *END**************************************************************************/
static void DSPI_DRV_EdmaCompleteTransfer(uint32_t instance)
{
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    uint32_t edmaBaseAddr = VIRTUAL_CHN_TO_EDMA_MODULE_REGBASE(dspiState->edmaRxChannel.channel);
    uint32_t edmaChannel = VIRTUAL_CHN_TO_EDMA_CHN(dspiState->edmaRxChannel.channel);
    uint32_t readData;

    // Disable Rx DMA major loop interrupt
    EDMA_HAL_HTCDSetIntCmd(edmaBaseAddr, edmaChannel, false);

    // Stop Rx DMA channel.
    EDMA_HAL_SetDmaRequestCmd(edmaBaseAddr, (edma_channel_indicator_t)edmaChannel, false);

    edmaBaseAddr = VIRTUAL_CHN_TO_EDMA_MODULE_REGBASE(dspiState->edmaTxChannel.channel);
    edmaChannel = VIRTUAL_CHN_TO_EDMA_CHN(dspiState->edmaTxChannel.channel);

    // Disable Tx DMA major loop interrupt
    EDMA_HAL_HTCDSetIntCmd(edmaBaseAddr, edmaChannel, false);

    // Stop Tx DMA channel.
    EDMA_HAL_SetDmaRequestCmd(edmaBaseAddr, (edma_channel_indicator_t)edmaChannel, false);

    // Stop transfer
    DSPI_HAL_StopTransfer(baseAddr);

    // Disable the transmit FIFO fill request
    DSPI_HAL_SetTxFifoFillDmaIntMode(baseAddr, kDspiGenerateDmaReq, false);
    // Disable the Receive FIFO Drain Request
    DSPI_HAL_SetRxFifoDrainDmaIntMode(baseAddr, kDspiGenerateDmaReq, false);
    // Disable transmit complete interrupt request
    DSPI_HAL_SetIntMode(baseAddr, kDspiTxComplete, false);

    // Update extra receive bytes
    if((dspiState->extraReceiveByte > 0) && (dspiState->receiveBuffer))
    {
        // Read data from FIFO and clear flag
        readData = DSPI_HAL_ReadData(baseAddr);

        // First byte
        dspiState->receiveBuffer[dspiState->remainingReceiveByteCount] = (uint8_t)readData;
        if ((dspiState->extraReceiveByte > 0) &&(--dspiState->extraReceiveByte > 0))
        {
            // Second byte if available
            dspiState->receiveBuffer[dspiState->remainingReceiveByteCount + 1] = (uint8_t)(readData >> 8);
        }

        if ((dspiState->extraReceiveByte > 0) &&(--dspiState->extraReceiveByte > 0))
        {
            // last byte if available
            dspiState->receiveBuffer[dspiState->remainingReceiveByteCount + 2] = (uint8_t)(readData >> 16);
        }
    }

    // Update status
    dspiState->status = kStatus_DSPI_Success;
    dspiState->isTransferInProgress = false;
    dspiState->sendBuffer = NULL;
    dspiState->receiveBuffer = NULL;
    dspiState->remainingSendByteCount = 0;
    dspiState->remainingReceiveByteCount = 0;
    dspiState->extraReceiveByte = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveGetTransferStatus
 * Description   : Returns whether the previous transfer has finished yet.
 * When performing an async transfer, the user can call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success). In addition, if the transfer
 * is still in progress, the user can obtain the number of words that have been currently
 * transferred.
 *
 *END**************************************************************************/
dspi_status_t DSPI_DRV_EdmaSlaveGetTransferStatus(uint32_t instance, uint32_t * framesTransferred)
{
    // instantiate local variable of type dspi_edma_slave_state_t and point to global state
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    uint32_t baseAddr = g_dspiBaseAddr[instance];

    // Fill in the bytes transferred.
    if (framesTransferred)
    {
        *framesTransferred = DSPI_HAL_GetTransferCount(baseAddr);
    }

    return ((dspiState->isTransferInProgress) ? kStatus_DSPI_Busy : kStatus_DSPI_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveTransfer
 * Description   : Transfer data to master
 * Start transfer data to master
 *
 *END**************************************************************************/
dspi_status_t DSPI_DRV_EdmaSlaveTransfer(uint32_t instance,
                                         const uint8_t *sendBuffer,
                                         uint8_t *receiveBuffer,
                                         uint32_t transferByteCount)
{
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];

    assert(instance < SPI_INSTANCE_COUNT);

    // Check if DSPI is not initialized
    if (!dspiState)
    {
        return kStatus_DSPI_NonInit;
    }

    // Check if buffers and length is empty
    if (((!sendBuffer) && (!receiveBuffer)) ||
        (!transferByteCount))
    {
        return kStatus_DSPI_InvalidParameter;
    }

    // Check if driver does not idle
    if (dspiState->status != kStatus_DSPI_Success)
    {
        return dspiState->status;
    }

    dspiState->isSync = false;

    DSPI_DRV_EdmaSlaveStartTransfer(instance, sendBuffer, receiveBuffer, transferByteCount);

    return kStatus_DSPI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaSlaveTransferBlocking
 * Description   : Transfer data - blocking
 * Transfer data - blocking
 *
 *END**************************************************************************/
dspi_status_t DSPI_DRV_EdmaSlaveTransferBlocking(uint32_t instance,
                                                 const uint8_t *sendBuffer,
                                                 uint8_t *receiveBuffer,
                                                 uint32_t transferByteCount,
                                                 uint32_t timeOut)
{
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    dspi_status_t result = kStatus_DSPI_Success;
    event_flags_t setFlags = 0;
    osa_status_t osaStatus = kStatus_OSA_Success;

    // Check if DSPI is not initialized
    if (!dspiState)
    {
        return kStatus_DSPI_NonInit;
    }

    // Check if buffers and length is empty
    if (((!sendBuffer) && (!receiveBuffer)) ||
        (!transferByteCount))
    {
        return kStatus_DSPI_InvalidParameter;
    }

    // Check if driver does not idle
    if (dspiState->status != kStatus_DSPI_Success)
    {
        return dspiState->status;
    }

    dspiState->isSync = true;

    DSPI_DRV_EdmaSlaveStartTransfer(instance, sendBuffer, receiveBuffer, transferByteCount);

    // wait transfer finished
    do
    {
        osaStatus = OSA_EventWait(&dspiState->event, kDspiEdmaTransferDone, true, timeOut, &setFlags);
    } while(osaStatus == kStatus_OSA_Idle);

    // Check status of OSA wait event
    switch (osaStatus)
    {
        case kStatus_OSA_Success:
            result = kStatus_DSPI_Success;
            break;
        case kStatus_OSA_Timeout:
            result = kStatus_DSPI_Timeout;
            break;
        case kStatus_OSA_Error:
        default:
            result = kStatus_DSPI_Error;
            break;
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaRxCallback
 * Description   : Callback function, this called when eDMA receiving data
 * completed
 *
 *END**************************************************************************/
static void DSPI_DRV_EdmaRxCallback(void *param, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)param;
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    uint32_t edmaBaseAddr = VIRTUAL_CHN_TO_EDMA_MODULE_REGBASE(dspiState->edmaRxChannel.channel);
    uint32_t edmaChannel = VIRTUAL_CHN_TO_EDMA_CHN(dspiState->edmaRxChannel.channel);

    // Disable DMA major loop interrupt
    EDMA_HAL_HTCDSetIntCmd(edmaBaseAddr, edmaChannel, false);

    // Stop DMA channel.
    EDMA_HAL_SetDmaRequestCmd(edmaBaseAddr, (edma_channel_indicator_t)edmaChannel, false);

    // Disable Rx Fifo drain interrupt
    DSPI_HAL_SetRxFifoDrainDmaIntMode(baseAddr, kDspiGenerateDmaReq, false);

    // If transmission completed, stop the transferring
    if ((dspiState->isTransferInProgress) &&
        (dspiState->remainingSendByteCount <= 0) &&
        (dspiState->extraReceiveByte == 0))
    {
        DSPI_DRV_EdmaCompleteTransfer(instance);

        // Notify event
        if(dspiState->isSync)
        {
            OSA_EventSet(&dspiState->event, kDspiEdmaTransferDone);
        }
    }
    else if (dspiState->extraReceiveByte != 0)
    {
        // Enable transmit complete interrupt
        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);
        DSPI_HAL_SetIntMode(baseAddr, kDspiTxComplete, true);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_EdmaTxCallback
 * Description   : Callback function, this called when eDMA transmitting data
 * completed
 *
 *END**************************************************************************/
static void DSPI_DRV_EdmaTxCallback(void *param, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)param;
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    uint32_t edmaBaseAddr = VIRTUAL_CHN_TO_EDMA_MODULE_REGBASE(dspiState->edmaTxChannel.channel);
    uint32_t edmaChannel = VIRTUAL_CHN_TO_EDMA_CHN(dspiState->edmaTxChannel.channel);
    uint8_t nBytes;

    nBytes = dspiState->bitsPerFrame / 8;
    if (dspiState->bitsPerFrame % 8 != 0)
    {
        nBytes += 1;
    }

    // Disable DMA major loop interrupt
    EDMA_HAL_HTCDSetIntCmd(edmaBaseAddr, edmaChannel, false);

    // Stop DMA channel.
    EDMA_HAL_SetDmaRequestCmd(edmaBaseAddr, (edma_channel_indicator_t)edmaChannel, false);

    // Disable Tx Fifo fill interrupt
    DSPI_HAL_SetTxFifoFillDmaIntMode(baseAddr, kDspiGenerateDmaReq, false);

    // Check if send bytes count is greater than FIFO size, update this count and enable
    // transmit completed interrupt.
    if (dspiState->remainingSendByteCount > (FSL_FEATURE_DSPI_FIFO_SIZEn(instance) * nBytes))
    {
        // Enable transmit complete interrupt
        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);
        DSPI_HAL_SetIntMode(baseAddr, kDspiTxComplete, true);
    }
}

/*!
 * @brief DSPI slave interrupt handler
 * @details This function is DSPI slave interrupt handler using eDMA mechanism. The
 *          pupose of this interrupt handler is indicates when the transfer is really
 *          finished. The eDMA only used to copy data from/to RX FIFO/TX FIFO, but it
 *          not sure the data was transmitted to the master. So must have to enable
 *          this interrupt to do it. This interrupt only be enabled when the last
 *          four FIFO will be transmitted.
 *
 * @param instance The SPI number
 */
void DSPI_DRV_EdmaSlaveIRQHandler(uint32_t instance)
{
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    dspi_edma_slave_state_t * dspiState = (dspi_edma_slave_state_t *)g_dspiStatePtr[instance];
    uint8_t nBytes;

    nBytes = dspiState->bitsPerFrame / 8;
    if (dspiState->bitsPerFrame % 8 != 0)
    {
        nBytes += 1;
    }

    // Catch Tx complete interrupt
    if ((DSPI_HAL_GetIntMode(baseAddr, kDspiTxComplete)) &&
        (DSPI_HAL_GetStatusFlag(baseAddr, kDspiTxComplete)))
    {
        // Clear this flag first
        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);

        // Check if number of transfered bytes is greater or equals user request
        if(dspiState->remainingSendByteCount <= (DSPI_HAL_GetTransferCount(baseAddr) * nBytes))
        {
            // Complete the transfer
            DSPI_DRV_EdmaCompleteTransfer(instance);
            // Notify to wait task
            if(dspiState->isSync)
            {
                OSA_EventSet(&dspiState->event, kDspiEdmaTransferDone);
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
