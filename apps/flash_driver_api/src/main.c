/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#include <stdint.h>
#include <stdio.h>
#include "application_common.h"
#include "flash.h"

#define TEST_ERASE_SECTOR_START_ADDRESSS    0x4000u
#define FLASH_ERASE_SIZE                    0x400

#define BOOTLOADER_TREE_LOCATION    (0x1c00001cul)


typedef union BootloaderVersion
{
    struct {
        uint32_t bugfix : 8;     //!< bugfix version [7:0]
        uint32_t minor : 8;      //!< minor version [15:8]
        uint32_t major : 8;      //!< major version [23:16]
        uint32_t name : 8;       //!< name [31:24]
    }B;
    uint32_t version;   //!< combined version numbers
} standard_version_t;



const uint32_t test_program_buffer[2] = {0x01234567, 0x89abcdef};

void flash_driver_api_tree_demo(void);

void verify_status(status_t status)
{
    char *tipString = "Unknown status";;
    switch (status)
    {
    case kStatus_Success:
        tipString = "Done.";
        break;
    case kStatus_FlashApiNotSupported:
        tipString = "This API is not supported in current target.";
        break;
    default:
        break;
    }

    printf("%s\r\n\r\n", tipString);
}

int main()
{
    // Initialize hardware
    init_hardware();
    // Initialize terminal uart
    init_term_uart();

    // Run flash driver api tree demo.
    flash_driver_api_tree_demo();

    while(1);
}

void flash_driver_api_tree_demo(void)
{
    flash_driver_t flashInstance;
    // Print basic information for Flash Driver API.
    printf("\r\nFlash driver API tree Demo Application...\r\n");
    printf("Getting bootloader tree location from address 0x%X\r\n", BOOTLOADER_TREE_LOCATION);
    standard_version_t flashDriverVersion;
    // Initialize flash driver
    printf("Initializing flash driver...\r\n");
    if (flash_init(&flashInstance) == kStatus_Success)
    {
        uint32_t value;
        status_t status;
        uint32_t * failedAddress =0;
        uint32_t * failedData = 0;

        printf("Done!\r\n");

        // Get flash properties
        printf("Flash Properties:\r\n");
        flash_get_property(&flashInstance, kFlashProperty_Version, &flashDriverVersion.version);
        printf("\tkFlashProperty_Version = %c%d.%d.%d\r\n", flashDriverVersion.B.name,
               flashDriverVersion.B.major, flashDriverVersion.B.minor, flashDriverVersion.B.bugfix);
        flash_get_property(&flashInstance, kFlashProperty_SectorSize, &value);
        printf("\tkFlashProperty_SectorSize = %d\r\n", value);
        flash_get_property(&flashInstance, kFlashProperty_TotalFlashSize, &value);
        printf("\tkFlashProperty_TotalFlashSize = %d\r\n", value);
        flash_get_property(&flashInstance, kFlashProperty_BlockCount, &value);
        printf("\tkFlashProperty_BlockCount = %d\r\n", value);
        flash_get_property(&flashInstance, kFlashProperty_FlashBlockBaseAddr, &value);
        printf("\tkFlashProperty_FlashBlockBaseAddr = 0x%X\r\n", value);
        status = flash_get_property(&flashInstance, kFlashProperty_FlashFacSupport, &value);
        if (status == kStatus_Success)
        {
            printf("\tFlashProperty_FlashFacSupport = 0x%x\r\n", value);
        }
        else
        {
            printf("\tProperty: FlashProperty_FlashFacSupport is not supported.\r\n");
        }
        status = flash_get_property(&flashInstance, kFlashProperty_FlashAccessSegmentSize, &value);
        if (status == kStatus_Success)
        {
            printf("\tFlashProperty_FlashAccessSegmentSize = 0x%x\r\n", value);
        }
        else
        {
            printf("\tProperty: FlashProperty_FlashAccessSegmentSize is not supported.\r\n");
        }

        flash_get_property(&flashInstance, kFlashProperty_FlashAccessSegmentSize, &value);
        if (status == kStatus_Success)
        {
            printf("\tFlashProperty_FlashAccessSegmentCount = 0x%x\r\n", value);
        }
        else
        {
            printf("\tProperty: FlashProperty_FlashAccessSegmentCount is not supported.\r\n");
        }

        // Erase a given flash range
        printf("\r\nCalling flash_erase() API...\r\n");
        status = flash_erase(&flashInstance, TEST_ERASE_SECTOR_START_ADDRESSS, FLASH_ERASE_SIZE, kFlashEraseKey);
        verify_status(status);

        // Verify if the given flash range is successfully erased.
        printf("Calling flash_verify_erase() API...\r\n");
        status = flash_verify_erase(&flashInstance, TEST_ERASE_SECTOR_START_ADDRESSS, FLASH_ERASE_SIZE, kFlashMargin_User);
        verify_status(status);

        // Start programming specified flash region
        printf("Calling flash_program() API...\r\n");
        status = flash_program(&flashInstance, TEST_ERASE_SECTOR_START_ADDRESSS, (uint32_t *)test_program_buffer, 8);
        verify_status(status);

        // Verify if the given flash region is successfully programmed with given data
        printf("Calling flash_verify_program() API...\r\n");
        status = flash_verify_program(&flashInstance, TEST_ERASE_SECTOR_START_ADDRESSS,
                                                         8, (uint8_t*)&test_program_buffer, kFlashMargin_User,
                                                         failedAddress, failedData);
        verify_status(status);

        // Start Program specified Program Once Field
        /*
        Note: Because the Program Once Field is only support being programmed once,
        Please uncomment follow codes if you are aware of the result.
        */
//        uint32_t temp = 0x12345678;
//        printf("Calling flash_program_once() API ...\r\n");
//        status = flash_program_once(&flashInstance, 0, &temp, sizeof(temp));
//        verify_status(status);

        // Start Read specified Program Once Field

        uint32_t readData[2];
        printf("Calling flash_read_once() API ...\r\n");
        status = flash_read_once(&flashInstance, 0, &readData[0], sizeof(uint32_t));
        verify_status(status);

        // Start running flash_read_resource API
        printf("Calling flash_read_resource() API ...\r\n");
        status = flash_read_resource(&flashInstance, 0,
                                     &readData[0], 8, kFlashResource_VersionID);
        verify_status(status);
        if (status == kStatus_Success)
        {
            printf ("Data read from Program Once Field: 0x%08x, 0x%08x\r\n",
                    readData[0], readData[1]);
        }

        // Start perform flash_register_callback API
        printf("Calling flash_register_callback() API ...\r\n");
        status = flash_register_callback(&flashInstance, NULL);
        verify_status(status);
    }
    else
    {
        printf("Flash init failure. Halting...\r\n");
    }

    printf("Done!\r\n");
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////




