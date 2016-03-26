DSPI Master Driver {#dspimaster}
================
@ingroup dspi_master_driver

# Overview

The DSPI master mode peripheral driver is used to transfer data to and from external devices on
the DSPI bus in master mode. It provides an easy way to transfer buffers of data with a single
function call.

# Run-time state structs

The DSPI master driver uses a run-time state struct dspi_master_state_t to keep track of the current data transfer.
This struct holds data that are used by the DSPI master peripheral driver to
communicate between the transfer function and the interrupt handler. The
interrupt handler also uses this information to keep track of its progress.
The user is only responsible to pass in the memory for this run-time state structure where the
DSPI master driver will take care of filling out the members.

# User config structs

The DSPI master driver uses instances of the user configuration structure dspi_master_user_config_t for the DSPI master driver.
This allows you to configure the most common settings of the DSPI peripheral with a single function call.

# Device structs

The DSPI master driver uses instances of the #dspi_device_t structure to represent the SPI bus configuration
required to communicate to an external device that is connected to the bus.

The device struct can be passed to the data transfer functions, and the bus will be reconfigured
before the transfer is started. Alternatively, you can configure the SPI bus for a device manually.
For instance, if there is only one device connected to the bus, you might configure it only once.

# Initialization

To initialize the DSPI master driver, simply call dspi_master_init() and pass the instance number
of the DSPI peripheral you want to use, the user config of type dspi_master_user_config_t , a pointer
to the variable for retrieving the calculated baud rate (this can be a NULL), and memory allocation
for the run-time state structure used by the master driver to keep track of data transfers.
For instance, to use DSPI1 pass a value of 1 to the init function. The following provides example
for the other parameters. 

Example code for initializing and configuring the driver:
~~~~~{.c}
    /* Set up and init the master  */
    dspi_master_state_t dspiMasterState;
    uint32_t calculatedBaudRate;

    /* configure the members of the user config */
    dspi_master_user_config_t userConfig;
    userConfig.isChipSelectContinuous = false;
    userConfig.isSckContinuous = false;
    userConfig.pcsPolarity = kDspiPcs_ActiveLow;
    userConfig.whichCtar = kDspiCtar0;
    userConfig.whichPcs = kDspiPcs1;

    /* init the dspi module */
    dspi_master_init(masterInstance, &dspiMasterState, &userConfig, &calculatedBaudRate);

    // Define bus configuration.
    dspi_device_t spiDevice;
    spiDevice.dataBusConfig.bitsPerFrame = 16;
    spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
    spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    spiDevice.dataBusConfig.direction = kDspiMsbFirst;
    spiDevice.bitsPerSec = 500000;
    
    /* configure the spi bus */
    dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate); 
~~~~~

# Transfers

The driver supports two different modes for transferring data, both blocking and nonblocking (async). The
blocking transfer function is dspi_master_transfer() and non-blocking is dspi_master_transfer_async() .  

Example of a blocking transfer:
~~~~~{.c}
/* Function prototype */
status_t dspi_master_transfer(dspi_master_state_t * dspiState,
                              const dspi_device_t * restrict device,
                              const uint8_t * sendBuffer,
                              uint8_t * receiveBuffer,
                              uint32_t transferByteCount,
                              uint32_t timeout);

/* Example function call */                              
dspi_master_transfer(&dspiMasterState, NULL, s_dspiSourceBuffer[masterInstance], s_dspiSinkBuffer[masterInstance], 32, 1);

~~~~~

Example of a nonblocking transfer:
~~~~~{.c}
/* Function prototype */
status_t dspi_master_transfer_async(dspi_master_state_t * dspiState,
                                    const dspi_device_t * restrict device,
                                    const uint8_t * sendBuffer,
                                    uint8_t * receiveBuffer,
                                    uint32_t transferByteCount);
/* Example function call */                              
dspi_master_transfer_async(&dspiMasterState, NULL, s_dspiSourceBuffer[masterInstance], s_dspiSinkBuffer[masterInstance], 32);

/* For non-blocking/async transfers, need to check back to get transfer status, for example */
/* Where framesXfer returns the number of frames tranferred */
dspi_master_get_transfer_status(&dspiMasterState, &framesXfer);

~~~~~


