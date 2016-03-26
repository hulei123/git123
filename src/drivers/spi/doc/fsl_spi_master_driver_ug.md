SPI Master Driver {#spimaster}
================
@ingroup spi_master_driver

# Overview

The SPI master mode peripheral driver is used to transfer data to and from external devices on
the SPI bus in master mode. It provides an easy way to transfer buffers of data with a single
function call.

# Device structs

The driver uses instances of the #spi_device_t structure to represent the SPI bus configuration
required to communicate to an external device that is connected to the bus.

The device struct can be passed to the data transfer functions, and the bus will be reconfigured
before the transfer is started. Alternatively, you can configure the SPI bus for a device manually.
For instance, if there is only one device connected to the bus, you might configure it only once.

# Initialization

To initialize the SPI master driver, simply call spi_master_init() and pass the instance number
of the SPI peripheral you want to use. For instance, to use SPI1 pass a value of 1 to the init
function.

If you only 

Example code for initializing and configuring the driver:
~~~~~{.c}
    // Define bus configuration.
    spi_device_t device = {
            .busFrequencyKHz = 1000,
            .polarity = kSpiClockPolarity_ActiveHigh,
            .phase = kSpiClockPhase_SecondEdge,
            .direction = kSpiMsbFirst
        };
    
    // Init driver.
    spi_master_init(0);
    
    // Configure bus.
    spi_master_configure_bus(0, &device);
~~~~~

# Transfers

The driver supports two different modes for transferring data, both blocking and nonblocking. The
blocking transfer function is spi_master_transfer().

Example of a blocking transfer:
~~~~~{.c}
    uint8_t sendBuf[] = { 0, 1, 2, 3 };
    uint8_t receiveBuf[sizeof(sendBuf)];
    
    spi_master_transfer(0,  // SPI0
        &device,            // Device SPI bus configuration
        sendBuf,            // Data to send
        receiveBuf,         // Buffer to hold received data
        sizeof(sendBuf),    // Number of bytes to transfer
        kSpiWaitForever);   // No timeout
~~~~~

Example of a nonblocking transfer:
~~~~~{.c}
    uint8_t sendBuf[] = { 0, 1, 2, 3 };
    uint8_t receiveBuf[sizeof(sendBuf)];
    
    // Start the transfer.
    spi_master_transfer_async(0,  // SPI0
        &device,            // Device SPI bus configuration
        sendBuf,            // Data to send
        receiveBuf,         // Buffer to hold received data
        sizeof(sendBuf));   // Number of bytes to transfer
    
    // Wait for the transfer to complete.
    while (spi_master_get_transfer_status(0, NULL) == kStatus_SPI_Busy)
    {
    }
~~~~~


