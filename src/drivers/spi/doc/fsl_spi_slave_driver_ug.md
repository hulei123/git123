SPI Slave Driver {#spislave}
================
@ingroup spi_slave_driver

# Overview

The SPI slave peripheral driver provides an easy way to use a SPI peripheral in slave mode.

Data transfer is performed entirely through a couple callback functions.

# Callbacks

To use this driver, you first need to define a few application callbacks. These are the callbacks
functions that are set in the #spi_slave_callbacks_t struct.

The two callbacks are:
* Data source
* Data sink

The two callbacks are for sending and receiving data. 

The prototypes for the three callbacks are:
~~~~~{.c}
    status_t dataSource(uint8_t * sourceByte, uint16_t instance);
    
    status_t dataSink(uint8_t sinkByte, uint16_t instance);
~~~~~

All callbacks are invoked from IRQ state.

# Setup

To initialize the SPI slave driver, first create and fill in a #spi_slave_config_t struct. This
struct defines the callbacks as well as the data format settings for the SPI peripheral.
The struct is not required after the driver is initialized, so it can be allocated on the stack.

Here is an example config struct definition:
~~~~~{.c}
    spi_slave_config_t config = {
            {
                appDataSource,
                appDataSink,
            },
            kSpiClockPhase_FirstEdge,
            kSpiClockPolarity_ActiveHigh,
            kSpiLsbFirst
        };
~~~~~



