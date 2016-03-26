DSPI Slave Driver {#dspislave}
================
@ingroup dspi_slave_driver

# Overview

The DSPI slave peripheral driver provides an easy way to use a SPI peripheral in slave mode.

Data transfer is performed entirely through a couple of callback functions.

# Runtime state of the DSPI slave driver

This struct holds data that are used by the DSPI slave peripheral driver to
communicate between the transfer function and the interrupt handler. The user just
needs to pass in the memory for this structure, the driver will take care of filling out
the members.

# Callbacks

To use this driver, you first need to define a few application callbacks. These are the callbacks
functions that are set in the #dspi_slave_callbacks_t struct.

The three callbacks are:
* Data source
* Data sink
* Error notification

The first two callbacks are for sending and receiving data. The third callback is invoked if
an error occurs.

The prototypes for the three callbacks are:
~~~~~{.c}
    status_t data_source(uint8_t * sourceWord, uint32_t instance);
    
    status_t data_sink(uint8_t sinkWord, uint32_t instance);
    
    void on_error(status_t error, uint32_t instance);
~~~~~

All callbacks are invoked from IRQ state.

# Setup

To initialize the DSPI slave driver, first create and fill in a #dspi_slave_user_config_t struct. This
struct defines the callbacks as well as the data format settings for the SPI peripheral.
The struct is not required after the driver is initialized, so it can be allocated on the stack.
You'll also have to pass in memory for the run-time state structure.

Here is an example config struct definition:
~~~~~{.c}
    /* Set up and init the slave  */
    dspi_slave_state_t dspiSlaveState;

    dspi_slave_user_config_t slaveUserConfig;
    slaveUserConfig.callbacks.dataSink = data_sink;
    slaveUserConfig.callbacks.dataSource = data_source;
    slaveUserConfig.callbacks.onError = on_error;
    slaveUserConfig.dataConfig.bitsPerFrame = 16;
    slaveUserConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    slaveUserConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    slaveUserConfig.isModifiedTimingFormatEnabled = false;

    dspi_slave_init(slaveInstance, &slaveUserConfig, &dspiSlaveState);
~~~~~



