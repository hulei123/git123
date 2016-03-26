Target Projects {#target_projects}
=====

# Introduction

The bootloader is built separately for each target device and board combination. This allows for compile-time customization of the bootloader.

# Directory Structure

Target projects are stored in the top level `targets` directory. There is a subdirectory for each device. Inside the device directory is a directory for each toolchain and a `src` directory.

# Sources

Each target must have several common source files:

- `bootloader_config.h`

  Configuration macros, see below.
- `clock_config_x.c`

  Clock configuration code.
- `hardware_init_x.c`

  Board-specific hardware related functions.
- `memory_map_x.c`

  Contains the memory map for the device.
- `peripherals_x.c`

  Contains the peripheral descriptor array.

(The "x" in the filenames above is replaced with the device name.)

# Configuration options

The `bootloader_config.h` file in the target source directory sets a number of macros used to configure the bootloader build.

`BL_MIN_PROFILE`

Selects between the maximal and minimal configuration for the bootloader. Set to 1 for the minimal configuration.

`BL_FEATURE_READ_MEMORY`

Forces inclusion of read memory command under minimal configuration.

`BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT`

The default active peripheral detection timeout in milliseconds.

`BL_HAS_MASS_ERASE`

Set to 1 if the device has the ERSALLU flash controller command. Only certain new devices have this command.

`BL_FLASH_VERIFY_DISABLE`

Set to 1 to disable verification of erases and writes to flash memory.

`BL_ENABLE_CRC_CHECK`

Enable application CRC check on targets with hardware CRC support.

`BL_UART_AUTOBAUD_IRQ`

Set to 1 to use pin interrupts for autobaud, rather than polling.

`BL_APP_VECTOR_TABLE_ADDRESS`

The bootloader will check this address for the application vector table upon startup.

`BL_FEATURE_POWERDOWN`

Enables entering low power mode if no host communication.

`BL_DEFAULT_POWERDOWN_TIMEOUT`

Sets timeout for entering low power mode.

The following macros control the inclusion of specific host communication peripherals in the build. (See `peripherals_x.c`.)

`BL_CONFIG_SCUART`

`BL_CONFIG_I2C`

`BL_CONFIG_DSPI`

`BL_CONFIG_USB_HID`

