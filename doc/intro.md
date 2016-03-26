Introduction {#index}
=====

Getting Started
-----
For each supported device there is a demo application users guide available in the package. This guide includes information about the tool chain required to build the bootloader and a guide to getting started using the supported board platform.

Features
-----
The bootloader code base contains source code and tool chain projects for building flash-resident bootloaders for the supported platforms. A flash-resident bootloader stays resident in flash along with the user application. The bootloader can be used to download and program an initial application image into a blank area on the flash, and to later to update the application.

The bootloader supports the following communication interfaces for downloading an application:
- USB HID
- UART
- I2C
- SPI
- CAN (on selected devices)

Typically, USB HID and UART connections are made directly to a PC, whereas I2C, SPI and CAN require additional hardware. See the `apps/bus_pal` project for examples of an embedded host using Freescale platforms. The bootloader, running on the target platform, acts as a communication slave. The bootloader can automatically detect which peripheral is being used to download the application and, in the case of UART, automatically detect the baud rate.

The application image is downloaded to the target through a series of command and data packets sent from a host PC or embedded host platform. For more information on the packet protocol, see the code documentation in the doc directory.

Host tools
-----
The code base also contains source code and build projects for the following PC-based host tools:

- Kinetis blhost - command line debug tool to send individual commands to the bootloader
- Kinetis Updater - GUI application to download and flash an application image

The blhost application runs on Windows 8/7/XP, Linux, and Mac OS X. The KinetisUpdater application runs on Windows 8/7. 
