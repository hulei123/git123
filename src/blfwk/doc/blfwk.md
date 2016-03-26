Bootloader Framework {#blfwk}
=====

Introduction
-----
The Bootloader Framework is a library of C++ classes implementing communication from a host PC to bootloader firmware running on a target device. Due to the nature of the underlying bootloader command and data protocol, the Bootloader Frameworks supports all target devices without requiring any device-specific knowledge.

Applications
-----
The Bootloader Framework library is used by the Blhost command line tool and by the KinetisUpdater GUI firmware download tool. It can be built as a library or included as source in a PC or embedded application. The current release includes tool chains to build the library for Windows OS and Mac OS X.

Peripherals
-----
Support from the following PC peripherals is included:

- UART (COM port)
- USB-HID (using custom reports)
- Bus Pal example (UART to I2C/SPI using special hardware)

Host Operations
-----
The basic flow for host operations is as follows:

~~~~~{.c}
// Create a command.
Command * cmd = Command::create(&m_cmdv);

// Initialize a configuration option for a COM port.
config.peripheralType = Peripheral::kHostPeripheralType_UART;
config.comPortName = m_comPort.c_str();
config.comPortSpeed = m_comSpeed;
config.packetTimeoutMs = m_packetTimeoutMs;

// Create a bootloader
Bootloader * bl = new Bootloader(config);

// Send the command to the target
bl->inject(*cmd);

// Clean up
bl->flush();
~~~~~
