Peripheral Interfaces {#peripheral_interfaces_page}
=====

# Overview

The block diagram below shows the connections between components in the architecture of the peripheral interface.

![peripheral interfaces block diagram](../../doc/image/peripheral_interfaces_block_diagram.png)

In this diagram, the byte and packet interfaces are shown to inherit from the control interface.

All peripheral drivers implement an abstract interface built on top of the driver's internal interface. The outermost abstract interface is a packet-level interface. It returns the payload of packets to the caller. Drivers which use framing packets have another abstract interface layer that operates at the byte level. The abstract interfaces allow the higher layers to use exactly the same code regardless which peripheral is being used.

The abstract packet interface feeds into the command and data packet processor. This component interprets the packets returned by the lower layer as command or data packets.

# Abstract control interface

This control interface provides a common method to initialize and shutdown peripheral drivers. It also provides the means to perform the active peripheral detection. No data transfer functionality is provided by this interface. That is handled by the interfaces that inherit the control interface.

The main reason this interface is separated out from the byte and packet interfaces is to show that it is common between the two. It also allows the driver to provide a single control interface struct definition that can be easily shared.

~~~~~{.c}
struct PeripheralDescriptor {
    uint32_t typeMask;
    uint32_t instance;
    void (*pinmuxConfig)(uint32_t instance, pinmux_type_t pinmux);
    const peripheral_control_interface_t * controlInterface;
    const peripheral_byte_inteface_t * byteInterface;
    const peripheral_packet_interface_t * packetInterface;
};

typedef struct _peripheral_control_interface
{
    bool (*pollForActivity)(const peripheral_descriptor_t * self);
    status_t (*init)(const peripheral_descriptor_t * self, serial_byte_receive_func_t function);
    void (*shutdown)(const peripheral_descriptor_t * self);
    void (*pump)(const peripheral_descriptor_t * self);
} peripheral_control_interface_t;
~~~~~

Interface | Description
--------- | -----------
`pollForActivity()` | Check whether communications has started.
`init()` | Fully initialize the driver.
`shutdown()` | Shutdown the fully initialized driver.
`pump()` | Give the peripheral interface some processing time.

# Abstract byte interface

This interface exists to give the framing packetizer a common interface to talk to for the periperals that use framing packets.

The abstract byte interface inherits the abstract control interface.

~~~~~{.c}
typedef struct _peripheral_byte_inteface
{
    status_t (*init)(const peripheral_descriptor_t * self);
    status_t (*read)(const peripheral_descriptor_t * self, uint8_t * buffer, uint32_t requestedBytes);
    status_t (*write)(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);
} peripheral_byte_inteface_t;
~~~~~

Interface | Description
--------- | -----------
`init()` | Initialize the interface.
`read()` | Return the requested number of bytes. Blocks until all bytes available.
`write()` | Write the requested number of bytes.


# Abstract packet interface

The abstract packet interface inherits the abstract control interface.

~~~~~{.c}
typedef struct _peripheral_packet_interface
{
    status_t (*init)(const peripheral_descriptor_t * self);
    status_t (*readPacket)(const peripheral_descriptor_t * self, uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType);
    status_t (*writePacket)(const peripheral_descriptor_t * self, const uint8_t * packet, uint32_t byteCount, packet_type_t packetType);
    void (*abortDataPhase)(const peripheral_descriptor_t * self);
    status_t (*finalize)(const peripheral_descriptor_t * self);
    uint32_t (*getMaxPacketSize)(const peripheral_descriptor_t * self);
    void (*byteReceivedCallback)(uint8_t byte);
} peripheral_packet_interface_t;
~~~~~

Interface | Description
--------- | -----------
`init()` | Initialize the peripheral.
`readPacket()` | Read a full packet from the peripheral.
`writePacket()` | Send a complete packet out the peripheral.
`abortDataPhase()` | Abort receiving of data packets.
`finalize()` | Shut down the peripheral when done with use.
`getMaxPacketSize()` | Returns the maximum buffer size.
`byteReceivedCallback()` | A byte was received on the interface.


## Framing packetizer

The [framing packetizer](@ref packet) processes framing packets received via the byte interface with which it talks. It builds and validates a framing packet as it reads bytes. And it constructs outgoing framing packets as needed to add flow control information and command or data packets. The framing packet also supports data phase abort.

## USB HID packetizer

The [USB HID packetizer](@ref usb_hid_peripheral) implements the abstract packet interface for USB HID, taking advantage of USB's inherent flow control and error detection capabilities. The USB HID packetizer provides a link layer that supports variable length packets and data phase abort.

# Command/data processor

This component reads complete packets from the abstract packet interface and interprets them as either command packets or data packets. The actual handling of each command is done by command handlers that are called by the command processor. The command handler will tell the command processor whether a data phase is expected and how much data is expected to be received.

If the command/data processor receives a unexpected command or data packet, it will ignore it. In this case, the communications link will resynchronize upon reception of the next valid command.
