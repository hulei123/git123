Memory Interface {#memifpage}
=====

# Abstract interface

The bootloader uses a common, abstract interface to implement the memory read/write/fill commands. This is to keep the command layer from having to know the details of the memory map and special routines. This memory interface is documented in detail [here](@ref memif).

A shared `memory_interface_t` struct is used for both the high-level abstract interface, as well as low-level entries in the memory map.

The global bootloader context contains a pointer to the high-level abstract memory interface, which is one of these `memory_interface_t` structs. The internal implementation of this abstract interface uses a memory map table, referenced from the global bootloader context, that describes the various regions of memory that are accessible and provides region-specific operations.

The high-level functions are implemented to iterate over the memory map entries until it finds the entry for the specified address range. Read and write operations are not permitted to cross region boundaries, and an error will be returned if such an attempt is made.

The `bootloader_context_t::memoryMap` member is set to an array of the `memory_map_entry_t` structures. This array must be terminated with an entry with all fields set to zero. The same `memory_interface_t` struct is also used to hold the memory-type-specific operations.

Note that the `memory_map_entry_t::endAddress` field must be set to the address of the last byte of the region, because a <= comparison is used.

Here is an example memory map:

~~~~~{.c}
const memory_map_entry_t g_baseMemoryMap[] = {
        { 0x00000000, 0x0001ffff, &g_hlfdInterface }, // Flash array
        { 0x1ffff000, 0x1fffffff, &g_memInterface },  // SRAM_L
        { 0x20000000, 0x20002fff, &g_memInterface },  // SRAM_U
        { 0x40000000, 0x4007ffff, &g_memInterface },  // AIPS peripherals
        { 0x400ff000, 0x400fffff, &g_memInterface },  // GPIO
        { 0x44000000, 0x5fffffff, &g_memInterface },  // BME
        { 0xe0000000, 0xe00fffff, &g_memInterface },  // M0+ private peripherals
        { 0xf0003000, 0xf0003fff, &g_memInterface },  // MCM
        { 0xf8000000, 0xffffffff, &g_memInterface },  // IOPORT (single-cycle GPIO)
        { 0 } // Terminator
    };
~~~~~

During bootloader startup, the memory map is copied into RAM and modified to match the actual sizes of flash and RAM on the chip, as determined by reading the `SIM_SDID` and `SIM_FCFG1/2` registers.

# Flash Driver Interface

The flash driver uses the common memory interface listed above to simplify the interaction with flash. It also takes care of high level features such as readback verification, flash protection awareness, and so on. The flash memory functions map to the interface functions as so:

~~~~~{.c}
const memory_region_interface_t g_flashMemoryInterface = {
    .read = &normal_mem_read,
    .write = &flash_mem_write,
    .fill = &flash_mem_fill
};
~~~~~

API | Description
--- | -----------
`normal_mem_read()` | Performs a normal memory read.
`flash_mem_write()` | Calls the low-level `flash_program()` API. Also performs program verification if enabled with the Set Property command.
`flash_mem_fill()` | Performs intelligent fill operations on flash memory ranges. If the fill pattern are all 1's then special action is taken. If the range is a whole number of sectors, then those sectors are erased rather than filled. Any part of an all-1's fill that is not sector-aligned and -sized is ignored (the assumption being that it has been erased to 1's already). Fills for patterns other than all 1's call into `flash_program()`.

Both `flash_mem_write()` and `flash_mem_fill()` check the flash protection status for the sectors being programmed or erased and return an appropriate error if the operation is not allowed.

Bootloader startup code is responsible for initializing the flash memory controller.

# Low level flash driver

The [low level flash driver](@ref flash) (LLFD) handles erase and write operations on a word basis. It cannot perform writes of less than a full word.

Bootloader startup code is responsible for initializing and shutting down the LLFD.

~~~~~{.c}
status_t flash_init();
status_t flash_erase_all();
status_t flash_erase(uint32_t start, uint32_t lengthInBytes);
status_t flash_program(uint32_t start, uint32_t * src, uint32_t lengthInBytes);
status_t flash_get_security_state(flash_security_state_t * state);
status_t flash_security_bypass(const uint8_t * backdoorKey);
status_t flash_verify_erase_all(flash_margin_value_t * margin);
status_t flash_verify_erase(uint32_t start, uint32_t lengthInBytes, flash_margin_value_t margin);
status_t flash_verify_program(uint32_t start, uint32_t lengthInBytes,
                              const uint8_t * expectedData, flash_margin_value_t margin,
                              uint32_t failedAddress, uint8_t *failedData);
status_t flash_is_region_protected(uint32_t start, uint32_t lengthInBytes,
                                   flash_protection_state_t * protection_state)
status_t flash_get_property(flash_property_t whichProperty, uint32_t * value)
~~~~~

The Flash Driver API can be called from an external application. For further information see the Flash Driver API example in `apps/flash_driver_api`.
