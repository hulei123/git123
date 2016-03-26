DSPI_DRIVER_DIR := $(SDK_ROOT)/platform/drivers/dspi
SOURCES += $(wildcard $(DSPI_DRIVER_DIR)/hal/*.c) \
           $(wildcard $(DSPI_DRIVER_DIR)/src/*.c)
