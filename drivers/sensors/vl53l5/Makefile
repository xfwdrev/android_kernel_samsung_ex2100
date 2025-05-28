#
# This file is part of VL53L5 Kernel Driver
#
# Copyright (C) 2020, STMicroelectronics - All Rights Reserved
#
# License terms: STMicroelectronics Proprietary in accordance with licensing
# terms at www.st.com/sla0081
#
# STMicroelectronics confidential
# Reproduction and Communication of this document is strictly prohibited unless
# specifically authorized in writing by STMicroelectronics.

# Parameters
# Configure the build type : RELEASE, DEBUG
BUILD_TYPE = RELEASE
# Specify the comms type : SPI, I2C
COMMS_TYPE = SPI
# Specify the I2C bus adapter number for use with COMMS_TYPE=I2C
I2C_ADAPTER_NUM := 1
# Specify the SPI comms speed in KHz for use with COMMS_TYPE=SPI
COMMS_SPEED_KHZ := 7000
# Specify the GPIO pin number for SPI chip select.
GPIO_CHIP_SELECT := 2
# Specify the GPIO pin number for VL53L5 comms select. Set to -1 to ignore.
GPIO_COMMS_SELECT := 23
# Specify the GPIO pin number for VL53L5 power enable. Set to -1 to ignore.
GPIO_POWER_ENABLE := 12
# Specify the GPIO pin number for VL53L5 low power. Set to -1 to ignore.
# Note: Low power mode is not currently available. However, this pin must be
#       set high in order to enable power.
GPIO_LOW_POWER := 19

# Sets the servicing mode to use i.e. INTERRUPT, WORKER or DEFAULT
RANGE_SERVICING = DEFAULT

# Set to TRUE to enable TCDM dump
TCDM_DUMP_ENABLE = FALSE

# Set to TRUE to enable offset calibration
OFFSET_CAL_ENABLE = FALSE

# Set to TRUE to enable code for internal testing
LEGACY_CODE = FALSE

# Set to TRUE to enable stress test functions.
TEST_ENABLE = FALSE

# Set to TRUE to enable single zone functions.
PSEUDO_SINGLE_ZONE = TRUE

ifeq "$(RANGE_SERVICING)" "INTERRUPT"
GPIO_INTERRUPT := 16
else
GPIO_INTERRUPT := -1
endif

# Set to TRUE to enable logging functions.
LOG_ENABLE = TRUE

ifeq "$(LOG_ENABLE)" "TRUE"
  $(warning LOG_ENABLE=$(LOG_ENABLE))
  ccflags-y += -DVL53L5_KERNEL_LOG
endif

#KDIR = /lib/modules/$(shell uname -r)/build

ccflags-y	+= -I$(srctree)/drivers/sensors/vl53l5/inc
ccflags-y	+= -I$(srctree)/drivers/sensors/vl53l5/bare_driver/common/inc
ccflags-y	+= -I$(srctree)/drivers/sensors/vl53l5/bare_driver/api/inc
ccflags-y	+= -I$(srctree)/drivers/sensors/vl53l5/bare_driver/dci/inc
ccflags-y	+= -I$(srctree)/drivers/sensors/vl53l5/platform/inc
ccflags-y	+= -I$(srctree)/drivers/sensors/vl53l5/algo/inc
ccflags-y	+= -Wall -Werror -Wno-missing-braces

ifeq "$(OFFSET_CAL_ENABLE)" "TRUE"
  $(warning OFFSET_CAL_ENABLE=$(OFFSET_CAL_ENABLE))
  ccflags-y += -DVL53L5_OFFSET_CAL
endif

ifeq "$(RANGE_SERVICING)" "WORKER"
ccflags-y += -DVL53L5_WORKER
endif

ifeq "$(BUILD_TYPE)" "DEBUG"
  $(warning BUILD_TYPE=$(BUILD_TYPE))
  ccflags-y += -DDEBUG
endif

ifeq "$(BUILD_TYPE)" "RELEASE"
  $(warning BUILD_TYPE=$(BUILD_TYPE))
endif

ifeq "$(LOG_ENABLE)" "TRUE"
  $(warning LOG_ENABLE=$(LOG_ENABLE))
  ccflags-y += -DVL53L5_KERNEL_LOG

endif

ifeq "$(LEGACY_CODE)" "TRUE"
  $(warning LEGACY_CODE=$(LEGACY_CODE))
  ccflags-y += -DSTM_VL53L5_SUPPORT_LEGACY_CODE
endif


ifeq "$(PSEUDO_SINGLE_ZONE)" "TRUE"
  $(warning PSEUDO_SINGLE_ZONE=$(PSEUDO_SINGLE_ZONE))
  ccflags-y += -DSTM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
endif

KERNEL_DRIVER_OBJS = 	src/vl53l5_k_ioctl_controls.o \
			src/vl53l5_k_gpio_utils.o \
			src/vl53l5_k_error_converter.o \
			src/vl53l5_k_range_wait_handler.o

PLATFORM_OBJS =		platform/src/vl53l5_platform.o \
			platform/src/vl53l5_platform_init.o \
			platform/src/vl53l5_platform_maths.o \
			platform/src/vl53l5_platform_log.o

BARE_DRIVER_OBJS = 	bare_driver/api/src/vl53l5_api_core.o \
			bare_driver/api/src/vl53l5_api_power.o \
			bare_driver/api/src/vl53l5_api_ranging.o \
			bare_driver/api/src/vl53l5_api_range_decode.o \
			bare_driver/dci/src/vl53l5_core_decode.o\
			bare_driver/dci/src/vl53l5_calibration_decode.o\
			bare_driver/dci/src/vl53l5_dci_core.o \
			bare_driver/dci/src/vl53l5_dci_decode.o \
			bare_driver/dci/src/vl53l5_dci_helpers.o \
			bare_driver/dci/src/vl53l5_dci_ranging.o \
			bare_driver/dci/src/vl53l5_dci_utils.o \
			bare_driver/dci/src/vl53l5_decode_switch.o\
			bare_driver/dci/src/vl53l5_results_decode.o \
			bare_driver/common/src/vl53l5_checks.o \
			bare_driver/common/src/vl53l5_commands.o \
			bare_driver/common/src/vl53l5_error_handler.o \
			bare_driver/common/src/vl53l5_load_firmware.o \
			bare_driver/common/src/vl53l5_register_utils.o \
			bare_driver/common/src/vl53l5_rom_boot.o

ALGO_OBJS = 		algo/src/vl53l5_output_target_filter_main.o\
			algo/src/vl53l5_output_target_filter_funcs.o\
			algo/src/vl53l5_output_target_filter.o\
			algo/src/vl53l5_long_tail_filter_main.o \
			algo/src/vl53l5_long_tail_filter__set_cfg_main.o\
			algo/src/vl53l5_long_tail_filter_funcs.o \
			algo/src/vl53l5_long_tail_filter.o \
			algo/src/vl53l5_long_tail_filter__set_cfg.o


PSEUDO_SINGLE_ZONE_OBJS =  algo/src/vl53l5_pseudo_single_zone.o\
			algo/src/vl53l5_pseudo_single_zone__set_cfg.o\
			algo/src/vl53l5_pseudo_single_zone_main.o\
			algo/src/vl53l5_pseudo_single_zone__set_cfg_main.o\
			algo/src/vl53l5_pseudo_single_zone_funcs.o

# SIMULATION_OBJS =	simulation/wrapper/vl53l5_api.o

ifeq "$(TCDM_DUMP_ENABLE)" "TRUE"
  $(warning TCDM_DUMP_ENABLE=$(TCDM_DUMP_ENABLE))
  ccflags-y += -DVL53L5_TCDM_ENABLE
  BARE_DRIVER_OBJS += bare_driver/common/src/vl53l5_tcdm_dump.o
endif

ifeq "$(TEST_ENABLE)" "TRUE"
  $(warning TEST_ENABLE=$(TEST_ENABLE))
  ccflags-y += -DVL53L5_TEST_ENABLE

endif

ifeq "$(RANGE_SERVICING)" "INTERRUPT"
  $(warning RANGE_SERVICING=$(RANGE_SERVICING))
  ccflags-y	+= -DVL53L5_INTERRUPT
  KERNEL_DRIVER_OBJS += src/vl53l5_k_interrupt.o
endif

ifeq "$(RANGE_SERVICING)" "WORKER"
  $(warning RANGE_SERVICING=$(RANGE_SERVICING))
  ccflags-y	+= -DVL53L5_WORKER
  KERNEL_DRIVER_OBJS += src/vl53l5_k_work_handler.o
endif


MODULE := vl53l5
# CONFIG_STMVL53L5 := m

obj-$(CONFIG_SENSORS_VL53L5)	+= $(MODULE).o
$(MODULE)-objs		:= src/vl53l5_k_module.o

# # Kernel level objects
$(MODULE)-objs	+= 	$(KERNEL_DRIVER_OBJS)
$(MODULE)-objs +=	$(ALGO_OBJS)
# Bare driver objects
$(MODULE)-objs +=	$(BARE_DRIVER_OBJS)
# Bare driver platform objects
$(MODULE)-objs +=	$(PLATFORM_OBJS)
ifeq "$(PSEUDO_SINGLE_ZONE)" "TRUE"
$(MODULE)-objs +=	$(PSEUDO_SINGLE_ZONE_OBJS)
endif


