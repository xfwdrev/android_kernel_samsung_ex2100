TARGET = slsi_ts

ifeq ($(CONFIG_SAMSUNG_PRODUCT_SHIP), y)
  $(TARGET)-objs := slsi_core.o slsi_fn.o slsi_fw.o slsi_cmd.o slsi_dump.o slsi_ioctrl.o
  obj-$(CONFIG_TOUCHSCREEN_SLSI) += $(TARGET).o
else
  $(TARGET)-objs := slsi_core.o slsi_fn.o slsi_fw.o slsi_cmd.o slsi_dump.o slsi_ioctrl.o slsi_vendor.o
ifeq ($(CONFIG_SEC_KUNIT), y)
endif
  obj-$(CONFIG_TOUCHSCREEN_SLSI) += $(TARGET).o
endif

GCOV_PROFILE_slsi_core.o := $(CONFIG_SEC_KUNIT)
GCOV_PROFILE_slsi_fn.o := $(CONFIG_SEC_KUNIT)
GCOV_PROFILE_slsi_fw.o := $(CONFIG_SEC_KUNIT)
GCOV_PROFILE_slsi_cmd.o := $(CONFIG_SEC_KUNIT)
GCOV_PROFILE_slsi_dump.o := $(CONFIG_SEC_KUNIT)
GCOV_PROFILE_slsi_ioctrl.o := $(CONFIG_SEC_KUNIT)
GCOV_PROFILE_slsi_vendor.o := $(CONFIG_SEC_KUNIT)

ccflags-y += -Wformat
