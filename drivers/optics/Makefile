#
# Makefile for the optics drivers.
#

# Each configuration option enables a list of files.

KBUILD_CFLAGS += -Wno-unused-variable -Wno-unused-function -Wno-unused-label -Wno-unused-parameter -Wno-frame-larger-than=

obj-$(CONFIG_SENSORS_TSL2510)		+= tsl2510.o
obj-$(CONFIG_SENSORS_FLICKER_SELF_TEST) += flicker_test.o
obj-$(CONFIG_SENSORS_STK3A8X)   += stk.o
stk-src += stk3a8x.c stk3a8x_drv_i2c.c stk3a8x_calibration.c stk3a8x_fifo.c
stk-objs := $(stk-src:.c=.o) #stk_otp_encrypt.o
