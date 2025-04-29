obj-$(CONFIG_CHARGER_PCA9468)	+= pca9468-charger.o
pca9468-charger-$(CONFIG_CHARGER_PCA9468)	+= pca9468_charger.o

ifeq ($(CONFIG_SEC_KUNIT), y)
 GCOV_PROFILE_pca9468_charger.o := $(CONFIG_SEC_KUNIT)
endif
ccflags-y := -Wformat
