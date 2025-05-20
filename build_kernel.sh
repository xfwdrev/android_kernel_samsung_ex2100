#!/bin/bash

export ARCH=arm64
export PLATFORM_VERSION=11
export ANDROID_MAJOR_VERSION=r
export SEC_BUILD_CONF_VENDOR_BUILD_OS=15

make ARCH=arm64 exynos2100-t2sxxx_defconfig
make ARCH=arm64 -j16
