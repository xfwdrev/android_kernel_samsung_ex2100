/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef LINUX_RANGE_SENSOR_H
#define LINUX_RANGE_SENSOR_H

#if   defined(__linux__)
#include <linux/types.h>
#else
#include <stdint.h>
#include <sys/types.h>
#endif

#define NUMBER_OF_ZONE   64
#define NUMBER_OF_TARGET  2

struct range_sensor_data_t {
   __s16 range_data[NUMBER_OF_ZONE][NUMBER_OF_TARGET];
   __u8 status[NUMBER_OF_ZONE][NUMBER_OF_TARGET];

};

#endif
