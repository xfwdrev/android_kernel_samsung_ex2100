/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L5 Kernel Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L5 Kernel Driver may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

#ifndef VL53L5_K_MODULE_DATA_H_
#define VL53L5_K_MODULE_DATA_H_

#include <linux/miscdevice.h>
#include "vl53l5_k_user_api.h"
#include "vl53l5_platform_user_data.h"
#include "vl53l5_k_state_def.h"
#include "vl53l5_k_spi_def.h"
#include "vl53l5_k_gpio_def.h"
#include "vl53l5_k_driver_config.h"
#include "vl53l5_ltf_cfg_map.h"
#include "vl53l5_psz_int_map.h"
#include "vl53l5_psz_cfg_map.h"
#include "vl53l5_psz_op_map.h"

#include "vl53l5_otf_cfg_map.h"

#ifdef STM_VL53L5_SUPPORT_SEC_CODE
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_UAPI
#include <uapi/linux/sensor/range_sensor.h>
#endif

#include <linux/sensor/sensors_core.h> //for sec dump
#include <linux/input.h> 	//for input_dev
#endif

struct vl53l5_k_module_t {
	struct miscdevice miscdev;

	struct vl53l5_dev_handle_t stdev;

	struct mutex mutex;

	struct vl53l5_k_spi_data_t spi_info;

	struct vl53l5_k_gpio_settings_t gpio;

	struct vl53l5_ranging_mode_flags_t ranging_flags;

	struct vl53l5_module_info_t m_info;

	struct vl53l5_ltf_cfg_dev_t ltf_cfg;

	struct vl53l5_otf_cfg_dev_t of_cfg;

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	struct common_grp__version_t  ltf_version;

	struct common_grp__version_t  of_version;
#endif

#ifdef STM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
	struct vl53l5_psz_cfg_dev_t sz_cfg;

	struct vl53l5_psz_int_dev_t sz_int;

	struct vl53l5_psz_op_dev_t sz_data;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	struct common_grp__version_t  sz_version;
#endif
#endif
#ifdef VL53L5_INTERRUPT
	struct workqueue_struct *irq_wq;

	struct work_struct data_work;
#endif
#ifdef VL53L5_WORKER
	struct delayed_work dwork;
#endif
	struct list_head reader_list;

	struct range_t {
		struct vl53l5_range_data_t data;

		struct vl53l5_range_results_t rotated;

		int count;

		bool is_valid;
	} range;

	struct calibration_data_t {
		struct vl53l5_version_t version;

		struct vl53l5_module_info_t info;

		struct vl53l5_calibration_dev_t cal_data;
	} calibration;

	wait_queue_head_t wait_queue;

	enum vl53l5_k_state_preset state_preset;

	enum vl53l5_power_states power_state;

	enum vl53l5_range_servicing_mode range_mode;

	unsigned int polling_start_time_ms;

	unsigned int polling_sleep_time_ms;

	int id;

	int comms_type;

	int config_preset;
#ifdef VL53L5_INTERRUPT
	int irq_val;
#endif
	int is_enabled;
#ifdef VL53L5_WORKER
	int worker_is_running;
#endif
	int timeout_occurred;

	int last_driver_error;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	int polling_count;
#endif
	const char *firmware_name;

	char name[64];

	char comms_buffer[VL53L5_COMMS_BUFFER_SIZE_BYTES];

	bool irq_is_active;

	bool irq_is_running;

	bool irq_wq_is_running;

#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	struct device *factory_device;
	struct input_dev *input_dev; //for input_dev
	struct pinctrl_state *pinctrl_vddoff;
	struct pinctrl_state *pinctrl_default;
	struct pinctrl *pinctrl;
	struct regulator *avdd_vreg;
	struct regulator *iovdd_vreg;

	struct vl53l5_cal_data_t cal_data;
	struct notifier_block dump_nb;  //for sec dump

	struct vl53l5_range_fac_results_t {
#ifdef VL53L5_PER_ZONE_RESULTS_ON
		struct vl53l5_range_per_zone_results_t per_zone_results;
#endif

#ifdef VL53L5_PER_TGT_RESULTS_ON
		struct vl53l5_range_per_tgt_results_t per_tgt_results;
#endif
	} range_data;

#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_UAPI
	struct range_sensor_data_t	af_range_data;
#endif

	const char *genshape_name;
	const char *avdd_vreg_name;
	const char *iovdd_vreg_name;

	uint32_t fac_rotation_mode;
	uint32_t rotation_mode;
	uint32_t force_suspend_count;

	int32_t data[64];
	int32_t max_targets_per_zone;
	int32_t number_of_zones;
	int32_t print_counter;
	int32_t min;
	int32_t avg;
	int32_t max;
	int32_t max_peak_signal;

	int8_t enabled;
	int8_t test_mode;
	int8_t failed_count;
	int8_t update_flag;
	int8_t pass_fail_flag;
	int8_t file_list;

	bool load_calibration;
	bool read_p2p_cal_data;
	bool read_data_valid;
	bool suspend_state;
	bool probe_done;
#endif
};
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
#define CMD_READ_CAL_FILE		0 // 0x1
#define CMD_WRITE_CAL_FILE		1 // 0x2
#define CMD_WRITE_P2P_FILE		2 // 0x4
#define CMD_WRITE_SHAPE_FILE	3 // 0x8
#define CMD_READ_P2P_CAL_FILE		4 // 0x10
#define CMD_READ_SHAPE_CAL_FILE		5 // 0x20
#define CMD_CHECK_CAL_FILE_TYPE		6 // 0x40
#define CMD_DELTE_OPENCAL_FILE		7 // 0x80

#define TIMEOUT_CNT 40
#endif
#endif
