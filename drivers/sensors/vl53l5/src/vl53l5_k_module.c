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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "vl53l5_k_module.h"
#include "vl53l5_k_driver_config.h"
#include "vl53l5_k_gpio_utils.h"
#include "vl53l5_k_logging.h"
#include "vl53l5_k_ioctl_controls.h"
#include "vl53l5_k_error_converter.h"
#include "vl53l5_k_error_codes.h"
#include "vl53l5_k_ioctl_codes.h"
#include "vl53l5_k_version.h"
#include "vl53l5_results_config.h"
#include "vl53l5_long_tail_filter__set_cfg.h"
#include "vl53l5_long_tail_filter_main.h"

#include "vl53l5_output_target_filter_main.h"
#include "vl53l5_pseudo_single_zone__set_cfg.h"
#include "vl53l5_pseudo_single_zone_main.h"
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
#include "vl53l5_platform_init.h"
#endif
#include "vl53l5_platform.h"
#ifdef VL53L5_INTERRUPT
#include "vl53l5_k_interrupt.h"
#endif
#ifdef VL53L5_WORKER
#include "vl53l5_k_work_handler.h"
#endif
#include "vl53l5_error_codes.h"
#ifdef VL53L5_TCDM_ENABLE
#include "vl53l5_tcdm_dump.h"
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
#include "vl53l5_k_range_wait_handler.h"

#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/sensor/sensors_core.h>
#ifdef CONFIG_SENSORS_VL53L5_QCOM
#include <linux/spi/spi-msm-geni.h> //for sec qc spi config
#endif
#include <linux/spi/spidev.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>

#ifdef CONFIG_SENSORS_VL53L5_SLSI
extern int sensors_register(struct device **dev, void *drvdata,
	struct device_attribute *attributes[], char *name);
#endif

#define MODULE_NAME "range_sensor"

#define MIN_SPEC 0
#define MAX_SPEC 100000

#define NUM_OF_ZONE 	64
#define NUM_OF_TARGET	2

#define TEST_300_MM_MAX_ZONE 64
#define TEST_500_MM_MAX_ZONE 16

#ifdef VL53L5_INTERRUPT
#define MAX_READ_COUNT        45
#else
#define MAX_READ_COUNT        10
#endif

#define ENTER 1
#define END   2

#define NUM_OF_PRINT 8

#define FAIL_GET_RANGE	-1
#define EXCEED_AMBIENT	-2
#define EXCEED_XTALK	-3
#define MAX_FAILING_ZONES_LIMIT_FAIL	-452581885
#define NO_VALID_ZONES	-452516349

#define FAC_CAL		1
#define USER_CAL	2

#endif //STM_VL53L5_SUPPORT_SEC_CODE

static struct spi_device *vl53l5_k_spi_device;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
static struct vl53l5_k_module_t *module_table[VL53L5_CFG_MAX_DEV];
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
static struct of_device_id vl53l5_k_match_table[] = {
	{ .compatible = "vl53l5",},
	{},
};
#endif

static const struct spi_device_id vl53l5_k_spi_id[] = {
	{ VL53L5_K_DRIVER_NAME, 0 },
	{ },
};

#ifdef STM_VL53L5_SUPPORT_SEC_CODE
static int vl53l5_suspend(struct device *dev)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	int status;

	vl53l5_k_log_info("fac en %d, state %d",
		p_module->enabled,
		p_module->state_preset);

	p_module->suspend_state = true;
	if (p_module->state_preset == VL53L5_STATE_RANGING) {
		msleep(20);
		vl53l5_k_log_info("force stop");
		status = vl53l5_ioctl_stop(p_module, NULL, 1);
		if (status != STATUS_OK) {
			status = vl53l5_ioctl_init(p_module);
			vl53l5_k_log_error("fail reset %d", status);
			status = vl53l5_ioctl_stop(p_module, NULL, 1);
		}
		if (status == STATUS_OK)
			vl53l5_k_log_info("stop success");
		p_module->force_suspend_count++;
 	}

	return 0;
}

static int vl53l5_resume(struct device *dev)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	p_module->suspend_state = false;
	vl53l5_k_log_info("err %d, force stop count %u", p_module->last_driver_error, p_module->force_suspend_count);

	return 0;
}

static const struct dev_pm_ops vl53l5_pm_ops = {
	.suspend = vl53l5_suspend,
	.resume = vl53l5_resume,
};
#endif

static struct spi_driver vl53l5_k_spi_driver = {
	.driver = {
		.name	= VL53L5_K_DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
		.of_match_table = vl53l5_k_match_table,
		.pm = &vl53l5_pm_ops
#endif
	},
	.probe	= vl53l5_k_spi_probe,
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	.shutdown = vl53l5_k_spi_shutdown,
#endif
	.remove	= vl53l5_k_spi_remove,
	.id_table = vl53l5_k_spi_id,
};

static const struct file_operations ranging_fops = {
	.owner =		THIS_MODULE,
	.unlocked_ioctl =	vl53l5_k_ioctl,
	.open =			vl53l5_k_open,
	.release =		vl53l5_k_release
};

MODULE_DEVICE_TABLE(of, vl53l5_k_match_table);
MODULE_DEVICE_TABLE(spi, vl53l5_k_spi_id);

static DEFINE_MUTEX(dev_table_mutex);

#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_KERNEL_INTERFACE
struct vl53l5_k_module_t *global_p_module = NULL;

enum vl53l5_external_control {
	VL53L5_EXT_START,
	VL53L5_EXT_STOP,
	VL53L5_EXT_GET_RANGE,
};

int vl53l5_ext_control(enum vl53l5_external_control input, void *data, u32 *size)
{
	int status = STATUS_OK;

	if (global_p_module == NULL) {
		if ((input == VL53L5_EXT_START)
			|| (input == VL53L5_EXT_STOP))
			vl53l5_k_log_error("probe failed");
		if (size != NULL)
			*size = 0;
		return -1;
	} else if ((global_p_module->last_driver_error == VL53L5_PROBE_FAILED)
			|| (global_p_module->last_driver_error == VL53L5_SHUTDOWN)
			|| (global_p_module->suspend_state == true)) {
		if ((input == VL53L5_EXT_START)
			|| (input == VL53L5_EXT_STOP))
			vl53l5_k_log_error("failed %d", global_p_module->last_driver_error);
		else
			if (size)
				*size = 0;
		if (global_p_module->suspend_state == true)
			vl53l5_k_log_error("cmd %d is called in suspend", input);
		return -1;
	}

	vl53l5_k_log_debug("Lock");
	mutex_lock(&global_p_module->mutex);

	switch (input) {
	case VL53L5_EXT_START:
		status = vl53l5_ioctl_start(global_p_module, NULL, 1);
#ifdef VL53L5_TEST_ENABLE
		if (status == STATUS_OK) {
			vl53l5_k_log_error("test for reset");
			status = -1;
		}
#endif
		if ((status != STATUS_OK) || (global_p_module->last_driver_error != STATUS_OK)) {
			status = vl53l5_ioctl_init(global_p_module);
			vl53l5_k_log_error("fail reset %d", status);
			status = vl53l5_ioctl_start(global_p_module, NULL, 1);
		}

		if (status != STATUS_OK) {
			vl53l5_k_log_error("start err");
			goto out_state;
		}
		global_p_module->of_cfg.otf_general_cfg.otf__rotation_sel = (uint8_t)global_p_module->rotation_mode;
		vl53l5_k_log_info("start ro %u", global_p_module->of_cfg.otf_general_cfg.otf__rotation_sel);
		break;
	case VL53L5_EXT_STOP:
		status = vl53l5_ioctl_stop(global_p_module, NULL, 1);
#ifdef VL53L5_TEST_ENABLE
		if (status == STATUS_OK) {
			vl53l5_k_log_error("test for reset");
			status = -1;
		}
#endif
		if ((status != STATUS_OK) || (global_p_module->last_driver_error != STATUS_OK)) {
			status = vl53l5_ioctl_init(global_p_module);
			vl53l5_k_log_error("fail reset %d", status);
			status = vl53l5_ioctl_stop(global_p_module, NULL, 1);
		}

		if (status != STATUS_OK) {
			vl53l5_k_log_error("stop err");
			goto out_state;
		}
		vl53l5_k_log_info("stop");
		break;
	case VL53L5_EXT_GET_RANGE:
		if ((global_p_module->state_preset != VL53L5_STATE_RANGING)
			|| (data == NULL)) {
			goto out_state;
		}
		status = vl53l5_ioctl_get_range(global_p_module, NULL);
		if (status != STATUS_OK) {
			*size = 0;
			vl53l5_k_log_debug("get_range err with size %d", *size);
			goto out_state;
		}
#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_UAPI
		memcpy(data, &global_p_module->af_range_data, sizeof(struct range_sensor_data_t));
		*size = sizeof(struct range_sensor_data_t);
#else
		memcpy(data, &global_p_module->range_data, sizeof(global_p_module->range_data));
		*size = sizeof(global_p_module->range_data);
#endif
		break;
	default:
		vl53l5_k_log_error("invalid %d", input);
		break;
	}

	vl53l5_k_log_debug("unLock");
	mutex_unlock(&global_p_module->mutex);

	return status;
out_state:

	vl53l5_k_log_debug("unLock");
	mutex_unlock(&global_p_module->mutex);

	vl53l5_k_log_error("Err status %d", status);
	return status;
}
EXPORT_SYMBOL_GPL(vl53l5_ext_control);
#endif

static void memory_release(struct kref *kref)
{
	struct vl53l5_k_module_t *p_module =
		container_of(kref, struct vl53l5_k_module_t, spi_info.ref);

	LOG_FUNCTION_START("");

	kfree(p_module);

	LOG_FUNCTION_END(0);
}

static int vl53l5_parse_dt(struct device *dev,
			   struct vl53l5_k_module_t *p_module)
{
	struct device_node *np = dev->of_node;
	int status = 0;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_k_log_debug("start");
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	struct device_node *spi_device_node;
	struct platform_device *spi_pdev;

	vl53l5_k_log_info("start");
#endif

	if (of_find_property(np, "spi-cpha", NULL))
		p_module->spi_info.device->mode |= SPI_CPHA;
	if (of_find_property(np, "spi-cpol", NULL))
		p_module->spi_info.device->mode |= SPI_CPOL;

	p_module->comms_type = VL53L5_K_COMMS_TYPE_SPI;

	p_module->gpio.interrupt = -1;

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	p_module->gpio.comms_select = -1;
	p_module->gpio.interrupt = -1;

	p_module->gpio.power_enable_owned = 0;
	p_module->gpio.power_enable =
			of_get_named_gpio(np, "stm,power_enable", 0);
	if (gpio_is_valid(p_module->gpio.power_enable)) {
		status = gpio_request(p_module->gpio.power_enable,
				      "vl53l5_k_pwren");
		if (!status) {
			vl53l5_k_log_debug(
				"Set gpio %d direction",
				p_module->gpio.power_enable);
			p_module->gpio.power_enable_owned = 1;
			status = gpio_direction_output(
					p_module->gpio.power_enable, 0);
			if (status) {
				vl53l5_k_log_error(
				"Set direction failed %d: %d",
				p_module->gpio.power_enable, status);
			}
		} else
			vl53l5_k_log_error("Request gpio failed %d: %d",
					p_module->gpio.power_enable, status);
	} else
		vl53l5_k_log_info(
			"Power enable is not configured.");

	p_module->gpio.low_power_owned = 0;
	p_module->gpio.low_power = of_get_named_gpio(np, "stm,low_power", 0);
	if (gpio_is_valid(p_module->gpio.low_power)) {
		status = gpio_request(
				p_module->gpio.low_power, "vl53l5_k_lp");
		if (!status) {
			vl53l5_k_log_debug(
					"Set gpio %d direction",
					p_module->gpio.low_power);
			p_module->gpio.low_power_owned = 1;
			status = gpio_direction_output(
						p_module->gpio.low_power, 0);
			if (status) {
				vl53l5_k_log_error(
				"Set direction failed %d: %d",
				p_module->gpio.low_power, status);
			}
		} else
			vl53l5_k_log_error("Request gpio failed %d: %d",
			p_module->gpio.low_power, status);
	} else
		vl53l5_k_log_info(
			"The GPIO setting of Low power is not configured.");

	p_module->gpio.comms_select_owned = 0;
	p_module->gpio.comms_select = of_get_named_gpio(
						np, "stm,comms_select", 0);
	if (gpio_is_valid(p_module->gpio.comms_select)) {
		status = gpio_request(
				p_module->gpio.comms_select, "vl53l5_k_cs");
		if (!status) {
			vl53l5_k_log_debug(
					"Set gpio %d direction",
					p_module->gpio.comms_select);
			p_module->gpio.comms_select_owned = 1;
			status = gpio_direction_output(
						p_module->gpio.comms_select, 0);
			if (status) {
				vl53l5_k_log_error(
				"Set direction failed gpio %d: %d",
				p_module->gpio.comms_select, status);
			}
		} else
			vl53l5_k_log_error("Request gpio failed %d: %d",
			p_module->gpio.comms_select, status);
	} else
		vl53l5_k_log_info(
			"The GPIO setting of comms select is not configured.");
#endif
#ifdef VL53L5_INTERRUPT

	p_module->gpio.interrupt_owned = 0;
	p_module->gpio.interrupt = of_get_named_gpio(
						np, "stm,interrupt", 0);
	if (gpio_is_valid(p_module->gpio.interrupt)) {
		status = gpio_request(
				p_module->gpio.interrupt, "vl53l5_k_interrupt");
		if (!status) {
			vl53l5_k_log_debug(
					"Set gpio %d direction",
					p_module->gpio.interrupt);
			p_module->gpio.interrupt_owned = 1;
			status = gpio_direction_input(
						p_module->gpio.interrupt);
			if (status) {
				p_module->gpio.interrupt_owned = 0;
				vl53l5_k_log_error(
				"Set input direction failed gpio %d: %d",
				p_module->gpio.interrupt, status);
			}
		} else
			vl53l5_k_log_error("Request gpio failed %d: %d",
			p_module->gpio.interrupt, status);
	} else
		vl53l5_k_log_info(
			"The GPIO setting of comms select is not configured.");
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	if (of_property_read_string_index(np, "stm,io_vdd", 0,
			(const char **)&p_module->iovdd_vreg_name)) {
		p_module->iovdd_vreg_name = NULL;
	}
	vl53l5_k_log_info("%s: io_vdd %s\n", __func__, p_module->iovdd_vreg_name);

	if (of_property_read_string_index(np, "stm,a_vdd", 0,
			(const char **)&p_module->avdd_vreg_name)) {
		p_module->avdd_vreg_name = NULL;
	}
	vl53l5_k_log_info("%s: a_vdd %s\n", __func__, p_module->avdd_vreg_name);
#endif
	of_property_read_string(np, "stm,firmware_name",
				&p_module->firmware_name);
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	if (!p_module->firmware_name) {
		vl53l5_k_log_error(
			"%s: firmware name does not declair in dts", __func__);
		p_module->firmware_name = "range_sensor/vl53l5.bin";
	}

	of_property_read_string(np, "stm,genshape_name",
				&p_module->genshape_name);
	if (!p_module->genshape_name) {
		vl53l5_k_log_error(
			"%s: generic shape name does not declair in dts", __func__);
		p_module->genshape_name = "range_sensor/generic_xtalk_shape.bin";
	}

	if (of_property_read_u32(np, "stm,fac_rotation_mode", &p_module->fac_rotation_mode))
		p_module->fac_rotation_mode = 0;
	else
		vl53l5_k_log_info("fac_rotation_mode %d", p_module->fac_rotation_mode);

	if (of_property_read_u32(np, "stm,rotation_mode", &p_module->rotation_mode))
		p_module->rotation_mode = 0;
	else
		vl53l5_k_log_info("rotation_mode %d", p_module->rotation_mode);

	//for pinctrl on vddoff
	p_module->pinctrl = NULL;
	p_module->pinctrl_vddoff = NULL;
	p_module->pinctrl_default = NULL;
	spi_device_node = of_parse_phandle(np, "stm,spi_node", 0);
	if (!IS_ERR_OR_NULL(spi_device_node)) {
		spi_pdev = of_find_device_by_node(spi_device_node);
		if (!spi_pdev) {
			vl53l5_k_log_error("of_find_device_by_node failed\n");
		}
		else {
			p_module->pinctrl = devm_pinctrl_get(&spi_pdev->dev);
			if (IS_ERR(p_module->pinctrl)) {
				vl53l5_k_log_error("devm_pinctrl_get failed\n");
				p_module->pinctrl = NULL;
			} else {
				p_module->pinctrl_vddoff = pinctrl_lookup_state(p_module->pinctrl, "vddoff");
				if (IS_ERR(p_module->pinctrl_vddoff)) {
					p_module->pinctrl_vddoff = NULL;
					vl53l5_k_log_error("vddoff pinctrl failed\n");
				}
				p_module->pinctrl_default = pinctrl_lookup_state(p_module->pinctrl, "default");
				if (IS_ERR(p_module->pinctrl_default)) {
					p_module->pinctrl_default = NULL;
					vl53l5_k_log_error("default pinctrl failed\n");
				}
			}
		}
	}
#endif
	return status;
}

static void vl53l5_k_clean_up_spi(void)
{
	LOG_FUNCTION_START("");
	if (vl53l5_k_spi_device != NULL) {
		vl53l5_k_log_debug("Unregistering spi device");
		spi_unregister_device(vl53l5_k_spi_device);
	}
	LOG_FUNCTION_END(0);
}

#ifdef STM_VL53L5_SUPPORT_SEC_CODE
void vl53l5_k_re_init(struct vl53l5_k_module_t *p_module)
{
	if (p_module->last_driver_error == VL53L5_PROBE_FAILED) {
		int status;

		vl53l5_k_log_error("last err %d", p_module->last_driver_error);
		if (p_module->pinctrl && p_module->pinctrl_default) {
			vl53l5_k_log_info("pinctrl_select_state for vddoff\n");
			status = pinctrl_select_state(p_module->pinctrl, p_module->pinctrl_default);
			if (status < 0)
				vl53l5_k_log_error("pinctrl_select_state failed: %d\n", status);
		}
		p_module->last_driver_error = STATUS_OK;
		status = vl53l5_ioctl_init(p_module);

		if (status == STATUS_OK) {
			vl53l5_k_log_info("get module_info");
			status = vl53l5_ioctl_get_module_info(p_module, NULL);
		}

		if (status != STATUS_OK) {
			vl53l5_k_log_error("fail reset %d", status);
			p_module->enabled = -1;
			p_module->last_driver_error = VL53L5_PROBE_FAILED;
			vl53l5_k_power_onoff(p_module, p_module->iovdd_vreg, p_module->iovdd_vreg_name, 0);
			vl53l5_k_power_onoff(p_module, p_module->avdd_vreg, p_module->avdd_vreg_name, 0);

			if (p_module->pinctrl && p_module->pinctrl_vddoff) {
				vl53l5_k_log_info("pinctrl_select_state for vddoff\n");
				status = pinctrl_select_state(p_module->pinctrl, p_module->pinctrl_vddoff);
				if (status < 0)
					vl53l5_k_log_error("pinctrl_select_state failed: %d\n", status);
			}
		} else {
#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_KERNEL_INTERFACE
			global_p_module = p_module;
#endif
			p_module->enabled = 0;
			vl53l5_ioctl_stop(p_module, NULL, 1);
		}
	}
}

int vl53l5_check_calibration_condition(struct vl53l5_k_module_t *p_module, int seq)
{
	int status = STATUS_OK, prev_state = VL53L5_STATE_RANGING;
	int count = 0;
	int ret = STATUS_OK;
	int max_ambient = 0, max_xtalk = 0, max_peaksignal = 0;
	int i = 0, j = 0, idx = 0;
#ifndef VL53L5_INTERRUPT
	int data_ready = 0;
#endif

	vl53l5_k_log_info("state_preset %d, power_state %d", p_module->state_preset, p_module->power_state);
	if (p_module->state_preset != VL53L5_STATE_RANGING) {
		prev_state = p_module->state_preset;

		status = vl53l5_ioctl_start(p_module, NULL, 1);
		if (status != STATUS_OK) {
			status = vl53l5_ioctl_init(p_module);
			vl53l5_k_log_error("restart %d", status);
			status = vl53l5_ioctl_start(p_module, NULL, 1);
		}

		if (status == STATUS_OK) {
			p_module->enabled = 1;
		} else {
			vl53l5_k_log_error("start err");
			vl53l5_k_store_error(p_module, status);
			ret = FAIL_GET_RANGE;
			goto out_result;
		}
	}
	msleep(180);

	vl53l5_k_log_info("get_range start %d", p_module->fac_rotation_mode);

	memset(&p_module->range_data, 0, sizeof(p_module->range_data));

#ifdef VL53L5_INTERRUPT
	while ((p_module->range.count == 0) && (count < MAX_READ_COUNT)) {
		usleep_range(10000, 10100);
		++count;
	}
	status = vl53l5_ioctl_get_range(p_module, NULL);
#else
	status = vl53l5_k_check_data_ready(p_module, &data_ready);
	while (!data_ready && (count < MAX_READ_COUNT)) {
		vl53l5_k_log_info("check ready %d %d", data_ready, status);
		usleep_range(10000, 10100);
		status = vl53l5_k_check_data_ready(p_module, &data_ready);
		count++;
	}
	status = vl53l5_ioctl_get_range(p_module, NULL);
#endif
	if (status == STATUS_OK) {
		for (i = 0; i < NUM_OF_PRINT; i++) {
			for (j = 0; j < NUM_OF_PRINT; j++) {
				idx = (i * NUM_OF_PRINT + j);
				if (seq == END && max_xtalk < (p_module->calibration.cal_data.pxtalk_grid_rate.cal__grid_data__rate_kcps_per_spad[idx] >> 11))
					max_xtalk = p_module->calibration.cal_data.pxtalk_grid_rate.cal__grid_data__rate_kcps_per_spad[idx] >> 11;
				if (max_peaksignal < (p_module->range_data.per_tgt_results.peak_rate_kcps_per_spad[idx * p_module->max_targets_per_zone] >> 11))
					max_peaksignal = p_module->range_data.per_tgt_results.peak_rate_kcps_per_spad[idx * p_module->max_targets_per_zone] >> 11;
				if (max_ambient < (p_module->range_data.per_zone_results.amb_rate_kcps_per_spad[idx] >> 11))
					max_ambient = p_module->range_data.per_zone_results.amb_rate_kcps_per_spad[idx] >> 11;
			}
		}
	} else {
		vl53l5_k_log_info("get range fail, status : %d", status);
		ret = FAIL_GET_RANGE;
		goto out_result;
	}

	p_module->max_peak_signal = max_peaksignal;

	if (max_ambient > 35)
		ret = EXCEED_AMBIENT;
	else if (seq == END && max_xtalk > 200)
		ret = EXCEED_XTALK;

	vl53l5_k_log_info("max_ambient : %d, max_xtalk : %d, ret : %d, max_peaksignal : %d \n", max_ambient, max_xtalk, ret, max_peaksignal);

out_result:
	if (prev_state != VL53L5_STATE_RANGING) {
		p_module->enabled = 0;
		vl53l5_ioctl_stop(p_module, NULL, 1);
	}

	return ret;
}

int vl53l5_perform_open_calibration(struct vl53l5_k_module_t *p_module)
{
	int ret = STATUS_OK;

	vl53l5_k_log_info("Do CHAR CAL\n");
	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_HP_IDLE);
	msleep(20);

	ret = vl53l5_perform_calibration(p_module, 3);

	msleep(20);
	vl53l5_ioctl_set_device_parameters(p_module, NULL, VL53L5_CFG__BACK_TO_BACK__8X8__15HZ);
	usleep_range(2000, 2100);
	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_LP_IDLE_COMMS);

	return ret;
}

int vl53l5_load_open_calibration(struct vl53l5_k_module_t *p_module)
{
	int p2p, cha;

	vl53l5_k_log_info("Open CAL Load\n");

	p_module->stdev.status_cal = 0;
	p_module->load_calibration = false;
	p_module->read_p2p_cal_data = false;

	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_HP_IDLE);
	usleep_range(2000, 2100);

	cha = vl53l5_ioctl_read_open_cal_shape_calibration(p_module);
	msleep(100);

	p2p = vl53l5_ioctl_read_open_cal_p2p_calibration(p_module);
	usleep_range(2000, 2100);

	vl53l5_k_log_info("cha %d, p2p %d\n", cha, p2p);

	if (cha == STATUS_OK)
		p_module->load_calibration = true;
	if (p2p == STATUS_OK)
		p_module->read_p2p_cal_data = true;

	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_LP_IDLE_COMMS);
	msleep(20);

	if (cha != STATUS_OK)
		return cha;
	else if (p2p != STATUS_OK)
		return p2p;
	else
		return STATUS_OK;
}

void vl53l5_load_factory_calibration(struct vl53l5_k_module_t *p_module)
{
	int cha, p2p;

	vl53l5_k_log_info("Load CAL\n");

	p_module->stdev.status_cal = 0;
	p_module->load_calibration = false;
	p_module->read_p2p_cal_data = false;

	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_HP_IDLE);
	usleep_range(2000, 2100);

	cha = vl53l5_ioctl_read_generic_shape(p_module);
	usleep_range(2000, 2100);

	p2p = vl53l5_ioctl_read_p2p_calibration(p_module, true);
	usleep_range(2000, 2100);

	vl53l5_k_log_info("cha %d, p2p %d\n", cha, p2p);

	if (cha == STATUS_OK)
		p_module->load_calibration = true;
	if (p2p == STATUS_OK)
		p_module->read_p2p_cal_data = true;

	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_LP_IDLE_COMMS);
}

static ssize_t vl53l5_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//test code
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	vl53l5_ioctl_init(p_module);
	///end

	return snprintf(buf, PAGE_SIZE, "VL53L5\n");
}

static ssize_t vl53l5_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "STM\n");
}

static ssize_t vl53l5_firmware_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	struct vl53l5_version_t p_version;
	int status = STATUS_OK;
	enum vl53l5_k_state_preset prev_state;

	p_version.firmware.ver_major = 0;
	p_version.firmware.ver_minor = 0;
	p_version.firmware.ver_build = 0;
	p_version.firmware.ver_revision = 0;

	prev_state = p_module->state_preset;

	if (p_module->state_preset <= VL53L5_STATE_LOW_POWER) {
		vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_HP_IDLE);
		usleep_range(2000, 2100);
	}

	status = vl53l5_get_version(&p_module->stdev, &p_version);

	if (prev_state <= VL53L5_STATE_LOW_POWER) {
		vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_LP_IDLE_COMMS);
	}

	vl53l5_k_log_info(
		"FW Version %d.%d.%d.%d\n",
		p_version.firmware.ver_major,
		p_version.firmware.ver_minor,
		p_version.firmware.ver_build,
		p_version.firmware.ver_revision);

	return snprintf(buf, PAGE_SIZE, "%d.%d.%d.%d\n",
		p_version.firmware.ver_major,
		p_version.firmware.ver_minor,
		p_version.firmware.ver_build,
		p_version.firmware.ver_revision);
}

#define DISTANCE_300MM_MIN_SPEC 255
#define DISTANCE_300MM_MAX_SPEC 345
#define MAX_READ_COUNT        10

static ssize_t vl53l5_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int min = 0, max = 0,  avg = 0;
	int data_ready = 0, count = 0;
	int status = STATUS_OK;
	p_module->read_data_valid = false;

	if ((p_module->state_preset != VL53L5_STATE_RANGING)
		|| (p_module->last_driver_error == VL53L5_PROBE_FAILED)) {
		vl53l5_k_log_error("state %d, err %d", p_module->state_preset, p_module->last_driver_error);
#ifdef VL53L5_TEST_ENABLE
		msleep(50);
		panic("probe failed %d", p_module->last_driver_error);
#endif
		goto out;
	}

	vl53l5_k_log_debug("Lock");
	mutex_lock(&p_module->mutex);

	vl53l5_k_log_info("get_range start %d", p_module->fac_rotation_mode);

#ifdef VL53L5_INTERRUPT
	p_module->range_mode = VL53L5_RANGE_SERVICE_DEFAULT;
	status = vl53l5_ioctl_get_range(p_module, NULL);
	p_module->range_mode = VL53L5_RANGE_SERVICE_INTERRUPT;
#else
	status = vl53l5_k_check_data_ready(p_module, &data_ready);
	while (!data_ready && (count < MAX_READ_COUNT)) {
		vl53l5_k_log_info("check ready %d %d", data_ready, status);
		usleep_range(10000, 10100);
		status = vl53l5_k_check_data_ready(p_module, &data_ready);
		count++;
	}

	status = vl53l5_ioctl_get_range(p_module, NULL);
#endif

	vl53l5_k_log_debug("unLock");
	mutex_unlock(&p_module->mutex);

	if (status != STATUS_OK) {
		vl53l5_k_log_info("get_range failed %d", status);
#ifdef VL53L5_TCDM_ENABLE
		_tcdm_dump(p_module);
#endif
#ifdef VL53L5_TEST_ENABLE
		msleep(50);
		panic("get_range failed %d", p_module->last_driver_error);
#endif
	} else {
		vl53l5_k_log_info("get_range done\n");
		p_module->read_data_valid = true;
		if (status != STATUS_OK)
			vl53l5_k_log_error("rotate fail %d",status);
	}
out:
	if (p_module->read_data_valid) {
		for (i = 0; i < p_module->print_counter; i++) {
			for (j = 0; j < p_module->print_counter; j++) {
				idx = (i * p_module->print_counter + j) * p_module->max_targets_per_zone;
				p_module->data[idx / p_module->max_targets_per_zone] =
					(p_module->range_data.per_tgt_results.target_status[idx] == 5 ||
					p_module->range_data.per_tgt_results.target_status[idx] == 9) ?
					p_module->range_data.per_tgt_results.median_range_mm[idx] >> 2 : 0;

			}
		}
	} else {
		for (i = 0 ; i < p_module->number_of_zones; i++) {
			p_module->data[i] = -1;
		}
	}

	min = max = avg = p_module->data[1];

	for (i = 2; i < p_module->number_of_zones; i++) {
		if ((i != 7) && (i != 56) && (i != 63)) {
			if (p_module->data[i] < min)
				min = p_module->data[i];
			if (max < p_module->data[i])
				max = p_module->data[i];
			avg += p_module->data[i];
		}
	}

	for (i = 0; i < 64; i+=8) {
		vl53l5_k_log_info("%d, %d, %d, %d, %d, %d, %d, %d\n",
		p_module->data[i], p_module->data[i+1], p_module->data[i+2],
		p_module->data[i+3], p_module->data[i+4], p_module->data[i+5],
		p_module->data[i+6], p_module->data[i+7]);
	}

	avg = avg / (p_module->number_of_zones - 4);

	i = j = 0;

	j += snprintf(buf + j, PAGE_SIZE - j, "Distance,%d,%d,%d,%d,%d,",
		DISTANCE_300MM_MIN_SPEC, DISTANCE_300MM_MAX_SPEC, min, max, avg);

	for (; i < TEST_300_MM_MAX_ZONE; i++) {
		if ((i != 0) &&(i != 7) && (i != 56) && (i != 63))
			j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "N");

		if (i != p_module->number_of_zones - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}

#define MIN_TEMP_SPEC 10
#define MAX_TEMP_SPEC 50

static ssize_t vl53l5_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	int status = STATUS_OK, prev_state = VL53L5_STATE_RANGING;
	int data_ready = 0, read_count = 10;
	int8_t temp = -100;

	vl53l5_k_log_debug("Lock");
	mutex_lock(&p_module->mutex);

	vl53l5_k_log_info("state_preset %d, power_state %d", p_module->state_preset, p_module->power_state);
	if (p_module->state_preset != VL53L5_STATE_RANGING) {
		prev_state = p_module->state_preset;
		vl53l5_ioctl_start(p_module, NULL, 1);
	}

	msleep(180);

#ifdef VL53L5_INTERRUPT
	p_module->range_mode = VL53L5_RANGE_SERVICE_DEFAULT;
#endif

	while ((read_count > 0) && (!data_ready)) {
		msleep(20);
		status = vl53l5_k_check_data_ready(p_module, &data_ready);
		if ((data_ready) || (status != STATUS_OK))
			break;
		read_count--;
	}

	status = vl53l5_ioctl_get_range(p_module, NULL);
	if (status == STATUS_OK) {
		temp = p_module->range.data.core.meta_data.silicon_temp_degc;
		vl53l5_k_log_info("temp %d", temp);
	} else {
		vl53l5_k_log_error("read temp failed");
	}

#ifdef VL53L5_INTERRUPT
	p_module->range_mode = VL53L5_RANGE_SERVICE_INTERRUPT;
#endif

	if (prev_state != VL53L5_STATE_RANGING) {
		vl53l5_ioctl_stop(p_module, NULL, 1);
	}

	vl53l5_k_log_debug("unLock");
	mutex_unlock(&p_module->mutex);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", MIN_TEMP_SPEC, MAX_TEMP_SPEC, temp);
}

static ssize_t vl53l5_ambient_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int min = 0, max = 0,  avg = 0;

	for (i = 0; i < p_module->print_counter; i++) {
		for (j = 0; j < p_module->print_counter; j++) {
			idx = i * p_module->print_counter + j;
			p_module->data[idx] =
				p_module->range_data.per_zone_results.amb_rate_kcps_per_spad[idx] >> 11;
		}
	}

	min = max = avg = p_module->data[0];
	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}
	avg = avg / p_module->number_of_zones;

	for (i = 0; i < 64; i+=8) {
		vl53l5_k_log_info("%d, %d, %d, %d, %d, %d, %d, %d\n",
		p_module->data[i], p_module->data[i+1], p_module->data[i+2],
		p_module->data[i+3], p_module->data[i+4], p_module->data[i+5],
		p_module->data[i+6], p_module->data[i+7]);
	}

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "Ambient,%d,%d,%d,%d,%d,",
		MIN_SPEC, MAX_SPEC, min, max, avg);
	for (; i < p_module->number_of_zones; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != p_module->number_of_zones - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}

#define MIN_MAX_RANGE_SPEC 500
#define MAX_MAX_RANGE_SPEC 10500

static ssize_t vl53l5_max_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int32_t min = 0, max = 0, avg = 0;

	if (p_module->read_data_valid) {
		for (i = 0; i < p_module->print_counter; i++) {
			for (j = 0; j < p_module->print_counter; j++) {
				idx = i * p_module->print_counter + j;
				p_module->data[idx] =
						p_module->range_data.per_zone_results.amb_dmax_mm[idx];
			}
		}
	} else {
		for (i = 0 ; i < p_module->number_of_zones; i++)
			p_module->data[i] = -1;
	}

	min = max = avg = p_module->data[0];

	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}

	avg = avg / p_module->number_of_zones;

	for (i = 0; i < 64; i+=8) {
		vl53l5_k_log_info("%d, %d, %d, %d, %d, %d, %d, %d\n",
		p_module->data[i], p_module->data[i+1], p_module->data[i+2],
		p_module->data[i+3], p_module->data[i+4], p_module->data[i+5],
		p_module->data[i+6], p_module->data[i+7]);
	}

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "Maxrange,%d,%d,%d,%d,%d,",
		MIN_MAX_RANGE_SPEC, MAX_MAX_RANGE_SPEC, min, max, avg);

	for (; i < TEST_300_MM_MAX_ZONE; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != TEST_300_MM_MAX_ZONE - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}


	return j;
}

#define MIN_PEAK_SIGNAL_SPEC 0
#define MAX_PEAK_SIGNAL_SPEC 1000

static ssize_t vl53l5_peak_signal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int32_t min = 0, max = 0, avg = 0;

	if (p_module->read_data_valid) {
		for (i = 0; i < p_module->print_counter; i++) {
			for (j = 0; j < p_module->print_counter; j++) {
				idx = (i * p_module->print_counter + j) * p_module->max_targets_per_zone;
				p_module->data[idx / p_module->max_targets_per_zone] =
					p_module->range_data.per_tgt_results.peak_rate_kcps_per_spad[idx] >> 11;
			}
		}
	} else {
		for (i = 0 ; i < p_module->number_of_zones; i++) {
			p_module->data[i] = -1;
		}
	}

	min = max = avg = p_module->data[0];

	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}

	avg = avg / p_module->number_of_zones;

	for (i = 0; i < NUM_OF_ZONE; i+=8) {
		vl53l5_k_log_info("%d, %d, %d, %d, %d, %d, %d, %d\n",
		p_module->data[i], p_module->data[i+1], p_module->data[i+2],
		p_module->data[i+3], p_module->data[i+4], p_module->data[i+5],
		p_module->data[i+6], p_module->data[i+7]);
	}

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "Peaksignal,%d,%d,%d,%d,%d,",
		MIN_PEAK_SIGNAL_SPEC, MAX_PEAK_SIGNAL_SPEC, min, max, avg);

	for (; i < TEST_300_MM_MAX_ZONE; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != TEST_300_MM_MAX_ZONE - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}


static ssize_t vl53l5_range_sigma_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int min = 0, max = 0,  avg = 0;

	for (i = 0; i < p_module->print_counter; i++) {
		for (j = 0; j < p_module->print_counter; j++) {
			idx = (i * p_module->print_counter + j) * p_module->max_targets_per_zone;
			p_module->data[idx / p_module->max_targets_per_zone] =
					p_module->range_data.per_tgt_results.range_sigma_mm[idx] >> 7;
		}
	}

	min = max = avg = p_module->data[0];
	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}
	avg = avg / p_module->number_of_zones;

	for (i = 0; i < NUM_OF_ZONE; i+=8) {
		vl53l5_k_log_info("%d, %d, %d, %d, %d, %d, %d, %d\n",
		p_module->data[i], p_module->data[i+1], p_module->data[i+2],
		p_module->data[i+3], p_module->data[i+4], p_module->data[i+5],
		p_module->data[i+6], p_module->data[i+7]);
	}

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "Rangesigma,%d,%d,%d,%d,%d,",
		MIN_SPEC, MAX_SPEC, min, max, avg);
	for (; i < p_module->number_of_zones; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != p_module->number_of_zones - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}

static ssize_t vl53l5_target_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int min = 0, max = 0,  avg = 0;

	for (i = 0; i < p_module->print_counter; i++) {
		for (j = 0; j < p_module->print_counter; j++) {
			idx = (i * p_module->print_counter + j) * p_module->max_targets_per_zone;
			p_module->data[idx / p_module->max_targets_per_zone] =
					p_module->range_data.per_tgt_results.target_status[idx];
		}
	}

	min = max = avg = p_module->data[0];
	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}

	avg = avg / p_module->number_of_zones;

	for (i = 0; i < NUM_OF_ZONE; i+=8) {
		vl53l5_k_log_info("%d, %d, %d, %d, %d, %d, %d, %d\n",
		p_module->data[i], p_module->data[i+1], p_module->data[i+2],
		p_module->data[i+3], p_module->data[i+4], p_module->data[i+5],
		p_module->data[i+6], p_module->data[i+7]);
	}

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "Targetstatus,%d,%d,%d,%d,%d,",
		MIN_SPEC, MAX_SPEC, min, max, avg);
	for (; i < p_module->number_of_zones; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != p_module->number_of_zones - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}

static ssize_t vl53l5_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", p_module->enabled);
}

static ssize_t vl53l5_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);

	u8 val;
	int status;

	vl53l5_k_log_debug("Lock");
	mutex_lock(&p_module->mutex);

	vl53l5_k_re_init(p_module);

	if (p_module->last_driver_error == VL53L5_PROBE_FAILED) {
		vl53l5_k_log_error("failed %d", p_module->last_driver_error);
#ifdef VL53L5_TEST_ENABLE
		msleep(50);
		panic("enable failed %d", p_module->last_driver_error);
#endif

		vl53l5_k_log_debug("unLock");
		mutex_unlock(&p_module->mutex);

		return count;
	}

	status = kstrtou8(buf, 10, &val);

	switch(val) {
	case 1:
		vl53l5_k_log_info("enable start\n");
		if (p_module->enabled < 0)
			vl53l5_k_log_info("probe failed\n");
		else {
			status = vl53l5_ioctl_start(p_module, NULL, 1);
/*#ifdef VL53L5_TEST_ENABLE
			if (status == STATUS_OK) {
				vl53l5_k_log_error("test for reset");
				status = -1;
			}
#endif*/
			if (status != STATUS_OK) {
				status = vl53l5_ioctl_init(p_module);
				vl53l5_k_log_info("reset %d", status);
				status = vl53l5_ioctl_start(p_module, NULL, 1);
			}

			if (status == STATUS_OK) {
				p_module->enabled = 1;
				p_module->of_cfg.otf_general_cfg.otf__rotation_sel = (uint8_t)p_module->fac_rotation_mode;
			} else {
				vl53l5_k_log_error("start err");
				vl53l5_k_store_error(p_module, status);
#ifdef VL53L5_TCDM_ENABLE
				_tcdm_dump(p_module);
#endif
#ifdef VL53L5_TEST_ENABLE
				msleep(50);
				panic("enable failed %d", p_module->last_driver_error);
#endif
			}
		}
		vl53l5_k_log_info("enable done\n");
		break;
	default:
		vl53l5_k_log_info("disable start\n");
		if (p_module->enabled < 0)
			vl53l5_k_log_info("probe failed\n");
		else {
			status = vl53l5_ioctl_stop(p_module, NULL, 1);
/*#ifdef VL53L5_TEST_ENABLE
			if (status == STATUS_OK) {
				vl53l5_k_log_error("test for reset");
				status = -1;
			}
#endif*/
			if ((status != STATUS_OK) || (p_module->last_driver_error != STATUS_OK)) {
				status = vl53l5_ioctl_init(p_module);
				vl53l5_k_log_info("reset %d", status);
				status = vl53l5_ioctl_stop(p_module, NULL, 1);
			}
			if (status == STATUS_OK) {
				p_module->enabled = 0;
				p_module->of_cfg.otf_general_cfg.otf__rotation_sel = (uint8_t)p_module->rotation_mode;
			} else {
				vl53l5_k_store_error(p_module, status);
#ifdef VL53L5_TCDM_ENABLE
				_tcdm_dump(p_module);
#endif
#ifdef VL53L5_TEST_ENABLE
				msleep(50);
				panic("disable failed %d", p_module->last_driver_error);
#endif
			}
		}
		vl53l5_k_log_info("disable done\n");
		break;
	}

	vl53l5_k_log_debug("unLock");
	mutex_unlock(&p_module->mutex);

	vl53l5_k_log_info("rotation %u", p_module->of_cfg.otf_general_cfg.otf__rotation_sel);
	return count;
}

static ssize_t vl53l5_test_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	if (p_module->test_mode)
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", p_module->test_mode, TEST_500_MM_MAX_ZONE);
	else
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", p_module->test_mode, TEST_300_MM_MAX_ZONE);
}

static ssize_t vl53l5_test_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);

	u8 val;
	int ret;

	ret = kstrtou8(buf, 10, &val);

	switch(val) {
	case 1:
		vl53l5_k_log_info("Set 500 mm test mode\n");
		p_module->test_mode = 1;
		break;
	default:
		vl53l5_k_log_info("Set 300 mm test mode\n");
		p_module->test_mode = 0;
		break;
	}

	return count;
}

static ssize_t vl53l5_uid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);

	vl53l5_k_log_info("%x,%x\n", p_module->m_info.module_id_hi, p_module->m_info.module_id_lo);
	return snprintf(buf, PAGE_SIZE, "%x%x\n", p_module->m_info.module_id_hi, p_module->m_info.module_id_lo);
}

static ssize_t vl53l5_cal_uid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);

	if (p_module->read_p2p_cal_data == true)
		vl53l5_k_log_info("%x,%x\n",
				p_module->calibration.info.module_id_hi,
				p_module->calibration.info.module_id_lo);

	return snprintf(buf, PAGE_SIZE, "%x%x\n", p_module->calibration.info.module_id_hi, p_module->calibration.info.module_id_lo);
}

static ssize_t vl53l5_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "continuos\n");
}

static ssize_t vl53l5_frame_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "15\n");
}

#define CAL_XTALK_MIN_SPEC 0
#define CAL_XTALK_MAX_SPEC 200 //120

static ssize_t vl53l5_cal_xtalk_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t idx = 0;
	int32_t min = 0;
	int32_t max = 0;
	int32_t avg = 0;

	for (i = 0; i < p_module->print_counter; i++) {
		for (j = 0; j < p_module->print_counter; j++) {
			idx = (i * p_module->print_counter + j);
			if (p_module->read_p2p_cal_data == true)
				p_module->data[idx] = p_module->calibration.cal_data.pxtalk_grid_rate.cal__grid_data__rate_kcps_per_spad[idx] >> 11;
			else
				p_module->data[idx] = -1;
		}
	}

	min = max = avg = p_module->data[0];
	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}

	if (p_module->read_p2p_cal_data == true)
		avg = avg / p_module->number_of_zones;
	else
		avg = 0;

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "crosstalk,%d,%d,%d,%d,%d,",
		CAL_XTALK_MIN_SPEC, CAL_XTALK_MAX_SPEC, min, max, avg);
	for (; i < p_module->number_of_zones; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != p_module->number_of_zones - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}

#define MIN_CAL_OFFSET_SPEC -500
#define MAX_CAL_OFFSET_SPEC 500

static ssize_t vl53l5_cal_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	uint32_t i = 0, j = 0, idx = 0;
	int min = 0, max = 0,  avg = 0;

	for (i = 0; i < p_module->print_counter; i++) {
		for (j = 0; j < p_module->print_counter; j++) {
			idx = (i * p_module->print_counter + j);
			if (p_module->read_p2p_cal_data == true)
				p_module->data[idx] = p_module->calibration.cal_data.poffset_grid_offset.cal__grid_data__range_mm[idx] >> 2;
			else
				p_module->data[idx] = -999999;
		}
	}

	min = max = avg = p_module->data[0];
	for (i = 1; i < p_module->number_of_zones; i++) {
		if (p_module->data[i] < min)
			min = p_module->data[i];
		if (max < p_module->data[i])
			max = p_module->data[i];
		avg += p_module->data[i];
	}

	if (p_module->read_p2p_cal_data == true)
		avg = avg / p_module->number_of_zones;
	else
		avg = 0;

	i = j = 0;
	j += snprintf(buf + j, PAGE_SIZE - j, "offset,%d,%d,%d,%d,%d,",
		MIN_CAL_OFFSET_SPEC, MAX_CAL_OFFSET_SPEC, min, max, avg);
	for (; i < p_module->number_of_zones; i++) {
		j += snprintf(buf + j, PAGE_SIZE - j, "%d", p_module->data[i]);
		if (i != p_module->number_of_zones - 1)
			j += snprintf(buf + j, PAGE_SIZE - j, ",");
		else
			j += snprintf(buf + j, PAGE_SIZE - j, "\n");
	}

	return j;
}

static ssize_t vl53l5_open_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	int ret = 0;
	int status = 0;
	int cal_status;

	cal_status = vl53l5_check_calibration_condition(p_module, ENTER);
	if (cal_status < 0) {
		vl53l5_k_log_info("fail enter open CAL!\n");
		if (cal_status == FAIL_GET_RANGE)
			return snprintf(buf, PAGE_SIZE, "FAIL_MODULE_CONNECTOR\n");
		else
			return snprintf(buf, PAGE_SIZE, "FAIL_AMBIENT\n");
	}

	ret = vl53l5_perform_open_calibration(p_module);
	if (ret == STATUS_OK)
		ret = vl53l5_load_open_calibration(p_module);

	cal_status = vl53l5_check_calibration_condition(p_module, END);
	if (cal_status < 0 || ret < 0) {
		vl53l5_k_log_info("cal fail!\n");
		status = vl53l5_input_report(p_module, 7, CMD_DELTE_OPENCAL_FILE);
		if (status < 0)
			vl53l5_k_log_error("could not delete wrong cal file!\n");

		vl53l5_k_log_error("Restore to factory cal file!\n");
		vl53l5_load_factory_calibration(p_module);

		if (cal_status == EXCEED_AMBIENT)
			return snprintf(buf, PAGE_SIZE, "FAIL_AMBIENT\n");
		else if ((ret == MAX_FAILING_ZONES_LIMIT_FAIL || ret == NO_VALID_ZONES) && p_module->max_peak_signal > 200)
			return snprintf(buf, PAGE_SIZE, "FAIL_CLOSED\n");
		else if ((ret == MAX_FAILING_ZONES_LIMIT_FAIL || ret == NO_VALID_ZONES) && p_module->max_peak_signal <= 200)
			return snprintf(buf, PAGE_SIZE, "FAIL_BG\n");
		else if (cal_status == EXCEED_XTALK)
			return snprintf(buf, PAGE_SIZE, "FAIL_CLOSED_DUST\n");
		else
			return snprintf(buf, PAGE_SIZE, "FAIL_MODULE_CONNECTOR\n");
	}

	vl53l5_k_log_info("cal done!\n");
	return snprintf(buf, PAGE_SIZE, "PASS\n");
}

static ssize_t vl53l5_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);

	if (p_module->read_p2p_cal_data == true)
		return snprintf(buf, PAGE_SIZE, "O\n");
	else
		return snprintf(buf, PAGE_SIZE, "X\n");
}

int vl53l5_reset(struct vl53l5_k_module_t *p_module)
{
	int status = STATUS_OK;
	status = vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_ULP_IDLE);
	if (status != STATUS_OK) {
		vl53l5_k_log_error("Set ulp err");
		goto out;
	}
	usleep_range(2000, 2100);
	status = vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_HP_IDLE);
	if (status != STATUS_OK) {
		vl53l5_k_log_error("Set ulp err");
		goto out;
	}
	usleep_range(2000, 2100);

	vl53l5_ioctl_read_generic_shape(p_module);
	if (status != STATUS_OK) {
		vl53l5_k_log_error("Set ulp err");
		p_module->load_calibration = false;
	} else {
		p_module->load_calibration = true;
	}
	usleep_range(1000, 1100);

	vl53l5_ioctl_set_device_parameters(p_module, NULL, VL53L5_CFG__BACK_TO_BACK__8X8__15HZ);
	if (status != STATUS_OK) {
		vl53l5_k_log_error("Set 8x8 err");
		goto out;
	}
	vl53l5_ioctl_set_device_parameters(p_module, NULL, VL53L5_CFG__STATIC__SS_4PC__VCSELCP_OFF);
	if (status != STATUS_OK) {
		vl53l5_k_log_error("Set vcsell err");
		goto out;
	}
	usleep_range(1000, 1100);
	return status;
out:
	vl53l5_k_log_error("Reset failed %d", status);
	vl53l5_k_store_error(p_module, status);
	return status;
}

static ssize_t vl53l5_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	u8 val = 0;
	int ret;
	int status = 0;
	int cal_type = 0;

	vl53l5_k_re_init(p_module);

	if (p_module->last_driver_error == VL53L5_PROBE_FAILED) {
		vl53l5_k_log_error("failed %d", p_module->last_driver_error);
		return count;
	}

	ret = kstrtou8(buf, 10, &val);

	vl53l5_k_log_debug("Lock");
	mutex_lock(&p_module->mutex);

	switch(val) {
	case 0: /* Calibration Load */
		status = vl53l5_input_report(p_module, 6, CMD_CHECK_CAL_FILE_TYPE);
		if (status < 0)
			vl53l5_k_log_error("could not find file_list");

		if ((p_module->file_list & 3) == 3) {
			vl53l5_k_log_info("Do user cal: %d", p_module->file_list);
			cal_type = USER_CAL;
		} else {
			vl53l5_k_log_info("Do fac cal: %d", p_module->file_list);
			cal_type = FAC_CAL;
		}

		if (cal_type != USER_CAL)
			vl53l5_load_factory_calibration(p_module);
		else
			vl53l5_load_open_calibration(p_module);

		break;
	case 1:  /* Factory Calibration */
		vl53l5_k_log_info("Do CAL\n");

		if (p_module->load_calibration == false) {
			vl53l5_k_log_info("generic_shape is not loaded");
			vl53l5_k_log_debug("unLock");
			mutex_unlock(&p_module->mutex);
			return count;
		}

		vl53l5_reset(p_module);
		ret = vl53l5_perform_calibration(p_module, 2);
		usleep_range(2000, 2100);
		if (ret != STATUS_OK) {
			memset(&p_module->cal_data, 0, sizeof(struct vl53l5_cal_data_t));	
			memcpy(p_module->cal_data.pcal_data, &val, 1);
			p_module->cal_data.size= 1;
			p_module->cal_data.cmd = CMD_WRITE_CAL_FILE;

			vl53l5_input_report(p_module, 2, p_module->cal_data.cmd);
			p_module->read_p2p_cal_data = false;

			vl53l5_reset(p_module);
#ifdef VL53L5_TCDM_DUMP
			_tcdm_dump(p_module);
#endif
		} else {
			p_module->load_calibration = true;
			ret = vl53l5_ioctl_read_p2p_calibration(p_module, false);
			msleep(30);
			if (ret == STATUS_OK)
				p_module->read_p2p_cal_data = true;
			else
				p_module->read_p2p_cal_data = false;

		}
		vl53l5_ioctl_set_device_parameters(p_module, NULL, VL53L5_CFG__BACK_TO_BACK__8X8__15HZ);
		usleep_range(2000, 2100);
		vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_LP_IDLE_COMMS);
		break;
	case 2:  /* Cal Erase */
		vl53l5_k_log_info("Erase CAL\n");

		val = 0;
		memset(&p_module->cal_data, 0, sizeof(struct vl53l5_cal_data_t));	
		memcpy(p_module->cal_data.pcal_data, &val, 1);
		p_module->cal_data.size= 1;
		p_module->cal_data.cmd = CMD_WRITE_CAL_FILE;

		vl53l5_input_report(p_module, 2, p_module->cal_data.cmd);

		p_module->read_p2p_cal_data = false;
		vl53l5_reset(p_module);
		break;
	case 3:
		vl53l5_perform_open_calibration(p_module);
		break;
	default:
		vl53l5_k_log_info("Not support: %d\n", val);
		break;
	}

	vl53l5_k_log_debug("unLock");
	mutex_unlock(&p_module->mutex);

	return count;
}

static ssize_t vl53l5_zone_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "8,8\n");
}

static ssize_t vl53l5_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct vl53l5_k_module_t *p_module = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", p_module->last_driver_error);
}

static DEVICE_ATTR(name, 0440, vl53l5_name_show, NULL);
static DEVICE_ATTR(vendor, 0440, vl53l5_vendor_show, NULL);
static DEVICE_ATTR(fw_version, 0440, vl53l5_firmware_version_show, NULL);
static DEVICE_ATTR(enable, 0660, vl53l5_enable_show, vl53l5_enable_store);
static DEVICE_ATTR(test_mode, 0660, vl53l5_test_mode_show, vl53l5_test_mode_store);
static DEVICE_ATTR(temp, 0440, vl53l5_temp_show, NULL);
static DEVICE_ATTR(test01, 0440, vl53l5_distance_show, NULL);
static DEVICE_ATTR(test02, 0440, vl53l5_peak_signal_show, NULL);
static DEVICE_ATTR(test03, 0440, vl53l5_max_range_show, NULL);
static DEVICE_ATTR(ambient, 0440, vl53l5_ambient_show, NULL);
static DEVICE_ATTR(range_sigma, 0440, vl53l5_range_sigma_show, NULL);
static DEVICE_ATTR(target_status, 0440, vl53l5_target_status_show, NULL);
static DEVICE_ATTR(uid, 0440, vl53l5_uid_show, NULL);
static DEVICE_ATTR(cal_uid, 0440, vl53l5_cal_uid_show, NULL);
static DEVICE_ATTR(mode, 0440, vl53l5_mode_show, NULL);
static DEVICE_ATTR(zone, 0440, vl53l5_zone_show, NULL);
static DEVICE_ATTR(frame_rate, 0440, vl53l5_frame_rate_show, NULL);
static DEVICE_ATTR(cal01, 0440, vl53l5_cal_offset_show, NULL);
static DEVICE_ATTR(cal02, 0440, vl53l5_cal_xtalk_show, NULL);
static DEVICE_ATTR(status, 0440, vl53l5_status_show, NULL);
static DEVICE_ATTR(calibration, 0660, vl53l5_calibration_show, vl53l5_calibration_store);
static DEVICE_ATTR(open_calibration, 0440, vl53l5_open_calibration_show, NULL);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_fw_version,
	&dev_attr_enable,
	&dev_attr_temp,
	&dev_attr_test01,
	&dev_attr_test02,
	&dev_attr_test03,
	&dev_attr_ambient,
	&dev_attr_range_sigma,
	&dev_attr_target_status,
	&dev_attr_cal_uid,
	&dev_attr_uid,
	&dev_attr_mode,
	&dev_attr_zone,
	&dev_attr_frame_rate,
	&dev_attr_cal01,
	&dev_attr_cal02,
	&dev_attr_calibration,
	&dev_attr_open_calibration,
	&dev_attr_status,
	&dev_attr_test_mode,
	NULL,
};


//for sec dump  -----
int vl53l5_dump_data_notify(struct notifier_block *nb,
	unsigned long val, void *v)
{
	struct vl53l5_k_module_t *p_module = container_of(nb, struct vl53l5_k_module_t, dump_nb);

	if ((val == 1) && (p_module != NULL)) {
		vl53l5_k_log_info("probe status %d", p_module->stdev.status_probe);
		vl53l5_k_log_info("cal status %d", !(p_module->stdev.status_cal)?1:p_module->stdev.status_cal);
		vl53l5_k_log_info("cal load %d", p_module->load_calibration);
		vl53l5_k_log_info("cal p2p %d", p_module->read_p2p_cal_data);
		vl53l5_k_log_info("last_driver_error: %d", p_module->last_driver_error);
		vl53l5_k_log_info("device id 0x%02X", p_module->stdev.id_device);
		vl53l5_k_log_info("revision id 0x%02X", p_module->stdev.id_revision);

		vl53l5_k_log_info("state %d, force count %u", p_module->state_preset, p_module->force_suspend_count);
	}
	return 0;
}
//----- for sec dump

#ifdef CONFIG_SENSORS_VL53L5_QCOM
//for sec qc spi config ------
static struct spi_geni_qcom_ctrl_data spi_qcom_config = {
		.spi_cs_clk_delay	= 0,
		.spi_inter_words_delay = 16
};
//------ for sec qc spi config 
#endif
#endif

int vl53l5_k_spi_probe(struct spi_device *spi)
{
	int status = 0;
	struct vl53l5_k_module_t *p_module = NULL;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	unsigned char device_id = 0;
	unsigned char revision_id = 0;
	unsigned char page = 0;
#endif

	LOG_FUNCTION_START("");

	vl53l5_k_log_debug("Allocate module data");
	p_module = kzalloc(sizeof(struct vl53l5_k_module_t), GFP_KERNEL);
	if (!p_module) {
		vl53l5_k_log_error("Failed.");
		status = -ENOMEM;
		goto done;
	}
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
#ifdef CONFIG_SENSORS_VL53L5_QCOM
	//for sec qc spi config ------
	spi->controller_data = &spi_qcom_config;
	spi_dev_put(spi);
	//------ for sec qc spi config
	p_module->stdev.status_probe = 1;
#endif
#endif
	vl53l5_k_log_debug("Assign data to spi handle");
	p_module->spi_info.device = spi;

	vl53l5_k_log_debug("Set client data");
	spi_set_drvdata(spi, p_module);

	vl53l5_k_log_debug("Init kref");
	kref_init(&p_module->spi_info.ref);

	if (spi->dev.of_node) {
		status = vl53l5_parse_dt(&spi->dev, p_module);
		if (status) {
			vl53l5_k_log_error("%s - Failed to parse DT", __func__);
			goto done_cleanup;
		}
	}

	vl53l5_k_log_debug("Set driver");
	status = vl53l5_k_setup(p_module);
	if (status != 0)
		goto done_freemem;

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	status = vl53l5_platform_init(&p_module->stdev);
	if (status != STATUS_OK)
		goto done;

	status = vl53l5_write_multi(&p_module->stdev, 0x7FFF, &page, 1);
	if (status < STATUS_OK)
		goto done;

	status = vl53l5_read_multi(&p_module->stdev, 0x00, &device_id, 1);
	if (status < STATUS_OK)
		goto done_change_page;

	status = vl53l5_read_multi(&p_module->stdev, 0x01, &revision_id, 1);
	if (status < STATUS_OK)
		goto done_change_page;

	vl53l5_k_log_info("device_id (0x%02X), revision_id (0x%02X)",
			  device_id, revision_id);
	
	if ((device_id != 0xF0) || (revision_id != 0x02)) {
		vl53l5_k_log_error("Unsupported device type");
	}
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	p_module->number_of_zones = NUM_OF_ZONE;
	p_module->max_targets_per_zone = NUM_OF_TARGET;
	p_module->print_counter = 8;

	vl53l5_k_log_info("device_id (0x%02X), revision_id (0x%02X)",
			  p_module->stdev.id_device, p_module->stdev.id_revision);

	p_module->input_dev = input_allocate_device();
	if (!p_module->input_dev) {
	    vl53l5_k_log_error("Failed to input_allocate_device\n");
	}
	else
	{
		int ret;
	    p_module->input_dev->name = MODULE_NAME;
	
	    input_set_capability(p_module->input_dev, EV_REL, REL_MISC);
	    input_set_drvdata(p_module->input_dev, p_module);
	
	    ret = input_register_device(p_module->input_dev);
	    if (ret)
		    vl53l5_k_log_info("failed input_register_device(%d)\n", ret);
	}

	status = sensors_register(&p_module->factory_device,
		p_module, sensor_attrs, MODULE_NAME);
	if (status) {
		vl53l5_k_log_error("%s - could not register sensor(%d).\n",
			__func__, status);
	}

	//for sec dump  -----
	p_module->dump_nb.notifier_call = vl53l5_dump_data_notify;
	p_module->dump_nb.priority = 1;

	{
		int ret;
		ret = sensordump_notifier_register(&p_module->dump_nb);
		vl53l5_k_log_info("notifier %d", ret);
	}
	//----- for sec dump

#endif
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
done_change_page:
	page = 2;
	if (status != STATUS_OK)
		(void)vl53l5_write_multi(&p_module->stdev, 0x7FFF, &page, 1);
	else
		status = vl53l5_write_multi(&p_module->stdev, 0x7FFF, &page, 1);
#endif
done:
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	p_module->probe_done = true;
#endif
	LOG_FUNCTION_END(status);
	return status;

done_cleanup:
	vl53l5_k_cleanup(p_module);

done_freemem:
	kfree(p_module);

	LOG_FUNCTION_END(status);
	return status;
}

int vl53l5_k_spi_remove(struct spi_device *device)
{
	int status = 0;
	struct vl53l5_k_module_t *p_module = spi_get_drvdata(device);

	LOG_FUNCTION_START("");

	vl53l5_k_log_debug("Driver cleanup");
	vl53l5_k_cleanup(p_module);

	kref_put(&p_module->spi_info.ref, memory_release);

	LOG_FUNCTION_END(status);
	status = vl53l5_k_convert_error_to_linux_error(status);
	return status;
}
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
void vl53l5_k_spi_shutdown(struct spi_device *device)
{
	struct vl53l5_k_module_t *p_module = spi_get_drvdata(device);

	vl53l5_k_log_info("state %d, err %d", p_module->state_preset, p_module->last_driver_error);

	p_module->last_driver_error = VL53L5_SHUTDOWN;
	if (p_module->state_preset == VL53L5_STATE_RANGING)
		msleep(20);
#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_KERNEL_INTERFACE
	global_p_module = NULL;
#endif

	vl53l5_k_power_onoff(p_module, p_module->iovdd_vreg, p_module->iovdd_vreg_name, 0);
	vl53l5_k_power_onoff(p_module, p_module->avdd_vreg, p_module->avdd_vreg_name, 0);
	
	if (p_module->pinctrl && p_module->pinctrl_vddoff) {
		vl53l5_k_log_info("pinctrl_select_state for vddoff\n");
		pinctrl_select_state(p_module->pinctrl, p_module->pinctrl_vddoff);
	}
}
#endif
static int setup_miscdev(struct vl53l5_k_module_t *p_module)
{
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	int retry = 3;
#endif
	int status = 0;

	vl53l5_k_log_debug("Setting up misc dev");
	p_module->miscdev.minor = MISC_DYNAMIC_MINOR;

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	if (p_module->id == 0)
		strcpy(p_module->name, VL53L5_K_DRIVER_NAME);
	else
		sprintf(p_module->name, "%s_%d", VL53L5_K_DRIVER_NAME,
			p_module->id);
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	strcpy(p_module->name, VL53L5_K_DRIVER_NAME);
#endif

	p_module->miscdev.name = p_module->name;
	p_module->miscdev.fops = &ranging_fops;
	vl53l5_k_log_debug("Misc device reg:%s", p_module->miscdev.name);
	vl53l5_k_log_debug("Reg misc device");
	status = misc_register(&p_module->miscdev);
	if (status != 0) {
		vl53l5_k_log_error("Failed to register misc dev: %d", status);
		goto done;
	}
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	vl53l5_k_log_info("init start");

	p_module->of_cfg.otf_general_cfg.otf__rotation_sel = (uint8_t)p_module->rotation_mode;
	status = 1; // For retry condition

	while ((retry > 0) && (status != STATUS_OK)) {
		status = vl53l5_ioctl_init(p_module);
		usleep_range(3000, 3100);
		vl53l5_k_log_info("Init retry %d with %d", retry, status);
		retry--;
	}

	if (status == STATUS_OK) {
		vl53l5_k_log_info("get_module_info start");
		status = vl53l5_ioctl_get_module_info(p_module, NULL);
	}

	if (status != STATUS_OK) {
#ifdef VL53L5_TCDM_ENABLE
		_tcdm_dump(p_module);
#endif
#ifdef VL53L5_TEST_ENABLE
		msleep(50);
		panic("probe failed %d", p_module->last_driver_error);
#endif
		vl53l5_k_log_info("probe fail");
		p_module->enabled = -1;
		p_module->last_driver_error = VL53L5_PROBE_FAILED;
		vl53l5_k_power_onoff(p_module, p_module->iovdd_vreg, p_module->iovdd_vreg_name, 0);
		vl53l5_k_power_onoff(p_module, p_module->avdd_vreg, p_module->avdd_vreg_name, 0);

		if (p_module->pinctrl && p_module->pinctrl_vddoff) {
			vl53l5_k_log_info("pinctrl_select_state for vddoff\n");
			status = pinctrl_select_state(p_module->pinctrl, p_module->pinctrl_vddoff);
			if (status < 0)
				vl53l5_k_log_error("pinctrl_select_state failed: %d\n", status);
		}
		status = STATUS_OK;
	} else {
#ifdef CONFIG_SENSORS_VL53L5_SUPPORT_KERNEL_INTERFACE
		global_p_module = p_module;
#endif
	}

	usleep_range(2000, 2100);
	vl53l5_ioctl_set_power_mode(p_module, NULL, VL53L5_POWER_STATE_LP_IDLE_COMMS);
	memset(&p_module->range_data, 0, sizeof(p_module->range_data));
	p_module->load_calibration = false;
#endif
done:
	return status;
}

static void cleanup_miscdev(struct vl53l5_k_module_t *p_module)
{
	if (!IS_ERR(p_module->miscdev.this_device) &&
			p_module->miscdev.this_device != NULL) {
		misc_deregister(&p_module->miscdev);
	}
}
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
static void deallocate_dev_id(int id)
{
	mutex_lock(&dev_table_mutex);

	module_table[id] = NULL;

	mutex_unlock(&dev_table_mutex);
}

static void allocate_dev_id(struct vl53l5_k_module_t *p_module)
{
	int i = 0;

	mutex_lock(&dev_table_mutex);

	while ((i < VL53L5_CFG_MAX_DEV) && (module_table[i]))
		i++;

	i = i < VL53L5_CFG_MAX_DEV ? i : -1;

	p_module->id = i;
	if (i != -1) {
		vl53l5_k_log_debug("Obtained device id %d", p_module->id);
		module_table[p_module->id] = p_module;
	}

	mutex_unlock(&dev_table_mutex);
}
#endif
static int vl53l5_k_ioctl_handler(struct vl53l5_k_module_t *p_module,
				  unsigned int cmd, unsigned long arg,
				  void __user *p)
{
	int status = 0;

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	if (!p_module) {
		status = -EINVAL;
		goto exit;
	}
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	if (p_module == NULL) {
		if ((cmd == VL53L5_IOCTL_START)
			|| (cmd == VL53L5_IOCTL_STOP))
			vl53l5_k_log_error("probe failed");

		status = -EINVAL;
		goto exit;
	} else if ((p_module->last_driver_error == VL53L5_PROBE_FAILED)
			|| (p_module->last_driver_error == VL53L5_SHUTDOWN)
			|| (p_module->suspend_state == true)) {
		if ((cmd == VL53L5_IOCTL_START)
			|| (cmd == VL53L5_IOCTL_STOP))
			vl53l5_k_log_error("failed %d", p_module->last_driver_error);
		status = -EINVAL;
		if (p_module->suspend_state == true)
			vl53l5_k_log_error("cmd %d is called in suspend", cmd);
		goto exit;
	}
#endif
	switch (cmd) {
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	case VL53L5_IOCTL_INIT:
		vl53l5_k_log_debug("VL53L5_IOCTL_INIT");
		status = vl53l5_ioctl_init(p_module);
		break;
	case VL53L5_IOCTL_TERM:
		vl53l5_k_log_debug("VL53L5_IOCTL_TERM");
		status = vl53l5_ioctl_term(p_module);
		break;
	case VL53L5_IOCTL_GET_VERSION:
		vl53l5_k_log_debug("VL53L5_IOCTL_GET_VERSION");
		status = vl53l5_ioctl_get_version(p_module, p);
		break;
#endif
	case VL53L5_IOCTL_START:
		vl53l5_k_log_debug("Lock");
		mutex_lock(&p_module->mutex);
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
		vl53l5_k_log_debug("VL53L5_IOCTL_START");
		status = vl53l5_ioctl_start(p_module, p);
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
		vl53l5_k_log_info("START");
		status = vl53l5_ioctl_start(p_module, p, 1);
#ifdef VL53L5_TEST_ENABLE
		if (status == STATUS_OK) {
			vl53l5_k_log_error("test for reset");
			status = -1;
		}
#endif
		if ((status != STATUS_OK) || (p_module->last_driver_error != STATUS_OK)) {
			status = vl53l5_ioctl_init(p_module);
			vl53l5_k_log_error("fail reset %d", status);
			status = vl53l5_ioctl_start(p_module, p, 1);
		}

		p_module->of_cfg.otf_general_cfg.otf__rotation_sel = (uint8_t)p_module->rotation_mode;
		vl53l5_k_log_info("start ro %u", p_module->of_cfg.otf_general_cfg.otf__rotation_sel);
#endif
		vl53l5_k_log_debug("unLock");
		mutex_unlock(&p_module->mutex);
		break;
	case VL53L5_IOCTL_STOP:
		vl53l5_k_log_debug("Lock");
		mutex_lock(&p_module->mutex);
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
		vl53l5_k_log_debug("VL53L5_IOCTL_STOP");
		status = vl53l5_ioctl_stop(p_module, p);
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
		vl53l5_k_log_info("STOP");
		status = vl53l5_ioctl_stop(p_module, p, 1);

#ifdef VL53L5_TEST_ENABLE
		if (status == STATUS_OK) {
			vl53l5_k_log_error("test for reset");
			status = -1;
		}
#endif

		if ((status != STATUS_OK) || (p_module->last_driver_error != STATUS_OK)) {
			status = vl53l5_ioctl_init(p_module);
			vl53l5_k_log_error("fail reset %d", status);
			status = vl53l5_ioctl_stop(p_module, p, 1);
		}
#endif
		vl53l5_k_log_debug("unLock");
		mutex_unlock(&p_module->mutex);
		break;
	case VL53L5_IOCTL_GET_RANGE:
		vl53l5_k_log_debug("Lock");
		mutex_lock(&p_module->mutex);
		vl53l5_k_log_debug("GET_RANGE");
		status = vl53l5_ioctl_get_range(p_module, p);
		vl53l5_k_log_debug("unLock");
		mutex_unlock(&p_module->mutex);
		break;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	case VL53L5_IOCTL_BLOCKING_GET_RANGE:
		vl53l5_k_log_debug("BLOCKING_GET_RANGE");
		status = vl53l5_ioctl_blocking_get_range(p_module, p);
		break;
	case VL53L5_IOCTL_SET_DEVICE_PARAMETERS:
		vl53l5_k_log_debug("VL53L5_IOCTL_SET_DEVICE_PARAMETERS");
		status = vl53l5_ioctl_set_device_parameters(p_module, p);
		break;
	case VL53L5_IOCTL_GET_DEVICE_PARAMETERS:
		vl53l5_k_log_debug("VL53L5_IOCTL_GET_DEVICE_PARAMETERS");
		status = vl53l5_ioctl_get_device_parameters(p_module, p);
		break;
	case VL53L5_IOCTL_GET_MODULE_INFO:
		vl53l5_k_log_debug("VL53L5_IOCTL_GET_MODULE_INFO");
		status = vl53l5_ioctl_get_module_info(p_module, p);
		break;
	case VL53L5_IOCTL_GET_ERROR_INFO:
		vl53l5_k_log_debug("VL53L5_IOCTL_GET_ERROR_INFO");
		status = vl53l5_ioctl_get_error_info(p_module, p);
		break;
	case VL53L5_IOCTL_SET_POWER_MODE:
		vl53l5_k_log_debug("VL53L5_IOCTL_SET_POWER_MODE");
		status = vl53l5_ioctl_set_power_mode(p_module, p);
		break;
	case VL53L5_IOCTL_POLL_DATA_READY:
		vl53l5_k_log_debug("VL53L5_IOCTL_POLL_DATA_READY");
		status = vl53l5_ioctl_poll_data_ready(p_module);
		break;
	case VL53L5_IOCTL_READ_P2P_FILE:
		vl53l5_k_log_debug("VL53L5_IOCTL_READ_P2P_FILE");
		status = vl53l5_ioctl_read_p2p_calibration(p_module);
		break;
	case VL53L5_IOCTL_PERFORM_CALIBRATION_600:
		vl53l5_k_log_debug("VL53L5_IOCTL_PERFORM_CALIBRATION_600");
		status = vl53l5_perform_calibration(p_module, 0);
		break;
	case VL53L5_IOCTL_PERFORM_CHARACTERISATION_600:
		vl53l5_k_log_debug("VL53L5_IOCTL_PERFORM_CHARACTERISATION_600");
		status = vl53l5_perform_calibration(p_module, 1);
		break;
	case VL53L5_IOCTL_PERFORM_CALIBRATION_300:
		vl53l5_k_log_debug("VL53L5_IOCTL_PERFORM_CALIBRATION_300");
		status = vl53l5_perform_calibration(p_module, 2);
		break;
	case VL53L5_IOCTL_READ_SHAPE_FILE:
		vl53l5_k_log_debug("VL53L5_IOCTL_READ_SHAPE_FILE");
		status = vl53l5_ioctl_read_shape_calibration(p_module);
		break;
	case VL53L5_IOCTL_READ_GENERIC_SHAPE:
		vl53l5_k_log_debug("VL53L5_IOCTL_READ_GENERIC_SHAPE");
		status = vl53l5_ioctl_read_generic_shape(p_module);
		break;
	case VL53L5_IOCTL_PERFORM_CHARACTERISATION_1000:
		vl53l5_k_log_debug(
			"VL53L5_IOCTL_PERFORM_CHARACTERISATION_1000");
		status = vl53l5_perform_calibration(p_module, 3);
		break;
	case VL53L5_IOCTL_SET_LTF_CONFIG:
		vl53l5_k_log_debug("VL53L5_IOCTL_SET_LTF_CONFIG");
		status = vl53l5_ioctl_set_ltf_config(p_module, p);
		break;
#ifdef STM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
	case VL53L5_IOCTL_SET_SINGLE_ZONE_CONFIG:
		vl53l5_k_log_debug("VL53L5_IOCTL_SET_SINGLE_ZONE_CONFIG");
		status = vl53l5_ioctl_set_single_zone_config(p_module, p);
		break;
#endif
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	case VL53L5_IOCTL_GET_CAL_DATA:
		vl53l5_k_log_info("GET_CAL_DATA");
		status = vl53l5_ioctl_get_cal_data(p_module, p);
		break;
	case VL53L5_IOCTL_SET_CAL_DATA:
		vl53l5_k_log_info("SET_CAL_DATA");
		status = vl53l5_ioctl_set_cal_data(p_module, p);
		break;
	case VL53L5_IOCTL_SET_PASS_FAIL:
		vl53l5_k_log_info("SET_PASS_FAIL");
		status = vl53l5_ioctl_set_pass_fail(p_module, p);
		break;
	case VL53L5_IOCTL_SET_FILE_LIST:
		vl53l5_k_log_info("SET_FILE_LIST");
		status = vl53l5_ioctl_set_file_list(p_module, p);
		break;
#endif
	default:
		status = -EINVAL;
	}

exit:
	return status;
}

int vl53l5_k_open(struct inode *inode, struct file *file)
{
	int status = STATUS_OK;
	struct vl53l5_k_module_t *p_module = NULL;

	LOG_FUNCTION_START("");

	p_module = container_of(file->private_data, struct vl53l5_k_module_t,
				miscdev);

	vl53l5_k_log_debug("Get SPI bus");

	kref_get(&p_module->spi_info.ref);

	LOG_FUNCTION_END(status);

	vl53l5_k_store_error(p_module, status);
	status = vl53l5_k_convert_error_to_linux_error(status);
	return status;
}

int vl53l5_k_release(struct inode *inode, struct file *file)
{
	int status = STATUS_OK;
	struct vl53l5_k_module_t *p_module = NULL;

	LOG_FUNCTION_START("");

	p_module = container_of(file->private_data, struct vl53l5_k_module_t,
			    miscdev);

	kref_put(&p_module->spi_info.ref, memory_release);

	LOG_FUNCTION_END(status);

	vl53l5_k_store_error(p_module, status);
	status = vl53l5_k_convert_error_to_linux_error(status);
	return status;
}

long vl53l5_k_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int status = STATUS_OK;
	struct vl53l5_k_module_t *p_module = NULL;

	LOG_FUNCTION_START("");

	p_module = container_of(file->private_data, struct vl53l5_k_module_t,
			    miscdev);

	status = vl53l5_k_ioctl_handler(p_module, cmd, arg, (void __user *)arg);

	LOG_FUNCTION_END(status);
	vl53l5_k_store_error(p_module, status);
	status = vl53l5_k_convert_error_to_linux_error(status);
	return status;
}

void vl53l5_k_cleanup(struct vl53l5_k_module_t *p_module)
{
	LOG_FUNCTION_START("");

	vl53l5_k_log_debug("Unregistering misc device");
	cleanup_miscdev(p_module);

	if (p_module->state_preset == VL53L5_STATE_RANGING) {
		vl53l5_k_log_debug("Stop ranging");
		(void)vl53l5_stop(&p_module->stdev, &p_module->ranging_flags);
#ifdef VL53L5_INTERRUPT
			vl53l5_k_log_debug("Stop interrupt");
			vl53l5_k_stop_interrupt(p_module);
#endif
#ifdef VL53L5_WORKER
			vl53l5_k_log_debug("Cancelling worker");
			vl53l5_k_cancel_worker(p_module);
#endif
	}
	if (p_module->state_preset == VL53L5_STATE_INITIALISED)
		(void)vl53l5_ioctl_term(p_module);

	vl53l5_k_log_debug("Acquiring lock");
	mutex_lock(&p_module->mutex);
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_k_log_debug("Deallocating dev id");
	deallocate_dev_id(p_module->id);
#endif
	vl53l5_k_log_debug("Setting state flags");
	p_module->state_preset = VL53L5_STATE_NOT_PRESENT;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_k_log_debug("Platform terminate");
	vl53l5_platform_terminate(&p_module->stdev);
#endif

	vl53l5_k_log_debug("Releasing all gpios");
	vl53l5_k_release_gpios(p_module);

	vl53l5_k_log_debug("Releasing mutex");
	mutex_unlock(&p_module->mutex);

	LOG_FUNCTION_END(0);
}

int vl53l5_k_setup(struct vl53l5_k_module_t *p_module)
{
	int status = 0;
	unsigned int ltf_cfg = 0;
#ifdef STM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
	unsigned int sz_cfg = 0;
#endif
	LOG_FUNCTION_START("");
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_k_log_debug("Acquire device id");
	allocate_dev_id(p_module);
	if (p_module->id < 0) {
		vl53l5_k_log_error("Failed");
		status = -1;
		goto done;
	}
#endif
	vl53l5_k_log_debug("Init device mutex");
	mutex_init(&p_module->mutex);

	vl53l5_k_log_debug("Acquiringe lock");
	mutex_lock(&p_module->mutex);
	vl53l5_k_log_debug("Set default sleep time");
	p_module->polling_sleep_time_ms = VL53L5_SLEEP_TIME_MS;
	p_module->range_mode = VL53L5_RANGE_SERVICE_DEFAULT;
#ifdef VL53L5_WORKER
	vl53l5_k_log_debug("Initialising delayed work");
	status = vl53l5_k_init_worker(p_module);
	if (status != 0) {
		vl53l5_k_log_error("Worker setup failed: %d", status);
		goto done;
	}
	p_module->range_mode = VL53L5_RANGE_SERVICE_WORKER;
#endif
#ifdef VL53L5_INTERRUPT
	p_module->range_mode = VL53L5_RANGE_SERVICE_INTERRUPT;
#endif
	vl53l5_k_log_debug("Range mode %d",
			   p_module->range_mode);

	vl53l5_k_log_debug("INIT waitqueue");
	INIT_LIST_HEAD(&p_module->reader_list);
	init_waitqueue_head(&p_module->wait_queue);

	vl53l5_k_log_debug("Set state flags");
	p_module->state_preset = VL53L5_STATE_PRESENT;

	vl53l5_k_log_debug("Set Long Tail Filter config");
	ltf_cfg = VL53L5_LTF_CFG_PRESET__LUT_RANGE_TUNING_1;
	(void)vl53l5_long_tail_filter__set_cfg(
		ltf_cfg, &p_module->stdev, &p_module->ltf_cfg);
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_ltf_get_version(&p_module->ltf_version);
	vl53l5_k_log_info("Long Tail Filter ver: %d.%d.%d.%d",
			p_module->ltf_version.version__major,
			p_module->ltf_version.version__minor,
			p_module->ltf_version.version__build,
			p_module->ltf_version.version__revision);
#endif
#ifdef STM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
	vl53l5_k_log_debug("Set single zone config");
	sz_cfg = VL53L5_PSZ_CFG_PRESET__TUNING_1;
	(void)vl53l5_pseudo_single_zone__set_cfg(
		sz_cfg, &p_module->stdev, &p_module->sz_cfg);
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_psz_get_version(&p_module->sz_version);
	vl53l5_k_log_info("Psuedo single zone ver: %d.%d.%d.%d",
			p_module->sz_version.version__major,
			p_module->sz_version.version__minor,
			p_module->sz_version.version__build,
			p_module->sz_version.version__revision);
#endif
#endif

	vl53l5_k_log_debug("Set Output filter");
	p_module->of_cfg.otf_general_cfg.otf__tgt_status_op_enables =
				VL53L5_OTF__TGT_OP_FILTER_FLAGS__VALID_ONLY;
	p_module->of_cfg.otf_general_cfg.otf__rotation_sel =
				VL53L5_OTF_ROTATION_SEL__NONE;
	p_module->of_cfg.otf_general_cfg.otf__mode =
				VL53L5_OTF_MODE__ENABLED,
	p_module->of_cfg.otf_general_cfg.otf__single_zone_op_en = 1;
	p_module->of_cfg.otf_general_cfg.otf__range_clip_en = 1;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	vl53l5_otf_get_version(&p_module->of_version);
	vl53l5_k_log_info("Output filter ver: %d.%d.%d.%d",
			p_module->of_version.version__major,
			p_module->of_version.version__minor,
			p_module->of_version.version__build,
			p_module->of_version.version__revision);
#endif
	vl53l5_k_log_debug("Releasing mutex");
	mutex_unlock(&p_module->mutex);

	status = setup_miscdev(p_module);
	if (status != 0) {
		vl53l5_k_log_error("Misc dev setup failed: %d", status);
		goto done;
	}

done:
	LOG_FUNCTION_END(status);
	return status;
}

static int __init vl53l5_k_init(void)
{
	int status = 0;

	LOG_FUNCTION_START("");

	vl53l5_k_log_info("Init %s driver", VL53L5_K_DRIVER_NAME);

	vl53l5_k_log_debug("Init spi bus");
	status = spi_register_driver(&vl53l5_k_spi_driver);
	if (status != 0) {
		vl53l5_k_log_error("Failed init bus: %d", status);
		goto done;
	}

	vl53l5_k_log_info("Kernel driver version: %d.%d.%d.%d",
			  VL53L5_K_VER_MAJOR,
			  VL53L5_K_VER_MINOR,
			  VL53L5_K_VER_BUILD,
			  VL53L5_K_VER_REVISION);

done:
	LOG_FUNCTION_END(status);
	status = vl53l5_k_convert_error_to_linux_error(status);
	return status;
}

static void __exit vl53l5_k_exit(void)
{
	LOG_FUNCTION_START("");

	vl53l5_k_log_info("Exiting %s driver", VL53L5_K_DRIVER_NAME);

	vl53l5_k_log_debug("Exiting spi bus");
	spi_unregister_driver(&vl53l5_k_spi_driver);
	vl53l5_k_log_debug("Cleaning up");
	vl53l5_k_clean_up_spi();

	LOG_FUNCTION_END(0);
}

#if defined(CONFIG_DEFERRED_INITCALLS)
deferred_module_init(vl53l5_k_init);
#else
module_init(vl53l5_k_init);
#endif
module_exit(vl53l5_k_exit);

MODULE_DESCRIPTION("Sample VL53L5 ToF client driver");
MODULE_LICENSE("GPL v2");
