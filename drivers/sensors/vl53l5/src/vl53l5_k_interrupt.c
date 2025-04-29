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
#include <linux/irq.h>
#include <linux/gpio.h>
#include "vl53l5_k_interrupt.h"
#include "vl53l5_k_driver_config.h"
#include "vl53l5_k_logging.h"
#include "vl53l5_k_range_wait_handler.h"
#include "vl53l5_k_error_converter.h"
#include "vl53l5_api_ranging.h"
#include "vl53l5_long_tail_filter.h"
#include "vl53l5_output_target_filter.h"
#ifdef STM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
#include "vl53l5_pseudo_single_zone.h"
#include "vl53l5_pseudo_single_zone__set_cfg.h"
#endif

static int _interrupt_get_range_data(struct vl53l5_k_module_t *p_module)
{
	int status = STATUS_OK;

	LOG_FUNCTION_START("");

	status = vl53l5_get_range_data(&p_module->stdev);
	if (status != STATUS_OK)
		goto out;

	status = vl53l5_decode_range_data(&p_module->stdev,
					  &p_module->range.data);
	if (status != STATUS_OK)
		goto out;

	status  = vl53l5_long_tail_filter(&p_module->stdev,
					  &p_module->range.data.core,
					  &p_module->ltf_cfg);
	if (status != STATUS_OK)
		goto out;
#ifdef STM_VL53L5_SUPPORT_PSEUDO_SINGLE_ZONE
	status = vl53l5_pseudo_single_zone(
					&p_module->stdev,
					&p_module->range.data.core,
					&p_module->sz_cfg,
					&p_module->sz_int,
					&p_module->sz_data);
	if (status != STATUS_OK)
		goto out;
#endif

	p_module->range.is_valid = 1;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	p_module->range.count++;
	p_module->polling_count = 0;
#endif

	status = vl53l5_output_target_filter(
					&p_module->stdev,
					&p_module->range.data.core,
					&p_module->of_cfg,
					&p_module->range.rotated,
					&p_module->range.data.single_zone_data);

out:
	if (status != STATUS_OK) {
		status = vl53l5_read_device_error(&p_module->stdev, status);
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
		vl53l5_k_log_error("Failed: %d", status);
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
		if (p_module->irq_is_active)
			vl53l5_k_log_error("Failed: %d", status);
		else
			vl53l5_k_log_info("%d", status);
#endif
	}

	vl53l5_k_store_error(p_module, status);

	LOG_FUNCTION_END(status);

	return status;
}

static irqreturn_t vl53l5_interrupt_handler(int irq, void *dev_id)
{
	struct vl53l5_k_module_t *p_module = (struct vl53l5_k_module_t *)dev_id;

	if (!p_module->irq_wq_is_running) {
		vl53l5_k_log_debug("Interrupt handled");
		queue_work(p_module->irq_wq, &p_module->data_work);
	}

	return IRQ_HANDLED;
}

static void vl53l5_irq_workqueue(struct work_struct *data_work)
{
	struct vl53l5_k_module_t *p_module;
	int status = STATUS_OK;

	p_module = container_of((struct work_struct *)data_work,
						struct vl53l5_k_module_t, data_work);

	vl53l5_k_log_debug("Lock");
	mutex_lock(&p_module->mutex);

	p_module->irq_wq_is_running = true;

	status = _interrupt_get_range_data(p_module);
	if (status == STATUS_OK) {
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
		vl53l5_k_wake_up_wait_list(p_module);
#endif
		vl53l5_k_log_debug("Interrupt handled");
	} else {
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
		if (p_module->irq_is_active)
			vl53l5_k_log_error("Interrupt not handled %d", status);
#endif
		status = vl53l5_read_device_error(&p_module->stdev, status);
		vl53l5_k_store_error(p_module, status);
		vl53l5_k_log_debug("Interrupt not handled");
	}

	p_module->irq_wq_is_running	= false;
	vl53l5_k_log_debug("unLock");
	mutex_unlock(&p_module->mutex);
}

int vl53l5_k_start_interrupt(struct vl53l5_k_module_t *p_module)
{
	int status = STATUS_OK;
	const char *p_dev_name = p_module->name;

	LOG_FUNCTION_START("");

	if (p_module->gpio.interrupt < 0) {
		status = VL53L5_K_ERROR_DEVICE_INTERRUPT_NOT_OWNED;
		goto out;
	}

	p_module->irq_val = gpio_to_irq(p_module->gpio.interrupt);

	p_module->irq_wq = create_workqueue("vl53l5_irq_wq");
	if (!p_module->irq_wq) {
		status = -ENOMEM;
		vl53l5_k_log_error("could not create irq work");
		goto out;
	} else {
		INIT_WORK(&p_module->data_work, vl53l5_irq_workqueue);
	}

	status = request_threaded_irq(p_module->irq_val,
				      NULL,
				      vl53l5_interrupt_handler,
				      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				      p_dev_name,
				      p_module);
	if (status) {
		vl53l5_k_log_error("Unable to assign IRQ: %d",
				   p_module->irq_val);
		goto out;
	} else {
		vl53l5_k_log_debug("IRQ %d now assigned",
				  p_module->irq_val);
		p_module->gpio.interrupt_owned = 1;
		p_module->irq_is_running = true;
	}

out:
	LOG_FUNCTION_END(status);
	return status;
}

int vl53l5_k_stop_interrupt(struct vl53l5_k_module_t *p_module)
{
	int status = STATUS_OK;

	LOG_FUNCTION_START("");

	disable_irq(p_module->irq_val);
	vl53l5_k_log_info("disable irq");

	free_irq(p_module->irq_val, p_module);
	vl53l5_k_log_debug("IRQ %d now free", p_module->irq_val);
	p_module->irq_val = 0;
	p_module->irq_is_running = false;

	LOG_FUNCTION_END(status);
	return status;
}
