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

#include "dci_defs.h"
#include "dci_luts.h"
#include "vl53l5_otf_version_defs.h"
#include "vl53l5_otf_defs.h"
#include "vl53l5_otf_luts.h"
#include "vl53l5_output_target_filter_funcs.h"
#include "vl53l5_output_target_filter_main.h"

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
void vl53l5_otf_get_version(
	struct  common_grp__version_t  *pversion)
{

	pversion->version__major = VL53L5_OTF_DEF__VERSION__MAJOR;
	pversion->version__minor = VL53L5_OTF_DEF__VERSION__MINOR;
	pversion->version__build = VL53L5_OTF_DEF__VERSION__BUILD;
	pversion->version__revision = VL53L5_OTF_DEF__VERSION__REVISION;
}
#endif

void vl53l5_output_target_filter_main(
	struct   vl53l5_range_results_t             *presults,
	struct   vl53l5_otf_cfg_dev_t     *pcfg_dev,
	struct   vl53l5_otf_int_dev_t     *pint_dev,
	struct   vl53l5_range_results_t             *ptmp,
	struct   vl53l5_otf_op_dev_t      *pop_dev,
	struct   vl53l5_otf_err_dev_t     *perr_dev)
{

	int32_t   no_of_zones = 0;
	int32_t   iz  = 0;

	int32_t   oz  = 0;

	int32_t   exit_function = 0;
	int32_t   single_zone_en = 0;

	struct   common_grp__status_t *perror = &(perr_dev->otf_error_status);

	no_of_zones = (int32_t)presults->common_data.rng__no_of_zones;

	if (pcfg_dev->otf_general_cfg.otf__mode != VL53L5_OTF_MODE__ENABLED)
		exit_function = 1;

	single_zone_en =
		(pcfg_dev->otf_general_cfg.otf__single_zone_op_en > 0U) &&
		(no_of_zones < (int32_t)VL53L5_MAX_ZONES) &&
		(pop_dev != NULL);

	vl53l5_otf_clear_data(
		pop_dev,
		perr_dev);

	if (perror->status__type == VL53L5_OTF_ERROR_NONE &&
		exit_function == 0) {
		vl53l5_otf_init_zone_order_coeffs(
			no_of_zones,
			(int32_t)pcfg_dev->otf_general_cfg.otf__rotation_sel,
			&(pint_dev->otf_zone_order_cfg),
			perror);
	}

	if (perror->status__type == VL53L5_OTF_ERROR_NONE &&
		exit_function == 0)
		memcpy(ptmp, presults, sizeof(struct vl53l5_range_results_t));

	for (oz = 0 ; oz < no_of_zones; oz++) {

		if (perror->status__type == VL53L5_OTF_ERROR_NONE &&
			exit_function == 0) {
			iz =
				vl53l5_otf_new_sequence_idx(
					oz,
					&(pint_dev->otf_zone_order_cfg),
					perror);
		}

		if (perror->status__type == VL53L5_OTF_ERROR_NONE &&
			exit_function == 0) {
			vl53l5_otf_rotate_per_zone_data(
				iz,
				oz,
				ptmp,
				presults);
		}

		if (perror->status__type == VL53L5_OTF_ERROR_NONE &&
			exit_function == 0) {
			vl53l5_otf_target_filter(
				iz,
				oz,
				pcfg_dev->otf_general_cfg.otf__tgt_status_op_enables,
				pcfg_dev->otf_general_cfg.otf__range_clip_en,
				ptmp,
				presults,
				perror);
		}
	}

	if (single_zone_en &&
		perror->status__type == VL53L5_OTF_ERROR_NONE &&
		exit_function == 0) {
		vl53l5_otf_target_filter(
			no_of_zones,
			no_of_zones,
			pcfg_dev->otf_general_cfg.otf__tgt_status_op_enables,
			pcfg_dev->otf_general_cfg.otf__range_clip_en,
			ptmp,
			presults,
			perror);
	}

	if (single_zone_en &&
		perror->status__type == VL53L5_OTF_ERROR_NONE &&
		exit_function == 0) {
		vl53l5_otf_init_single_zone_data(
			no_of_zones,
			pcfg_dev->otf_general_cfg.otf__range_clip_en,
			presults,
			pop_dev,
			perror);
	}
}
