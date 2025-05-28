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

#include "vl53l5_platform_algo_macros.h"
#include "vl53l5_platform_algo_limits.h"
#include "vl53l5_results_build_config.h"
#include "dci_defs.h"
#include "dci_luts.h"
#include "vl53l5_otf_defs.h"
#include "vl53l5_otf_luts.h"
#include "vl53l5_output_target_filter_funcs.h"
#include <stddef.h>

void vl53l5_otf_clear_data(
	struct vl53l5_otf_op_dev_t          *pop_dev,
	struct vl53l5_otf_err_dev_t         *perr_dev)
{

	struct common_grp__status_t    *perror   = &(perr_dev->otf_error_status);
	struct common_grp__status_t    *pwarning = &(perr_dev->otf_warning_status);

	if (pop_dev != NULL) {
		memset(&(pop_dev->otf_zone_data), 0,
			   sizeof(struct vl53l5_otf__op__zone_data_t));
		memset(&(pop_dev->otf_target_data), 0,
			   sizeof(struct vl53l5_otf__op__target_data_t));
	}

	perror->status__group = VL53L5_ALGO_OUTPUT_TARGET_FILTER_GROUP_ID;
	perror->status__stage_id = VL53L5_OTF_STAGE_ID__MAIN;
	perror->status__type = VL53L5_OTF_ERROR_NONE;
	perror->status__debug_0 = 0U;
	perror->status__debug_1 = 0U;
	perror->status__debug_2 = 0U;

	pwarning->status__group = VL53L5_ALGO_OUTPUT_TARGET_FILTER_GROUP_ID;
	pwarning->status__stage_id = VL53L5_OTF_STAGE_ID__MAIN;
	pwarning->status__type = VL53L5_OTF_WARNING_NONE;
	pwarning->status__debug_0 = 0U;
	pwarning->status__debug_1 = 0U;
	pwarning->status__debug_2 = 0U;
}

void vl53l5_otf_init_zone_order_coeffs(
	int32_t    no_of_zones,
	int32_t    rotation_sel,
	struct     vl53l5_otf__op__zone_order__cfg_t  *pdata,
	struct     common_grp__status_t               *perror)
{

	int16_t grid_size = 0;

	perror->status__stage_id = VL53L5_OTF_STAGE_ID__INIT_ZONE_ORDER_COEFFS;
	perror->status__type = VL53L5_OTF_ERROR_NONE;

	switch (no_of_zones) {
	case 64:
		grid_size = 8;
		break;
	case 16:
		grid_size = 4;
		break;
	default:
		perror->status__type = VL53L5_OTF_ERROR_UNSUPPORTED_NO_OF_ZONES;
		break;
	}

	pdata->otf__no_of_zones = (int16_t)no_of_zones;
	pdata->otf__grid_size = grid_size;

	switch (rotation_sel) {
	case VL53L5_OTF_ROTATION_SEL__NONE:
		pdata->otf__col_m =  0;
		pdata->otf__col_x =  1;
		pdata->otf__row_m =  0;
		pdata->otf__row_x =  grid_size;
		break;
	case VL53L5_OTF_ROTATION_SEL__CW_90:
		pdata->otf__col_m = (grid_size - 1) * grid_size;
		pdata->otf__col_x =  -grid_size;
		pdata->otf__row_m =  0;
		pdata->otf__row_x =  1;
		break;
	case VL53L5_OTF_ROTATION_SEL__CW_180:
		pdata->otf__col_m =  0;
		pdata->otf__col_x = -1;
		pdata->otf__row_m = (grid_size * grid_size) - 1;
		pdata->otf__row_x =  -grid_size;
		break;
	case VL53L5_OTF_ROTATION_SEL__CW_270:
		pdata->otf__col_m =  0;
		pdata->otf__col_x =  grid_size;
		pdata->otf__row_m =  grid_size - 1;
		pdata->otf__row_x =  -1;
		break;
	case VL53L5_OTF_ROTATION_SEL__MIRROR_X:
		pdata->otf__col_m =  grid_size - 1;
		pdata->otf__col_x = -1;
		pdata->otf__row_m =  0;
		pdata->otf__row_x =  grid_size;
		break;
	case VL53L5_OTF_ROTATION_SEL__MIRROR_Y:
		pdata->otf__col_m =  0;
		pdata->otf__col_x =  1;
		pdata->otf__row_m = (grid_size - 1) * grid_size;
		pdata->otf__row_x =  -grid_size;
		break;
	default:
		perror->status__type = VL53L5_OTF_ERROR_INVALID_ROTATION_SEL;
		break;
	}
}

int32_t vl53l5_otf_new_sequence_idx(
	int32_t    zone_id,
	struct     vl53l5_otf__op__zone_order__cfg_t    *pdata,
	struct     common_grp__status_t                 *perror)
{

	int32_t  r = 0U;
	int32_t  c = 0U;
	int32_t  sequence_idx = 0U;

	perror->status__stage_id = VL53L5_OTF_STAGE_ID__NEW_SEQUENCE_IDX;
	perror->status__type = VL53L5_OTF_ERROR_NONE;

	if (zone_id < 0 || zone_id > (int32_t)pdata->otf__no_of_zones) {
		sequence_idx = -1;
		perror->status__type = VL53L5_OTF_ERROR_INVALID_ZONE_ID;
	} else {
		r = zone_id / pdata->otf__grid_size;
		c = zone_id % pdata->otf__grid_size;

		sequence_idx = ((r * pdata->otf__row_x) + pdata->otf__row_m);
		sequence_idx += ((c * pdata->otf__col_x) + pdata->otf__col_m);
	}

	return sequence_idx;
}

void  vl53l5_otf_init_single_zone_data(
	int32_t                              z,
	uint8_t                              range_clip_en,
	struct     vl53l5_range_results_t             *presults,
	struct     vl53l5_otf_op_dev_t      *pop_dev,
	struct     common_grp__status_t     *perror)
{

	int32_t   t = 0;
	int32_t   i = 0;
	int32_t   max_targets =
		(int32_t)presults->common_data.rng__max_targets_per_zone;
	int32_t   no_of_targets =
		(int32_t)presults->per_zone_results.rng__no_of_targets[z];
	int16_t   range_mm = 0;

	perror->status__stage_id = VL53L5_OTF_STAGE_ID__INIT_SINGLE_ZONE_DATA;
	perror->status__type = VL53L5_OTF_ERROR_INVALID_SINGLE_ZONE_ID;

	if (no_of_targets > (int32_t)VL53L5_OTF_DEF__MAX_TARGETS_PER_ZONE)
		no_of_targets = (int32_t)VL53L5_OTF_DEF__MAX_TARGETS_PER_ZONE;

	if (z < (int32_t)VL53L5_MAX_ZONES) {
		perror->status__type = VL53L5_OTF_ERROR_NONE;

#ifdef VL53L5_AMB_RATE_KCPS_PER_SPAD_ON
		pop_dev->otf_zone_data.amb_rate_kcps_per_spad =
			presults->per_zone_results.amb_rate_kcps_per_spad[z];
#endif
#ifdef VL53L5_EFFECTIVE_SPAD_COUNT_ON
		pop_dev->otf_zone_data.rng__effective_spad_count =
			presults->per_zone_results.rng__effective_spad_count[z];
#endif
#ifdef VL53L5_AMB_DMAX_MM_ON
		pop_dev->otf_zone_data.amb_dmax_mm =
			presults->per_zone_results.amb_dmax_mm[z];
#endif
		pop_dev->otf_zone_data.rng__no_of_targets = (uint8_t)no_of_targets;
#ifdef VL53L5_ZONE_ID_ON
		pop_dev->otf_zone_data.rng__zone_id =
			presults->per_zone_results.rng__zone_id[z];
#endif

		for (t = 0; t < no_of_targets; t++) {
			i = (z * max_targets) + t;
#ifdef VL53L5_PEAK_RATE_KCPS_PER_SPAD_ON
			pop_dev->otf_target_data.peak_rate_kcps_per_spad[t] =
				presults->per_tgt_results.peak_rate_kcps_per_spad[i];
#endif
#ifdef VL53L5_RANGE_SIGMA_MM_ON
			pop_dev->otf_target_data.range_sigma_mm[t] =
				presults->per_tgt_results.range_sigma_mm[i];
#endif

			range_mm = presults->per_tgt_results.median_range_mm[i];
			if (range_clip_en > 0 && range_mm < 0)
				range_mm = 0;
			pop_dev->otf_target_data.median_range_mm[t] = range_mm;

			pop_dev->otf_target_data.target_status[t] =
				presults->per_tgt_results.target_status[i];
		}
	}
}

void  vl53l5_otf_rotate_per_zone_data(
	int32_t                            iz,
	int32_t                            oz,
	struct   vl53l5_range_results_t             *pinput,
	struct   vl53l5_range_results_t             *poutput)
{

#ifdef VL53L5_AMB_RATE_KCPS_PER_SPAD_ON
	poutput->per_zone_results.amb_rate_kcps_per_spad[oz] =
		pinput->per_zone_results.amb_rate_kcps_per_spad[iz];
#endif
#ifdef VL53L5_EFFECTIVE_SPAD_COUNT_ON
	poutput->per_zone_results.rng__effective_spad_count[oz] =
		pinput->per_zone_results.rng__effective_spad_count[iz];
#endif
#ifdef VL53L5_AMB_DMAX_MM_ON
	poutput->per_zone_results.amb_dmax_mm[oz] =
		pinput->per_zone_results.amb_dmax_mm[iz];
#endif
#ifdef VL53L5_SILICON_TEMP_DEGC_START_ON
	poutput->per_zone_results.silicon_temp_degc__start[oz] =
		pinput->per_zone_results.silicon_temp_degc__start[iz];
#endif
#ifdef VL53L5_SILICON_TEMP_DEGC_END_ON
	poutput->per_zone_results.silicon_temp_degc__end[oz] =
		pinput->per_zone_results.silicon_temp_degc__end[iz];
#endif
	poutput->per_zone_results.rng__no_of_targets[oz] =
		pinput->per_zone_results.rng__no_of_targets[iz];
#ifdef VL53L5_ZONE_ID_ON
	poutput->per_zone_results.rng__zone_id[oz] =
		pinput->per_zone_results.rng__zone_id[iz];
#endif
#ifdef VL53L5_SEQUENCE_IDX_ON
	poutput->per_zone_results.rng__sequence_idx[oz] =
		pinput->per_zone_results.rng__sequence_idx[iz];
#endif
}

void vl53l5_otf_target_filter(
	int32_t                            iz,
	int32_t                            oz,
	uint32_t                           tgt_status_op_enables,
	uint8_t                            range_clip_en,
	struct   vl53l5_range_results_t             *pinput,
	struct   vl53l5_range_results_t             *poutput,
	struct   common_grp__status_t     *perror)
{

	int32_t   i   = 0;

	int32_t   t   = 0;

	int32_t   it  = 0;

	int32_t   ot  = 0;

	int32_t   no_of_targets = 0;
	int32_t   max_targets =
		(int32_t)pinput->common_data.rng__max_targets_per_zone;
	uint16_t  wrap_dmax_mm = pinput->common_data.wrap_dmax_mm;
	uint16_t  system_dmax_mm = 0U;
	uint32_t  tgt_op_enable  = 0U;
	int16_t   range_mm = 0;

	perror->status__stage_id = VL53L5_OTF_STAGE_ID__TARGET_FILTER;
	perror->status__type = VL53L5_OTF_ERROR_NONE;

	no_of_targets =
		(int32_t)pinput->per_zone_results.rng__no_of_targets[iz];

	system_dmax_mm = pinput->per_zone_results.amb_dmax_mm[iz];
	if (wrap_dmax_mm < system_dmax_mm)
		system_dmax_mm = wrap_dmax_mm;

	for (i = 0 ; i < max_targets ; i++) {
		ot = (oz * max_targets) + i;
		if (i == 0) {
			poutput->per_tgt_results.median_range_mm[ot] =
				(int16_t)(system_dmax_mm << 2U);
		} else
			poutput->per_tgt_results.median_range_mm[ot] = 0U;
#ifdef VL53L5_PEAK_RATE_KCPS_PER_SPAD_ON
		poutput->per_tgt_results.peak_rate_kcps_per_spad[ot] = 0U;
#endif
		poutput->per_tgt_results.target_status[ot] =
			DCI_TARGET_STATUS__NOUPDATE;
	}

	t = 0;
	for (i = 0 ; i < no_of_targets ; i++) {

		it = (iz * max_targets) + i;

		tgt_op_enable =
			1U << (uint32_t)pinput->per_tgt_results.target_status[it];

		if ((tgt_op_enable & tgt_status_op_enables) == 0)
			continue;

		ot = (oz * max_targets) + t;

#ifdef VL53L5_PEAK_RATE_KCPS_PER_SPAD_ON
		poutput->per_tgt_results.peak_rate_kcps_per_spad[ot] =
			pinput->per_tgt_results.peak_rate_kcps_per_spad[it];
#endif
#ifdef VL53L5_MEDIAN_PHASE_ON
		poutput->per_tgt_results.median_phase[ot] =
			pinput->per_tgt_results.median_phase[it];
#endif
#ifdef VL53L5_RATE_SIGMA_KCPS_PER_SPAD_ON
		poutput->per_tgt_results.rate_sigma_kcps_per_spad[ot] =
			pinput->per_tgt_results.rate_sigma_kcps_per_spad[it];
#endif
#ifdef VL53L5_RANGE_SIGMA_MM_ON
		poutput->per_tgt_results.range_sigma_mm[ot] =
			pinput->per_tgt_results.range_sigma_mm[it];
#endif

		range_mm = pinput->per_tgt_results.median_range_mm[it];
		if (range_clip_en > 0 && range_mm < 0)
			range_mm = 0;
		poutput->per_tgt_results.median_range_mm[ot] = range_mm;

#ifdef VL53L5_MIN_RANGE_DELTA_MM_ON
		poutput->per_tgt_results.min_range_delta_mm[ot] =
			pinput->per_tgt_results.min_range_delta_mm[it];
#endif
#ifdef VL53L5_MAX_RANGE_DELTA_MM_ON
		poutput->per_tgt_results.max_range_delta_mm[ot] =
			pinput->per_tgt_results.max_range_delta_mm[it];
#endif
#ifdef VL53L5_TARGET_REFLECTANCE_EST_PC_ON
		poutput->per_tgt_results.target_reflectance_est_pc[ot] =
			pinput->per_tgt_results.target_reflectance_est_pc[it];
#endif
		poutput->per_tgt_results.target_status[ot] =
			pinput->per_tgt_results.target_status[it];
		t++;
	}

	poutput->per_zone_results.rng__no_of_targets[oz] = (uint8_t)t;

}
