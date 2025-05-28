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
#include "dci_defs.h"
#include "dci_luts.h"
#include "vl53l5_ltf_defs.h"
#include "vl53l5_ltf_luts.h"
#include "vl53l5_long_tail_filter_funcs.h"

void vl53l5_ltf_clear_data(
	struct vl53l5_ltf_err_dev_t         *perr_dev)
{

	struct common_grp__status_t    *perror   = &(perr_dev->ltf_error_status);
	struct common_grp__status_t    *pwarning = &(perr_dev->ltf_warning_status);

	perror->status__group = VL53L5_ALGO_LONG_TAIL_FILTER_GROUP_ID;
	perror->status__stage_id = VL53L5_LTF_STAGE_ID__MAIN;
	perror->status__type = VL53L5_LTF_ERROR_NONE;
	perror->status__debug_0 = 0U;
	perror->status__debug_1 = 0U;
	perror->status__debug_2 = 0U;

	pwarning->status__group = VL53L5_ALGO_LONG_TAIL_FILTER_GROUP_ID;
	pwarning->status__stage_id = VL53L5_LTF_STAGE_ID__MAIN;
	pwarning->status__type = VL53L5_LTF_WARNING_NONE;
	pwarning->status__debug_0 = 0U;
	pwarning->status__debug_1 = 0U;
	pwarning->status__debug_2 = 0U;
}

void vl53l5_ltf_linear_interp(
	int32_t                           range_mm,
	uint32_t                          lut_no_of_entries,
	int32_t                          *plut_ranges,
	int32_t                          *plut_values,
	uint32_t                         *pop_idx,
	int32_t                          *pop_value,
	struct common_grp__status_t      *pstatus)
{

	uint32_t  idx = 0U;
	int32_t   diff0 = 0;
	int32_t   diff1 = 0;
	int64_t   value = 0;
	int64_t   denom = 0;

	pstatus->status__stage_id = VL53L5_LTF_STAGE_ID__LINEAR_INTERP;
	pstatus->status__type = VL53L5_LTF_ERROR_NONE;

	if (range_mm < *plut_ranges) {

		idx = 0U;
		*pop_idx = idx;
		*pop_value = *plut_values;
	} else if (range_mm >= *(plut_ranges + lut_no_of_entries - 1U)) {

		idx = lut_no_of_entries - 1U;
		*pop_idx = idx;
		*pop_value = *(plut_values + idx);
	} else {

		idx = 0U;
		while ((*(plut_ranges + idx + 1U)) <= range_mm
			   && idx < (lut_no_of_entries - 2U))
			idx++;

		diff0 = range_mm - (*(plut_ranges + idx));
		diff1 = (*(plut_values + idx + 1)) - (*(plut_values + idx));
		value  = MULS_32X32_64(diff0, diff1);

		denom = (int64_t)(*(plut_ranges + idx + 1)) - (*(plut_ranges + idx));

		if (denom != 0) {

			if (value < 0)
				value -= DIVS_64D64_64(denom, 2);
			else
				value += DIVS_64D64_64(denom, 2);

			value = DIVS_64D64_64(value, denom);
			value += (*(plut_values + idx));

			if (value > VL53L5_INT32_MAX)
				*pop_value = VL53L5_INT32_MAX;
			else if (value < VL53L5_INT32_MIN)
				*pop_value = VL53L5_INT32_MIN;
			else
				*pop_value = (int32_t)value;
			*pop_idx = idx;
		} else {
			pstatus->status__type =
				VL53L5_LTF_ERROR_DIVISION_BY_ZERO;
		}
	}
}

int32_t vl53l5_ltf_is_target_valid(
	uint8_t     target_status,
	uint32_t    tgt_status__nb_entries,
	uint8_t    *ptgt_status_list)
{

	int32_t  tgt_valid = 0;
	uint32_t s = 0U;

	for (s = 0 ; s < tgt_status__nb_entries ; s++) {
		if (target_status == *(ptgt_status_list + s))
			tgt_valid = 1;
	}
	return tgt_valid;
}

void vl53l5_ltf_calc_rate_ratio(
	uint32_t                          rate0,
	uint32_t                          rate1,
	int32_t                          *pratio,
	struct common_grp__status_t      *pstatus)
{

	uint64_t  ratio = 0U;

	pstatus->status__stage_id = VL53L5_LTF_STAGE_ID__CALC_RATE_RATIO;
	pstatus->status__type = VL53L5_LTF_ERROR_DIVISION_BY_ZERO;

	if (rate1 > 0U) {
		pstatus->status__type =
			VL53L5_LTF_ERROR_NONE;

		ratio  = (uint64_t)rate0 << (uint64_t)VL53L5_LTF_DEF__RATIO_FRAC_BITS;
		ratio += (uint64_t)(rate1 >> 1U);
		ratio  = DIV_64D64_64(ratio, (uint64_t)rate1);

		if (ratio > (uint32_t)VL53L5_LTF_DEF__RATIO_MAX_VALUE)
			ratio = VL53L5_LTF_DEF__RATIO_MAX_VALUE;

	}

	*pratio = (int32_t)ratio;
}

void vl53l5_ltf_calc_threshold(
	int32_t                               primary_range_mm,
	int32_t                               diff_range_mm,
	struct vl53l5_ltf__general__cfg_t    *pcfg,
	struct vl53l5_ltf__thres_lut__cfg_t  *plut,
	int32_t                              *pthresh,
	struct common_grp__status_t          *pstatus)
{

	uint32_t   idx = 0;
	int32_t    slope = 0;
	int32_t    y_intercept = 0;
	int64_t    thresh = 0;

	pstatus->status__stage_id = VL53L5_LTF_STAGE_ID__CALC_THRESHOLD;
	pstatus->status__type = VL53L5_LTF_ERROR_NONE;

	vl53l5_ltf_linear_interp(
		primary_range_mm,
		(uint32_t)pcfg->ltf__thresh_lut__nb_entries,
		&(plut->ltf__lut__range_mm[0]),
		&(plut->ltf__lut__slope[0]),
		&idx,
		&slope,
		pstatus);

	if (pstatus->status__type == VL53L5_LTF_ERROR_NONE) {
		vl53l5_ltf_linear_interp(
			primary_range_mm,
			(uint32_t)pcfg->ltf__thresh_lut__nb_entries,
			&(plut->ltf__lut__range_mm[0]),
			&(plut->ltf__lut__y_intercept[0]),
			&idx,
			&y_intercept,
			pstatus);
	}

	if (pstatus->status__type == VL53L5_LTF_ERROR_NONE) {

		thresh  = MULS_32X32_64(diff_range_mm, slope);
		thresh += 512;
		thresh  = DIVS_64D64_64(thresh, 1024);
		thresh += y_intercept;

		if (thresh > VL53L5_INT32_MAX)
			thresh = VL53L5_INT32_MAX;
		if (thresh < VL53L5_INT32_MIN)
			thresh = VL53L5_INT32_MIN;

		*pthresh = (int32_t)thresh;
	}
}

void vl53l5_ltf_zone_lut_range(
	int32_t                       zone,
	int32_t                       max_targets,
	int32_t                       no_of_targets,
	struct vl53l5_ltf_cfg_dev_t  *pcfg_dev,
	struct vl53l5_range_results_t          *presults,
	struct common_grp__status_t  *pstatus)
{

	int32_t   t = 0;
	int32_t   tgt_valid = 0;

	int32_t   idx = 0;
	int32_t   primary_idx = 0;

	uint32_t  rate_kcps_per_spad = 0U;
	uint32_t  primary_rate_kcps_per_spad = 0U;

	int32_t   primary_range_mm = 0;
	int32_t   diff_range_mm = 0;

	int32_t   valid_tgt_idxs[DCI_MAX_TARGET_PER_ZONE] = {0};
	int32_t   valid_tgt_count = 0;

	int32_t   rate_ratio = 0;
	int32_t   rate_thresh = 0;

	pstatus->status__stage_id = VL53L5_LTF_STAGE_ID__ZONE_LUT_RANGE;
	pstatus->status__type = VL53L5_LTF_ERROR_NONE;

	for (t = 0; t < no_of_targets && t < max_targets; t++) {

		idx = (zone * max_targets) + t;

		tgt_valid =
			vl53l5_ltf_is_target_valid(
				presults->per_tgt_results.target_status[idx],
				pcfg_dev->ltf_general_cfg.ltf__tgt_status__nb_entries,
				&(pcfg_dev->ltf_tgt_status_cfg.ltf__tgt_status_list[0]));

		rate_kcps_per_spad =
			presults->per_tgt_results.peak_rate_kcps_per_spad[idx];

		if (tgt_valid > 0) {
			if (rate_kcps_per_spad > primary_rate_kcps_per_spad) {
				primary_rate_kcps_per_spad = rate_kcps_per_spad;
				primary_range_mm =
					(int32_t)presults->per_tgt_results.median_range_mm[idx];
				primary_idx = idx;
			}
			valid_tgt_idxs[valid_tgt_count] = idx;
			valid_tgt_count++;
		}
	}

	for (t = 0; t < valid_tgt_count && valid_tgt_count > 1; t++) {
		idx = valid_tgt_idxs[t];

		diff_range_mm =
			(int32_t)presults->per_tgt_results.median_range_mm[idx];
		diff_range_mm -= primary_range_mm;

		if (diff_range_mm > 0 && idx != primary_idx) {
			diff_range_mm -=
				pcfg_dev->ltf_general_cfg.ltf__range_offset_mm;

			if (diff_range_mm > 0) {

				vl53l5_ltf_calc_threshold(
					primary_range_mm,
					diff_range_mm,
					&(pcfg_dev->ltf_general_cfg),
					&(pcfg_dev->ltf_thresh_lut_cfg),
					&rate_thresh,
					pstatus);

				vl53l5_ltf_calc_rate_ratio(
					primary_rate_kcps_per_spad,
					presults->per_tgt_results.peak_rate_kcps_per_spad[idx],
					&rate_ratio,
					pstatus);

				if (rate_ratio > rate_thresh) {

					presults->per_tgt_results.target_status[idx] =
						LTF_TARGET_STATUS__TARGETDUETOLONGTAIL;
				}

			} else {

				presults->per_tgt_results.target_status[idx] =
					LTF_TARGET_STATUS__TARGETSEPARATIONTOOSMALL;
			}
		}
	}
}
