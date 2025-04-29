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
#include "vl53l5_psz_defs.h"
#include "vl53l5_psz_luts.h"
#include "vl53l5_pseudo_single_zone_funcs.h"

void vl53l5_psz_clear_data(
	struct vl53l5_psz_int_dev_t         *pint_dev,
	struct vl53l5_psz_err_dev_t         *perr_dev)
{

	struct common_grp__status_t    *perror   =
		&(perr_dev->psz_error_status);
	struct common_grp__status_t    *pwarning =
		&(perr_dev->psz_warning_status);

	memset(
		&(pint_dev->psz_filtered_data),
		0,
		sizeof(struct vl53l5_psz__filtered__data_t));
	memset(
		&(pint_dev->psz_rng_tol_limits),
		0,
		sizeof(struct vl53l5_psz__tol__limits_t));
	memset(
		&(pint_dev->psz_tgt_meta),
		0,
		sizeof(struct vl53l5_psz__tgt__meta_t));
	memset(
		&(pint_dev->psz_tgt_data),
		0,
		sizeof(struct vl53l5_psz__tgt__data_t));

	pint_dev->psz_tgt_meta.psz__max_targets =
		VL53L5_PSZ_DEF__MAX_TARGETS;

	perror->status__group = VL53L5_ALGO_PSEUDO_SINGLE_ZONE_GROUP_ID;
	perror->status__stage_id = VL53L5_PSZ_STAGE_ID__MAIN;
	perror->status__type = VL53L5_PSZ_ERROR_NONE;
	perror->status__debug_0 = 0U;
	perror->status__debug_1 = 0U;
	perror->status__debug_2 = 0U;

	pwarning->status__group = VL53L5_ALGO_PSEUDO_SINGLE_ZONE_GROUP_ID;
	pwarning->status__stage_id = VL53L5_PSZ_STAGE_ID__MAIN;
	pwarning->status__type = VL53L5_PSZ_WARNING_NONE;
	pwarning->status__debug_0 = 0U;
	pwarning->status__debug_1 = 0U;
	pwarning->status__debug_2 = 0U;
}

void vl53l5_psz_div_and_clip_uint64(
	uint64_t                       ip_value,
	uint64_t                       ip_divisor,
	uint64_t                       upper_clip,
	uint32_t                      *presult,
	struct common_grp__status_t   *pstatus)
{

	uint64_t         op_value = ip_value;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__DIV_AND_CLIP_UINT64;
	pstatus->status__type = VL53L5_PSZ_ERROR_DIVISION_BY_ZERO;

	if (ip_divisor > 0) {
		pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

		op_value += DIV_64D64_64(ip_divisor, 2);

		op_value = DIVS_64D64_64(op_value, ip_divisor);

		if (op_value > upper_clip)
			op_value = upper_clip;
		*presult = (uint32_t)op_value;
	}
}

void vl53l5_psz_div_and_clip_uint32(
	uint32_t                       ip_value,
	uint32_t                       ip_divisor,
	uint32_t                       upper_clip,
	uint32_t                      *presult,
	struct common_grp__status_t   *pstatus)
{

	uint32_t        op_value = ip_value;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__DIV_AND_CLIP_UINT32;
	pstatus->status__type = VL53L5_PSZ_ERROR_DIVISION_BY_ZERO;

	if (ip_divisor > 0) {
		pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

		op_value += (ip_divisor / 2);

		op_value /= ip_divisor;

		if (op_value > upper_clip)
			op_value = upper_clip;
		*presult = op_value;
	}
}

void vl53l5_psz_div_and_clip_int32(
	int32_t                       ip_value,
	int32_t                       ip_divisor,
	int32_t                       lower_clip,
	int32_t                       upper_clip,
	int32_t                       *presult,
	struct common_grp__status_t   *pstatus)
{

	int32_t         op_value = ip_value;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__DIV_AND_CLIP_INT32;
	pstatus->status__type = VL53L5_PSZ_ERROR_DIVISION_BY_ZERO;

	if (ip_divisor > 0) {
		pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

		if (op_value >= 0)
			op_value += (ip_divisor / 2);
		else
			op_value -= (ip_divisor / 2);

		op_value /= ip_divisor;

		if (op_value < lower_clip)
			op_value = lower_clip;
		if (op_value > upper_clip)
			op_value = upper_clip;
		*presult = op_value;
	}
}

void vl53l5_psz_linear_interp(
	int32_t                           range_mm,
	uint32_t                          lut_no_of_entries,
	int16_t                          *plut_ranges,
	int16_t                          *plut_values,
	uint32_t                         *pop_idx,
	int32_t                          *pop_value,
	struct common_grp__status_t      *pstatus)
{

	uint32_t  idx = 0U;
	int32_t   diff0 = 0;
	int32_t   diff1 = 0;
	int32_t   value = 0;
	int32_t   denom = 0;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__LINEAR_INTERP;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	if (range_mm < (int32_t)(*plut_ranges)) {

		idx = 0U;
		*pop_idx = idx;
		*pop_value = *plut_values;
	} else if (range_mm >= (int32_t)(*(plut_ranges + lut_no_of_entries - 1U))) {

		idx = lut_no_of_entries - 1U;
		*pop_idx = idx;
		*pop_value = *(plut_values + idx);
	} else {

		idx = 0U;
		while ((*(plut_ranges + idx + 1U)) <= range_mm
			   && idx < (lut_no_of_entries - 2U))
			idx++;

		diff0 = range_mm - (int32_t)(*(plut_ranges + idx));
		diff1 = (int32_t)((*(plut_values + idx + 1)) - (*(plut_values + idx)));
		value  = diff0 *  diff1;

		denom = (int32_t)(*(plut_ranges + idx + 1)) - (*(plut_ranges + idx));

		if (denom != 0) {

			if (value < 0)
				value -= denom / 2;
			else
				value += denom / 2;

			value = value / denom;
			value += (int32_t)(*(plut_values + idx));

			if (value > (int32_t)VL53L5_INT16_MAX)
				*pop_value = (int32_t)VL53L5_INT16_MAX;
			else if (value < (int32_t)VL53L5_INT16_MIN)
				*pop_value = (int32_t)VL53L5_INT16_MIN;
			else
				*pop_value = value;
			*pop_idx = idx;
		} else {
			pstatus->status__type =
				VL53L5_PSZ_ERROR_DIVISION_BY_ZERO;
		}
	}
}

uint32_t vl53l5_psz_calc_accuracy(
	uint16_t       range_sigma_mm,
	int16_t        median_range_mm)
{

	uint32_t  accuracy = 0U;

	if (median_range_mm > 0) {

		accuracy = 1000U * (uint32_t)range_sigma_mm;
		accuracy += ((uint32_t)median_range_mm / 2U);
		accuracy /= (uint32_t)median_range_mm;

		accuracy += (1U << 4U);
		accuracy >>= 5U;
	}

	return accuracy;
}

uint32_t  vl53l5_psz_is_target_valid(
	uint32_t    target_idx,
	uint32_t    rng__no_of_targets,
	uint32_t    target_status,
	uint8_t    *ptgt_status_list)
{

	uint32_t  target_idx_valid   = 0U;
	uint32_t  target_status_valid = 0U;
	uint32_t  target_valid = 0U;
	uint8_t  *ptgt_status = ptgt_status_list;

	if (target_idx < rng__no_of_targets)
		target_idx_valid = 1U;

	while (*ptgt_status != 0U && *ptgt_status != 255U) {
		if (target_status == *ptgt_status)
			target_status_valid = 1U;
		ptgt_status++;
	}

	if (target_idx_valid > 0 && target_status_valid > 0)
		target_valid = 1U;

	return target_valid;
}

void vl53l5_psz_filter_results(
	struct   vl53l5_range_results_t          *presults,
	struct   vl53l5_psz_cfg_dev_t  *pcfg_dev,
	struct   vl53l5_psz_int_dev_t  *pint_dev,
	struct   common_grp__status_t  *pstatus)
{

	uint32_t  z = 0U;
	uint32_t  t = 0U;
	uint32_t  i = 0U;
	uint32_t  j = 0U;

	uint32_t  max_targets_per_zone =
		(uint32_t)presults->common_data.rng__max_targets_per_zone;
	uint32_t  rng__no_of_zones =
		(uint32_t)presults->common_data.rng__no_of_zones;
	uint32_t  rng_targets = 0U;
	uint32_t  psz_targets = 0U;
	uint32_t  psz__no_of_zones =
		(uint32_t)pcfg_dev->psz_general_cfg.psz__zone_list__nb_entries;

	uint32_t target_valid     = 0U;
	uint16_t max__amb_dmax_mm = 0U;
	uint32_t sum__amb_dmax_mm = 0U;
	uint32_t avg__amb_dmax_mm = 0U;
	uint32_t zone_count = 0U;

	struct  vl53l5_range_per_tgt_results_t  *ptargets =
		&(presults->per_tgt_results);
	struct  vl53l5_psz__filtered__data_t    *pfiltered =
		&(pint_dev->psz_filtered_data);
	struct  vl53l5_psz__tgt__data_t    *ptgt_data =
		&(pint_dev->psz_tgt_data);

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__FILTER_RESULTS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	for (t = 0U; t < 1; t++) {
		max__amb_dmax_mm = 0U;
		sum__amb_dmax_mm = 0U;

		for (j = 0; j < psz__no_of_zones ; j++) {
			z = pcfg_dev->psz_zone_id_list.psz__zone_id[j];

			if (z >= rng__no_of_zones)
				continue;

			zone_count++;

#ifdef VL53L5_AMB_DMAX_MM_ON
			sum__amb_dmax_mm +=
				(uint32_t)presults->per_zone_results.amb_dmax_mm[z];
			if (presults->per_zone_results.amb_dmax_mm[z] > max__amb_dmax_mm)
				max__amb_dmax_mm = presults->per_zone_results.amb_dmax_mm[z];
#endif

			rng_targets =
				(uint32_t)presults->per_zone_results.rng__no_of_targets[z];

			i = (z * max_targets_per_zone) + t;

			target_valid =
				vl53l5_psz_is_target_valid(
					t,
					rng_targets,
					(uint32_t)ptargets->target_status[i],
					&(pcfg_dev->psz_tgt_status_list_t0.psz__tgt_status[0]));

			if (target_valid > 0U && psz_targets < VL53L5_PSZ_DEF__MAX_TARGETS) {
				pfiltered->psz__filtered_sigma_mm[psz_targets] =
					ptargets->range_sigma_mm[i];
				pfiltered->psz__filtered_range_mm[psz_targets] =
					ptargets->median_range_mm[i];
				pfiltered->psz__filtered_tgts_idx[psz_targets] =
					(uint8_t)i;
				ptgt_data->psz__target_status[psz_targets] =
					ptargets->target_status[i];
				psz_targets++;
			}
		}
	}

	if (zone_count > 0) {
		vl53l5_psz_div_and_clip_uint32(
			sum__amb_dmax_mm,
			zone_count,
			VL53L5_UINT16_MAX,
			&avg__amb_dmax_mm,
			pstatus);
	}

	pint_dev->psz_tgt_meta.psz__max__amb_dmax_mm =
		max__amb_dmax_mm;
	pint_dev->psz_tgt_meta.psz__avg__amb_dmax_mm =
		(uint16_t)avg__amb_dmax_mm;
	pint_dev->psz_tgt_meta.psz__no_of_targets =
		(uint8_t)psz_targets;

}

void vl53l5_psz_calc_tol_limits(
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	int32_t                                 sigma_mm,
	int32_t                                 range_mm,
	int16_t                                *ptol_min_mm,
	int16_t                                *ptol_max_mm,
	struct   common_grp__status_t          *pstatus)
{

	int32_t  tolerance_pc = 0;
	int32_t  tolerance_mm = 0;
	int32_t  tolerance_sigma = 0;
	int32_t  tol_sigma_mm = 0;
	int32_t  tol_min_mm = 0;
	int32_t  tol_max_mm = 0;
	uint32_t idx = 0;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__CALC_TOL_LIMITS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
		vl53l5_psz_linear_interp(
			range_mm,
			(uint32_t)pcfg_dev->psz_general_cfg.psz__tol_lut__nb_entries,
			&(pcfg_dev->psz_rng_tol_lut.psz__lut__range_mm[0]),
			&(pcfg_dev->psz_rng_tol_lut.psz__lut__tolerance_pc[0]),
			&idx,
			&tolerance_pc,
			pstatus);
	}

	if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
		vl53l5_psz_linear_interp(
			range_mm,
			(uint32_t)pcfg_dev->psz_general_cfg.psz__tol_lut__nb_entries,
			&(pcfg_dev->psz_rng_tol_lut.psz__lut__range_mm[0]),
			&(pcfg_dev->psz_rng_tol_lut.psz__lut__tolerance_mm[0]),
			&idx,
			&tolerance_mm,
			pstatus);
	}

	if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
		vl53l5_psz_linear_interp(
			range_mm,
			(uint32_t)pcfg_dev->psz_general_cfg.psz__tol_lut__nb_entries,
			&(pcfg_dev->psz_rng_tol_lut.psz__lut__range_mm[0]),
			&(pcfg_dev->psz_rng_tol_lut.psz__lut__tolerance_sigma[0]),
			&idx,
			&tolerance_sigma,
			pstatus);
	}

	if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
		tol_sigma_mm = tolerance_sigma * sigma_mm;
		tol_sigma_mm += (1 << 14);
		tol_sigma_mm /= (1 << 15);

		if (range_mm < 0) {
			tol_min_mm = ((tolerance_pc * range_mm) - 50) / 100;
			tol_min_mm = -tol_min_mm;
		} else
			tol_min_mm = ((tolerance_pc * range_mm) + 50) / 100;

		tol_max_mm = tol_min_mm;

		tol_min_mm =
			range_mm - tol_min_mm - tolerance_mm - tol_sigma_mm;

		if (tol_min_mm > (int32_t)VL53L5_INT16_MAX)
			tol_min_mm = (int32_t)VL53L5_INT16_MAX;
		else if (tol_min_mm < (int32_t)VL53L5_INT16_MIN)
			tol_min_mm = (int32_t)VL53L5_INT16_MIN;

		tol_max_mm =
			range_mm + tol_max_mm + tolerance_mm + tol_sigma_mm;

		if (tol_max_mm > (int32_t)VL53L5_INT16_MAX)
			tol_max_mm = (int32_t)VL53L5_INT16_MAX;
		else if (tol_max_mm < (int32_t)VL53L5_INT16_MIN)
			tol_max_mm = (int32_t)VL53L5_INT16_MIN;
	}

	*ptol_min_mm = (int16_t)tol_min_mm;
	*ptol_max_mm = (int16_t)tol_max_mm;
}

void vl53l5_psz_update_range_tol_limits(
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	int32_t                                 no_of_ranges,
	uint16_t                               *psigmas,
	int16_t                                *pranges,
	struct   vl53l5_psz__tol__limits_t     *plimits,
	struct   common_grp__status_t          *pstatus)
{

	int32_t  i = 0;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__UPDATE_RANGE_TOL_LIMITS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	for (i = 0; i < no_of_ranges
		 && pstatus->status__type == VL53L5_PSZ_ERROR_NONE; i++) {

		vl53l5_psz_calc_tol_limits(
			pcfg_dev,
			(int32_t)(*(psigmas + i)),
			(int32_t)(*(pranges + i)),
			&(plimits->psz__tol__min_mm[i]),
			&(plimits->psz__tol__max_mm[i]),
			pstatus);
	}
}

void vl53l5_psz_accumulate_target_stats(
	uint32_t                                idx,
	struct   vl53l5_range_results_t                  *presults,
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus)
{

	uint32_t  t = 0U;
	uint32_t  z = 0U;
	uint32_t  i = 0U;

	uint32_t  max_targets_per_zone =
		(uint32_t)presults->common_data.rng__max_targets_per_zone;
	uint32_t  no_of_zones =
		(uint32_t)presults->common_data.rng__no_of_zones;
	uint32_t  no_of_targets = 0U;
	uint32_t  no_of_samples = 0U;

	struct  vl53l5_psz__sum__data_t  *psums =
		&(pint_dev->psz_sum_data);

	struct  vl53l5_range_per_zone_results_t  *pzones =
		&(presults->per_zone_results);
	struct  vl53l5_range_per_tgt_results_t   *ptargets =
		&(presults->per_tgt_results);

	int16_t  min_limit_mm =
		pint_dev->psz_rng_tol_limits.psz__tol__min_mm[idx];
	int16_t  max_limit_mm =
		pint_dev->psz_rng_tol_limits.psz__tol__max_mm[idx];
	int16_t  range_mm = 0;

	uint32_t target_valid   = 0U;
	uint32_t range_group    = 0U;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__ACCUMULATE_TARGET_STATS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	memset(psums, 0, sizeof(struct vl53l5_psz__sum__data_t));

	for (z = 0; z < no_of_zones ; z++) {
		no_of_targets =
			(uint32_t)presults->per_zone_results.rng__no_of_targets[z];

		for (t = 0U; t < no_of_targets &&  z < no_of_zones; t++) {
			i = z * max_targets_per_zone + t;

			target_valid =
				vl53l5_psz_is_target_valid(
					t,
					no_of_targets,
					(uint32_t)ptargets->target_status[i],
					&(pcfg_dev->psz_tgt_status_list_t1.psz__tgt_status[0]));

			range_mm = ptargets->median_range_mm[i];
			range_group = 1U;
			if (range_mm < min_limit_mm || range_mm > max_limit_mm)
				range_group = 0U;

			if (target_valid > 0U && range_group > 0U) {

#ifdef VL53L5_AMB_RATE_KCPS_PER_SPAD_ON
				psums->psz__sum__amb_rate_kcps_per_spad +=
					(uint64_t)pzones->amb_rate_kcps_per_spad[z];
#endif
#ifdef VL53L5_EFFECTIVE_SPAD_COUNT_ON
				psums->psz__sum__effective_spad_count +=
					(uint64_t)pzones->rng__effective_spad_count[z];
#endif
#ifdef VL53L5_AMB_DMAX_MM_ON
				psums->psz__sum__amb_dmax_mm +=
					(uint32_t)pzones->amb_dmax_mm[z];
#endif

#ifdef VL53L5_PEAK_RATE_KCPS_PER_SPAD_ON
				psums->psz__sum__peak_rate_kcps_per_spad +=
					(uint64_t)ptargets->peak_rate_kcps_per_spad[i];
#endif
#ifdef VL53L5_RANGE_SIGMA_MM_ON
				psums->psz__sum__range_sigma_mm +=
					((uint32_t)ptargets->range_sigma_mm[i] *
					 (uint32_t)ptargets->range_sigma_mm[i]);
#endif
				psums->psz__sum__median_range_mm +=
					(int32_t)range_mm;

				no_of_samples++;

			}
		}
	}

	pint_dev->psz_tgt_data.psz__no_of_matches[idx] =
		(uint8_t)no_of_samples;

}

void vl53l5_psz_compute_target_stats(
	uint32_t                                idx,
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus)
{

	struct  vl53l5_psz__sum__data_t  *psums =
		&(pint_dev->psz_sum_data);
	struct  vl53l5_psz__tgt__data_t   *pdata =
		&(pint_dev->psz_tgt_data);
	uint32_t no_of_matches =
		(uint32_t)pdata->psz__no_of_matches[idx];

	uint32_t tmp32  = 0U;
	int32_t  tmp32s = 0;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__COMPUTE_TARGET_STATS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	pdata->psz__amb_rate_kcps_per_spad[idx] = 0U;
	pdata->psz__effective_spad_count[idx] = 0U;
	pdata->psz__peak_rate_kcps_per_spad[idx] = 0U;
	pdata->psz__amb_dmax_mm[idx] = 0U;
	pdata->psz__range_sigma_mm[idx] = 0U;
	pdata->psz__median_range_mm[idx] = 0U;

	if (no_of_matches > 0) {

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_div_and_clip_uint64(
				psums->psz__sum__amb_rate_kcps_per_spad,
				(uint64_t)no_of_matches,
				(uint64_t)VL53L5_UINT32_MAX,
				&(pdata->psz__amb_rate_kcps_per_spad[idx]),
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_div_and_clip_uint64(
				psums->psz__sum__effective_spad_count,
				(uint64_t)no_of_matches,
				(uint64_t)VL53L5_UINT32_MAX,
				&(pdata->psz__effective_spad_count[idx]),
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_div_and_clip_uint64(
				psums->psz__sum__peak_rate_kcps_per_spad,
				(uint64_t)no_of_matches,
				(uint64_t)VL53L5_UINT32_MAX,
				&(pdata->psz__peak_rate_kcps_per_spad[idx]),
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_div_and_clip_uint32(
				psums->psz__sum__amb_dmax_mm,
				(uint32_t)no_of_matches,
				(uint32_t)VL53L5_UINT16_MAX,
				&tmp32,
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE)
			pdata->psz__amb_dmax_mm[idx] = (uint16_t)tmp32;

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_div_and_clip_int32(
				psums->psz__sum__median_range_mm,
				(int32_t)no_of_matches,
				(int32_t)VL53L5_INT16_MIN,
				(int32_t)VL53L5_INT16_MAX,
				&tmp32s,
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE)
			pdata->psz__median_range_mm[idx] = (int16_t)tmp32s;

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_div_and_clip_uint32(
				psums->psz__sum__range_sigma_mm,
				(uint32_t)(no_of_matches * no_of_matches),
				(uint32_t)VL53L5_UINT32_MAX,
				&tmp32,
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			pdata->psz__range_sigma_mm[idx] =
				(uint16_t)INTEGER_SQRT(tmp32);
		}
	}
}

void vl53l5_psz_find_similar_targets(
	struct   vl53l5_range_results_t                  *presults,
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus)
{

	uint32_t  i = 0U;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__FIND_SIMILAR_TARGETS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	for (i = 0; i < (uint32_t)pint_dev->psz_tgt_meta.psz__no_of_targets; i++) {

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_accumulate_target_stats(
				i,
				presults,
				pcfg_dev,
				pint_dev,
				pstatus);
		}

		if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {
			vl53l5_psz_compute_target_stats(
				i,
				pint_dev,
				pstatus);
		}
	}
}

void vl53l5_psz_swap_uint32(
	uint32_t  *p0,
	uint32_t  *p1)
{

	uint32_t  tmp  = *p0;
	*p0 = *p1;
	*p1 = tmp;
}

void vl53l5_psz_swap_uint16(
	uint16_t  *p0,
	uint16_t  *p1)
{

	uint16_t  tmp  = *p0;
	*p0 = *p1;
	*p1 = tmp;
}

void vl53l5_psz_swap_int16(
	int16_t  *p0,
	int16_t  *p1)
{

	int16_t  tmp  = *p0;
	*p0 = *p1;
	*p1 = tmp;
}

void vl53l5_psz_swap_uint8(
	uint8_t  *p0,
	uint8_t  *p1)
{

	uint8_t  tmp  = *p0;
	*p0 = *p1;
	*p1 = tmp;
}

void vl53l5_psz_sort(
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus)
{

	uint32_t  i0        = 0;
	uint32_t  i1        = 0;
	uint32_t  swapped   = 1U;

	uint8_t   v0      = 0;
	uint8_t   v1      = 0;

	uint32_t  no_of_targets =
		(uint32_t)pint_dev->psz_tgt_meta.psz__no_of_targets;

	struct  vl53l5_psz__tgt__data_t *ptgt_data =
		&(pint_dev->psz_tgt_data);

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__SORT;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	while (swapped > 0U &&
		   pint_dev->psz_tgt_meta.psz__no_of_targets > 0U) {
		swapped = 0U;

		for (i1 = 1U; i1 < no_of_targets; i1++) {
			i0 = i1 - 1;
			v0 = ptgt_data->psz__no_of_matches[i0];
			v1 = ptgt_data->psz__no_of_matches[i1];

			if (v0 < v1) {

				vl53l5_psz_swap_uint32(
					&(ptgt_data->psz__amb_rate_kcps_per_spad[i0]),
					&(ptgt_data->psz__amb_rate_kcps_per_spad[i1]));

				vl53l5_psz_swap_uint32(
					&(ptgt_data->psz__peak_rate_kcps_per_spad[i0]),
					&(ptgt_data->psz__peak_rate_kcps_per_spad[i1]));

				vl53l5_psz_swap_uint32(
					&(ptgt_data->psz__effective_spad_count[i0]),
					&(ptgt_data->psz__effective_spad_count[i1]));

				vl53l5_psz_swap_uint16(
					&(ptgt_data->psz__amb_dmax_mm[i0]),
					&(ptgt_data->psz__amb_dmax_mm[i1]));

				vl53l5_psz_swap_uint16(
					&(ptgt_data->psz__range_sigma_mm[i0]),
					&(ptgt_data->psz__range_sigma_mm[i1]));

				vl53l5_psz_swap_int16(
					&(ptgt_data->psz__median_range_mm[i0]),
					&(ptgt_data->psz__median_range_mm[i1]));

				vl53l5_psz_swap_uint8(
					&(ptgt_data->psz__target_status[i0]),
					&(ptgt_data->psz__target_status[i1]));

				ptgt_data->psz__no_of_matches[i0] = v1;
				ptgt_data->psz__no_of_matches[i1] = v0;

				swapped = 1;

			}
		}
	}
}

void vl53l5_psz_remove_duplicate_targets(
	struct   vl53l5_psz_int_dev_t         *pint_dev,
	struct   common_grp__status_t         *pstatus)
{

	int32_t  y = 0;
	int32_t  z = 0;
	int32_t  no_of_targets =
		(int32_t)pint_dev->psz_tgt_meta.psz__no_of_targets;

	struct   vl53l5_psz__tol__limits_t *plimits =
		&(pint_dev->psz_rng_tol_limits);
	struct   vl53l5_psz__tgt__data_t *ptgt_data =
		&(pint_dev->psz_tgt_data);

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__REMOVE_DUPLICATE_TARGETS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	for (y = 0; y < (no_of_targets - 1); y++) {
		for (z = y + 1; z < no_of_targets; z++) {
			if (ptgt_data->psz__median_range_mm[y] > plimits->psz__tol__min_mm[z] &&
				ptgt_data->psz__median_range_mm[y] < plimits->psz__tol__max_mm[z]) {
				ptgt_data->psz__median_range_mm[z] =
					VL53L5_PSZ_DEF__INVALID_RANGE_MM;
				ptgt_data->psz__no_of_matches[z] = 0;
			}
		}
	}
}

void vl53l5_psz_clear_range_data(
	uint16_t                               amb_dmax_mm,
	struct   vl53l5_psz__range__data_t    *prng_data)
{

	memset(
		prng_data,
		0,
		sizeof(struct vl53l5_psz__range__data_t));

	prng_data->psz__amb_dmax_mm = amb_dmax_mm;
	prng_data->psz__median_range_mm =
		VL53L5_PSZ_DEF__INVALID_RANGE_MM;
}

void vl53l5_psz_set_range_data(
	int32_t                                  idx,
	struct   vl53l5_psz__tgt__data_t        *ptgt_data,
	struct   vl53l5_psz__range__data_t      *prng_data)
{

	prng_data->psz__amb_rate_kcps_per_spad =
		ptgt_data->psz__amb_rate_kcps_per_spad[idx];
	prng_data->psz__peak_rate_kcps_per_spad =
		ptgt_data->psz__peak_rate_kcps_per_spad[idx];
	prng_data->psz__effective_spad_count =
		ptgt_data->psz__effective_spad_count[idx];
	prng_data->psz__amb_dmax_mm =
		ptgt_data->psz__amb_dmax_mm[idx];
	prng_data->psz__range_sigma_mm =
		ptgt_data->psz__range_sigma_mm[idx];
	prng_data->psz__median_range_mm =
		ptgt_data->psz__median_range_mm[idx];
	prng_data->psz__target_status =
		ptgt_data->psz__target_status[idx];
	prng_data->psz__no_of_matches =
		ptgt_data->psz__no_of_matches[idx];
}

void vl53l5_psz_temporal_filter(
	int32_t                                depth_temporal,
	struct   vl53l5_psz_int_dev_t         *pint_dev,
	struct   vl53l5_psz_op_dev_t          *pop_dev,
	struct   common_grp__status_t         *pstatus)
{

	int32_t  i   = 0;
	int32_t  x   = 0;
	int32_t  idx = 0;
	int32_t  depth_count = 0;
	int32_t  target_match = 0;

	struct   vl53l5_psz__tol__limits_t    *plimits =
		&(pint_dev->psz_rng_tol_limits);
	struct   vl53l5_psz__tgt__meta_t      *ptgt_meta =
		&(pint_dev->psz_tgt_meta);
	struct   vl53l5_psz__tgt__data_t      *ptgt_data =
		&(pint_dev->psz_tgt_data);

	struct   vl53l5_psz__temporal__data_t *ptem_data =
		&(pop_dev->psz_temporal_data);

	struct   vl53l5_psz__range__data_t    *prng_new =
		&(pop_dev->psz_rng_new);
	struct   vl53l5_psz__range__data_t    *prng_last =
		&(pop_dev->psz_rng_last);
	struct   vl53l5_psz__range__data_t    *prng_output =
		&(pop_dev->psz_rng_output);

	int32_t  no_of_targets =
		(int32_t)ptgt_meta->psz__no_of_targets;

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__TEMPORAL_FILTER;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	target_match = 0;
	for (i = 0; i < no_of_targets && target_match == 0; i++) {
		if ((ptgt_data->psz__no_of_matches[i] > 0) &&
			(prng_last->psz__median_range_mm < plimits->psz__tol__max_mm[i]) &&
			(prng_last->psz__median_range_mm > plimits->psz__tol__min_mm[i])) {
			target_match = 1;
			vl53l5_psz_clear_range_data(
				ptgt_meta->psz__avg__amb_dmax_mm,
				prng_new);

			depth_count = 0U;
			for (x = 0; x < depth_temporal; x++) {
				if ((ptem_data->psz__range_mm[x] < plimits->psz__tol__max_mm[0]) &&
					(ptem_data->psz__range_mm[x] > plimits->psz__tol__min_mm[0]))
					depth_count++;
			}

			idx = i;
			if (depth_count == depth_temporal || i == 0) {

				idx = 0;
			}

			vl53l5_psz_set_range_data(
				idx,
				ptgt_data,
				prng_output);
		}
	}

	if (target_match == 0 && prng_new->psz__no_of_matches > 0) {
		if ((prng_new->psz__median_range_mm > plimits->psz__tol__min_mm[0]) &&
			(prng_new->psz__median_range_mm < plimits->psz__tol__max_mm[0])) {

			vl53l5_psz_set_range_data(
				0,
				ptgt_data,
				prng_output);

			vl53l5_psz_clear_range_data(
				ptgt_meta->psz__avg__amb_dmax_mm,
				prng_new);
		} else if (no_of_targets == 0) {
			vl53l5_psz_clear_range_data(
				ptgt_meta->psz__avg__amb_dmax_mm,
				prng_new);
			vl53l5_psz_clear_range_data(
				ptgt_meta->psz__avg__amb_dmax_mm,
				prng_output);
		} else {
			vl53l5_psz_set_range_data(
				0,
				ptgt_data,
				prng_new);

			memcpy(
				prng_output,
				prng_last,
				sizeof(struct vl53l5_psz__range__data_t));
		}
	} else if (target_match == 0) {

		if (no_of_targets == 0) {
			vl53l5_psz_clear_range_data(
				ptgt_meta->psz__avg__amb_dmax_mm,
				prng_new);
			prng_new->psz__no_of_matches = 1;
		} else {
			vl53l5_psz_set_range_data(
				0,
				ptgt_data,
				prng_new);
		}

		memcpy(
			prng_output,
			prng_last,
			sizeof(struct vl53l5_psz__range__data_t));
	}

	memcpy(
		prng_last,
		prng_output,
		sizeof(struct vl53l5_psz__range__data_t));

	for (x = (depth_temporal - 2); x > -1; x--) {
		ptem_data->psz__range_mm[x + 1] =
			ptem_data->psz__range_mm[x];
		ptem_data->psz__no_of_matches[x + 1] =
			ptem_data->psz__no_of_matches[x];
	}

	if (no_of_targets > 0) {
		ptem_data->psz__range_mm[0] =
			ptgt_data->psz__median_range_mm[0];
		ptem_data->psz__no_of_matches[0] =
			ptgt_data->psz__no_of_matches[0];
	} else {
		ptem_data->psz__range_mm[0] =
			VL53L5_PSZ_DEF__INVALID_RANGE_MM;
		ptem_data->psz__no_of_matches[0] = 0U;
	}
}

void vl53l5_psz_update_range_results(
	struct   vl53l5_psz__general__cfg_t   *pgen_cfg,
	struct   vl53l5_psz__range__data_t    *prng_output,
	struct   vl53l5_range_results_t                 *presults,
	struct   common_grp__status_t         *pstatus)
{

	uint32_t  max_targets_per_zone =
		(uint32_t)presults->common_data.rng__max_targets_per_zone;
	uint32_t  no_of_zones =
		(uint32_t)presults->common_data.rng__no_of_zones;
	uint32_t  idx = no_of_zones * max_targets_per_zone;

	uint16_t  wrap_dmax_mm = presults->common_data.wrap_dmax_mm;
	uint16_t  system_dmax_mm = 0U;

	struct  vl53l5_range_per_zone_results_t *pzones =
		&(presults->per_zone_results);
	struct  vl53l5_range_per_tgt_results_t *ptargets =
		&(presults->per_tgt_results);

	pstatus->status__stage_id =
		VL53L5_PSZ_STAGE_ID__UPDATE_RANGE_RESULTS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	if (no_of_zones >= VL53L5_MAX_ZONES)
		pstatus->status__type = VL53L5_PSZ_ERROR_MAX_ZONES_TOO_SMALL;
	if (idx >= VL53L5_MAX_TARGETS)
		pstatus->status__type = VL53L5_PSZ_ERROR_MAX_TARGETS_TOO_SMALL;

	system_dmax_mm = prng_output->psz__amb_dmax_mm;
	if (wrap_dmax_mm < system_dmax_mm)
		system_dmax_mm = wrap_dmax_mm;

	if (pstatus->status__type == VL53L5_PSZ_ERROR_NONE) {

#ifdef VL53L5_ZONE_ID_ON
		pzones->rng__zone_id[no_of_zones] =
			pgen_cfg->psz__zone__idx;
#endif

		if (prng_output->psz__no_of_matches > 0) {
#ifdef VL53L5_AMB_RATE_KCPS_PER_SPAD_ON
			pzones->amb_rate_kcps_per_spad[no_of_zones] =
				prng_output->psz__amb_rate_kcps_per_spad;
#endif
#ifdef VL53L5_EFFECTIVE_SPAD_COUNT_ON
			pzones->rng__effective_spad_count[no_of_zones] =
				prng_output->psz__effective_spad_count;
#endif
#ifdef VL53L5_AMB_DMAX_MM_ON
			pzones->amb_dmax_mm[no_of_zones] =
				prng_output->psz__amb_dmax_mm;
#endif

			pzones->rng__no_of_targets[no_of_zones] = 1U;

#ifdef VL53L5_PEAK_RATE_KCPS_PER_SPAD_ON
			ptargets->peak_rate_kcps_per_spad[idx] =
				prng_output->psz__peak_rate_kcps_per_spad;
#endif
#ifdef VL53L5_RANGE_SIGMA_MM_ON
			ptargets->range_sigma_mm[idx] =
				prng_output->psz__range_sigma_mm;
#endif

			ptargets->median_range_mm[idx] =
				prng_output->psz__median_range_mm;
			ptargets->target_status[idx] =
				prng_output->psz__target_status;
		} else {
#ifdef VL53L5_AMB_RATE_KCPS_PER_SPAD_ON
			pzones->amb_rate_kcps_per_spad[no_of_zones] = 0U;
#endif
#ifdef VL53L5_EFFECTIVE_SPAD_COUNT_ON
			pzones->rng__effective_spad_count[no_of_zones] = 0U;
#endif
#ifdef VL53L5_AMB_DMAX_MM_ON
			pzones->amb_dmax_mm[no_of_zones] =
				prng_output->psz__amb_dmax_mm;
#endif

			pzones->rng__no_of_targets[no_of_zones] = 0U;

#ifdef VL53L5_PEAK_RATE_KCPS_PER_SPAD_ON
			ptargets->peak_rate_kcps_per_spad[idx] = 0U;
#endif
#ifdef VL53L5_RANGE_SIGMA_MM_ON
			ptargets->range_sigma_mm[idx] = 0U;
#endif

			ptargets->median_range_mm[idx] =
				(int16_t)(system_dmax_mm << 2U);
			ptargets->target_status[idx] = 0U;
		}
	}
}
