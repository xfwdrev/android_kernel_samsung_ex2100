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

#include "dci_luts.h"
#include "vl53l5_psz_version_defs.h"
#include "vl53l5_psz_defs.h"
#include "vl53l5_psz_luts.h"
#include "vl53l5_pseudo_single_zone_funcs.h"
#include "vl53l5_pseudo_single_zone_main.h"
#include "vl53l5_platform_log.h"
#include "vl53l5_k_logging.h"
#ifdef STM_PSZ_DEDIAN_RANGE_MM_STABILITY
#define SAMPLE_MAX 20
#define PSZ_MEDIAN_RANGE_THRESHOLD 102 /* 102-20% 51-10 % */

static int psz_median_range_mm_samplecnt;
static int psz_median_range_mm_samplecnt;
static int sum;
static int16_t psz_median_range_mm_baseline;
static uint16_t current_psz_median_range_mm;
static bool first_baseline;
#endif

#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
void vl53l5_psz_get_version(
	struct  common_grp__version_t  *pversion)
{

	pversion->version__major = VL53L5_PSZ_DEF__VERSION__MAJOR;
	pversion->version__minor = VL53L5_PSZ_DEF__VERSION__MINOR;
	pversion->version__build = VL53L5_PSZ_DEF__VERSION__BUILD;
	pversion->version__revision = VL53L5_PSZ_DEF__VERSION__REVISION;
}
#endif

void vl53l5_pseudo_single_zone_init_start_rng(
	struct vl53l5_psz_int_dev_t   *pint_dev,
	struct vl53l5_psz_op_dev_t    *pop_dev,
	struct vl53l5_psz_err_dev_t   *perr_dev)
{

	struct   common_grp__status_t *pstatus = &(perr_dev->psz_error_status);

	vl53l5_psz_clear_data(
		pint_dev,
		perr_dev);

	pstatus->status__stage_id = VL53L5_PSZ_STAGE_ID__COMPUTE_TARGET_STATS;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;

	memset(pop_dev, 0, sizeof(struct vl53l5_psz_op_dev_t));

	vl53l5_psz_clear_range_data(
		0U,
		&(pop_dev->psz_rng_new));
	vl53l5_psz_clear_range_data(
		0U,
		&(pop_dev->psz_rng_last));
	vl53l5_psz_clear_range_data(
		0U,
		&(pop_dev->psz_rng_output));
#ifdef STM_PSZ_DEDIAN_RANGE_MM_STABILITY

	psz_median_range_mm_samplecnt = 0;

	psz_median_range_mm_baseline = 0;

	psz_median_range_mm_samplecnt = 0;

	sum = 0;

	current_psz_median_range_mm = 0;

	first_baseline = false;
#endif
}

void vl53l5_pseudo_single_zone_main(
	struct vl53l5_range_results_t           *presults,
	struct vl53l5_psz_cfg_dev_t   *pcfg_dev,
	struct vl53l5_psz_int_dev_t   *pint_dev,
	struct vl53l5_psz_op_dev_t    *pop_dev,
	struct vl53l5_psz_err_dev_t   *perr_dev)
{
#ifdef STM_PSZ_DEDIAN_RANGE_MM_STABILITY
	int16_t   psz__median_range_mm;
	uint16_t  psz__median_range_mm_threshold = 0;
#endif
	struct   common_grp__status_t *perror = &(perr_dev->psz_error_status);
	struct   common_grp__status_t *pwarning = &(perr_dev->psz_warning_status);

	vl53l5_psz_clear_data(
		pint_dev,
		perr_dev);

	switch (pcfg_dev->psz_general_cfg.psz__mode) {
	case VL53L5_PSZ_MODE__DISABLED:
		pwarning->status__type = VL53L5_PSZ_WARNING_DISABLED;
		break;
	case VL53L5_PSZ_MODE__ENABLED:
		pwarning->status__type = VL53L5_PSZ_WARNING_NONE;
		break;
	default:
		perror->status__type = VL53L5_PSZ_ERROR_INVALID_PARAMS;
		break;
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_filter_results(
			presults,
			pcfg_dev,
			pint_dev,
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_update_range_tol_limits(
			pcfg_dev,
			(int32_t)pint_dev->psz_tgt_meta.psz__no_of_targets,
			&(pint_dev->psz_filtered_data.psz__filtered_sigma_mm[0]),
			&(pint_dev->psz_filtered_data.psz__filtered_range_mm[0]),
			&(pint_dev->psz_rng_tol_limits),
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_find_similar_targets(
			presults,
			pcfg_dev,
			pint_dev,
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_sort(
			pint_dev,
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_update_range_tol_limits(
			pcfg_dev,
			(int32_t)pint_dev->psz_tgt_meta.psz__no_of_targets,
			&(pint_dev->psz_tgt_data.psz__range_sigma_mm[0]),
			&(pint_dev->psz_tgt_data.psz__median_range_mm[0]),
			&(pint_dev->psz_rng_tol_limits),
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_remove_duplicate_targets(
			pint_dev,
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_sort(
			pint_dev,
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_update_range_tol_limits(
			pcfg_dev,
			(int32_t)pint_dev->psz_tgt_meta.psz__no_of_targets,
			&(pint_dev->psz_tgt_data.psz__range_sigma_mm[0]),
			&(pint_dev->psz_tgt_data.psz__median_range_mm[0]),
			&(pint_dev->psz_rng_tol_limits),
			perror);
	}

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_temporal_filter(
			(int32_t)pcfg_dev->psz_general_cfg.psz__temporal_filter_depth,
			pint_dev,
			pop_dev,
			perror);
	}

#ifdef STM_PSZ_DEDIAN_RANGE_MM_STABILITY
	psz__median_range_mm = pop_dev->psz_rng_output.psz__median_range_mm;
	if (first_baseline) {
		vl53l5_k_log_debug("first sampling for baseline ");
		if (psz_median_range_mm_samplecnt < SAMPLE_MAX) {
			vl53l5_k_log_debug("sampling = [%d]", psz_median_range_mm_samplecnt);

			sum += psz__median_range_mm;
			psz_median_range_mm_samplecnt++;
		} else {
			vl53l5_k_log_debug("sampling done = [%d]", psz_median_range_mm_samplecnt);

			if (sum)
				psz_median_range_mm_baseline = sum / SAMPLE_MAX;
			else
				psz_median_range_mm_baseline = 0;

			psz_median_range_mm_samplecnt = 0;

			sum = 0;
			first_baseline = false;
		}
		current_psz_median_range_mm = psz__median_range_mm;
	} else {
		vl53l5_k_log_debug("running stability, threshold = baseline of XX percent");
		if (psz_median_range_mm_samplecnt < SAMPLE_MAX) {
			sum += psz__median_range_mm;
			psz_median_range_mm_samplecnt++;
		} else {
			if (sum)
				psz_median_range_mm_baseline = sum / SAMPLE_MAX;
			else
				psz_median_range_mm_baseline = 0;

			psz_median_range_mm_samplecnt = 0;
			sum = 0;
		}

		if (psz_median_range_mm_baseline != 0) {
			psz__median_range_mm_threshold = psz_median_range_mm_baseline;
			psz__median_range_mm_threshold *= PSZ_MEDIAN_RANGE_THRESHOLD;
			psz__median_range_mm_threshold = (psz__median_range_mm_threshold >> 9);

			vl53l5_k_log_debug("THRESHOD = [%d]", psz__median_range_mm_threshold);
			if (psz__median_range_mm > psz__median_range_mm_threshold) {
				if (((psz_median_range_mm_baseline + psz__median_range_mm_threshold)
						< psz__median_range_mm)
						|| ((psz_median_range_mm_baseline - psz__median_range_mm_threshold)
						> psz__median_range_mm)) {
					current_psz_median_range_mm = psz__median_range_mm;
					vl53l5_k_log_debug("change");
				} else {
					vl53l5_k_log_debug("sustain");
				}
			} else {
				vl53l5_k_log_debug("change");
				current_psz_median_range_mm = psz__median_range_mm;
			}
		} else {
			vl53l5_k_log_debug("change");
			current_psz_median_range_mm = psz__median_range_mm;
		}
	}

	pop_dev->psz_rng_output.psz__median_range_mm = current_psz_median_range_mm;
#endif

	if (perror->status__type == VL53L5_PSZ_ERROR_NONE &&
		pwarning->status__type != VL53L5_PSZ_WARNING_DISABLED) {
		vl53l5_psz_update_range_results(
			&(pcfg_dev->psz_general_cfg),
			&(pop_dev->psz_rng_output),
			presults,
			perror);
	}
}
