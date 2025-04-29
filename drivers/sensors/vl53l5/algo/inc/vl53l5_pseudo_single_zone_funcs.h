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

#ifndef VL53L5_PSEUDO_SINGLE_ZONE_FUNCS_H_
#define VL53L5_PSEUDO_SINGLE_ZONE_FUNCS_H_

#include "vl53l5_types.h"
#include "common_datatype_structs.h"
#include "vl53l5_results_map.h"
#include "vl53l5_psz_cfg_map.h"
#include "vl53l5_psz_int_map.h"
#include "vl53l5_psz_op_map.h"
#include "vl53l5_psz_err_map.h"

void vl53l5_psz_clear_data(
	struct vl53l5_psz_int_dev_t          *pint_dev,
	struct vl53l5_psz_err_dev_t          *perr_dev);

void vl53l5_psz_div_and_clip_uint64(
	uint64_t                       ip_value,
	uint64_t                       ip_divisor,
	uint64_t                       upper_clip,
	uint32_t                      *presult,
	struct common_grp__status_t   *pstatus);

void vl53l5_psz_div_and_clip_uint32(
	uint32_t                       ip_value,
	uint32_t                       ip_divisor,
	uint32_t                       upper_clip,
	uint32_t                      *presult,
	struct common_grp__status_t   *pstatus);

void vl53l5_psz_div_and_clip_int32(
	int32_t                       ip_value,
	int32_t                       ip_divisor,
	int32_t                       lower_clip,
	int32_t                       upper_clip,
	int32_t                       *presult,
	struct common_grp__status_t   *pstatus);

void vl53l5_psz_linear_interp(
	int32_t                           range_mm,
	uint32_t                          lut_no_of_entries,
	int16_t                          *plut_ranges,
	int16_t                          *plut_values,
	uint32_t                         *pop_idx,
	int32_t                          *pop_value,
	struct common_grp__status_t      *pstatus);

uint32_t vl53l5_psz_calc_accuracy(
	uint16_t       range_sigma_mm,
	int16_t        median_range_mm);

uint32_t  vl53l5_psz_is_target_valid(
	uint32_t    target_idx,
	uint32_t    rng__no_of_targets,
	uint32_t    target_status,
	uint8_t    *ptgt_status_list);

void vl53l5_psz_filter_results(
	struct   vl53l5_range_results_t          *presults,
	struct   vl53l5_psz_cfg_dev_t  *pcfg_dev,
	struct   vl53l5_psz_int_dev_t  *pint_dev,
	struct   common_grp__status_t  *pstatus);

void vl53l5_psz_calc_tol_limits(
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	int32_t                                 sigma_mm,
	int32_t                                 range_mm,
	int16_t                                *ptol_min_mm,
	int16_t                                *ptol_max_mm,
	struct   common_grp__status_t          *pstatus);

void vl53l5_psz_update_range_tol_limits(
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	int32_t                                 no_of_ranges,
	uint16_t                               *psigmas,
	int16_t                                *pranges,
	struct   vl53l5_psz__tol__limits_t     *plimits,
	struct   common_grp__status_t          *pstatus);

void vl53l5_psz_accumulate_target_stats(
	uint32_t                                idx,
	struct   vl53l5_range_results_t                  *presults,
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus);

void vl53l5_psz_compute_target_stats(
	uint32_t                                idx,
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus);

void vl53l5_psz_find_similar_targets(
	struct   vl53l5_range_results_t                  *presults,
	struct   vl53l5_psz_cfg_dev_t          *pcfg_dev,
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus);

void vl53l5_psz_swap_uint32(
	uint32_t  *p0,
	uint32_t  *p1);

void vl53l5_psz_swap_uint16(
	uint16_t  *p0,
	uint16_t  *p1);

void vl53l5_psz_swap_int16(
	int16_t  *p0,
	int16_t  *p1);

void vl53l5_psz_swap_uint8(
	uint8_t  *p0,
	uint8_t  *p1);

void vl53l5_psz_sort(
	struct   vl53l5_psz_int_dev_t          *pint_dev,
	struct   common_grp__status_t          *pstatus);

void vl53l5_psz_remove_duplicate_targets(
	struct   vl53l5_psz_int_dev_t         *pint_dev,
	struct   common_grp__status_t         *pstatus);

void vl53l5_psz_clear_range_data(
	uint16_t                               amb_dmax_mm,
	struct   vl53l5_psz__range__data_t    *prng_data);

void vl53l5_psz_set_range_data(
	int32_t                                  idx,
	struct   vl53l5_psz__tgt__data_t        *ptgt_data,
	struct   vl53l5_psz__range__data_t      *prng_data);

void vl53l5_psz_temporal_filter(
	int32_t                                depth_temporal,
	struct   vl53l5_psz_int_dev_t         *pint_dev,
	struct   vl53l5_psz_op_dev_t          *pop_dev,
	struct   common_grp__status_t         *pstatus);

void vl53l5_psz_update_range_results(
	struct   vl53l5_psz__general__cfg_t   *pgen_cfg,
	struct   vl53l5_psz__range__data_t    *prng_output,
	struct   vl53l5_range_results_t                 *presults,
	struct   common_grp__status_t         *pstatus);

#endif
