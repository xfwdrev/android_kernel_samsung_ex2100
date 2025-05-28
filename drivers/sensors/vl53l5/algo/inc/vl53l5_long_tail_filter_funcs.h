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

#ifndef VL53L5_LONG_TAIL_FILTER_FUNCS_H_
#define VL53L5_LONG_TAIL_FILTER_FUNCS_H_

#include "common_datatype_structs.h"
#include "vl53l5_ltf_cfg_map.h"
#include "vl53l5_ltf_err_map.h"
#include "vl53l5_results_map.h"

void vl53l5_ltf_clear_data(
	struct vl53l5_ltf_err_dev_t          *perr_dev);

void vl53l5_ltf_linear_interp(
	int32_t                           range_mm,
	uint32_t                          lut_no_of_entries,
	int32_t                          *plut_ranges,
	int32_t                          *plut_values,
	uint32_t                         *pop_idx,
	int32_t                          *pop_value,
	struct common_grp__status_t      *pstatus);

int32_t vl53l5_ltf_is_target_valid(
	uint8_t     target_status,
	uint32_t    tgt_status__nb_entries,
	uint8_t    *ptgt_status_list);

void vl53l5_ltf_calc_rate_ratio(
	uint32_t                          rate0,
	uint32_t                          rate1,
	int32_t                          *pratio,
	struct common_grp__status_t      *pstatus);

void vl53l5_ltf_calc_threshold(
	int32_t                               primary_range_mm,
	int32_t                               diff_range_mm,
	struct vl53l5_ltf__general__cfg_t    *pcfg,
	struct vl53l5_ltf__thres_lut__cfg_t  *plut,
	int32_t                              *pthresh,
	struct common_grp__status_t          *pstatus);

void vl53l5_ltf_zone_lut_range(
	int32_t                       zone,
	int32_t                       max_targets,
	int32_t                       no_of_targets,
	struct vl53l5_ltf_cfg_dev_t  *pcfg_dev,
	struct vl53l5_range_results_t          *presults,
	struct common_grp__status_t  *pstatus);

#endif
