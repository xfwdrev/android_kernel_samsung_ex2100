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

#ifndef __VL53L5_PSZ_STRUCTS_H__
#define __VL53L5_PSZ_STRUCTS_H__

#include "vl53l5_psz_defs.h"
#include "vl53l5_psz_luts.h"
#include "vl53l5_psz_structs.h"
#include "vl53l5_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct vl53l5_psz__general__cfg_t {

	uint8_t psz__mode;

	uint8_t psz__tol_lut__nb_entries;

	uint8_t psz__zone_list__nb_entries;

	uint8_t psz__zone__idx;

	uint8_t psz__temporal_filter_depth;

	uint8_t psz__gen_cfg__spare_0;

	uint8_t psz__gen_cfg__spare_1;

	uint8_t psz__gen_cfg__spare_2;
};

struct vl53l5_psz__tol_lut__cfg_t {

	int16_t psz__lut__tolerance_pc[VL53L5_PSZ_DEF__MAX_TOLERANCE_LUT_SIZE];

	int16_t psz__lut__tolerance_mm[VL53L5_PSZ_DEF__MAX_TOLERANCE_LUT_SIZE];

	int16_t psz__lut__tolerance_sigma[
		VL53L5_PSZ_DEF__MAX_TOLERANCE_LUT_SIZE];

	int16_t psz__lut__range_mm[VL53L5_PSZ_DEF__MAX_TOLERANCE_LUT_SIZE];
};

struct vl53l5_psz__zone_list__cfg_t {

	uint8_t psz__zone_id[VL53L5_PSZ_DEF__MAX_ZONES];
};

struct vl53l5_psz__tgt_status__cfg_t {

	uint8_t psz__tgt_status[VL53L5_PSZ_DEF__MAX_TGT_STATUS_LIST_SIZE];
};

struct vl53l5_psz__filtered__data_t {

	uint16_t psz__filtered_sigma_mm[VL53L5_PSZ_DEF__MAX_TARGETS];

	int16_t psz__filtered_range_mm[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint8_t psz__filtered_tgts_idx[VL53L5_PSZ_DEF__MAX_TARGETS];
};

struct vl53l5_psz__tol__limits_t {

	int16_t psz__tol__min_mm[VL53L5_PSZ_DEF__MAX_TARGETS];

	int16_t psz__tol__max_mm[VL53L5_PSZ_DEF__MAX_TARGETS];
};

struct vl53l5_psz__sum__data_t {

	uint64_t psz__sum__amb_rate_kcps_per_spad;

	uint64_t psz__sum__peak_rate_kcps_per_spad;

	uint32_t psz__sum__effective_spad_count;

	uint32_t psz__sum__amb_dmax_mm;

	uint32_t psz__sum__range_sigma_mm;

	int32_t psz__sum__median_range_mm;
};

struct vl53l5_psz__tgt__meta_t {

	uint16_t psz__max__amb_dmax_mm;

	uint16_t psz__avg__amb_dmax_mm;

	uint8_t psz__max_targets;

	uint8_t psz__no_of_targets;

	uint8_t psz__filtered__spare_0;

	uint8_t psz__filtered__spare_1;
};

struct vl53l5_psz__tgt__data_t {

	uint32_t psz__amb_rate_kcps_per_spad[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint32_t psz__peak_rate_kcps_per_spad[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint32_t psz__effective_spad_count[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint16_t psz__amb_dmax_mm[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint16_t psz__range_sigma_mm[VL53L5_PSZ_DEF__MAX_TARGETS];

	int16_t psz__median_range_mm[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint8_t psz__target_status[VL53L5_PSZ_DEF__MAX_TARGETS];

	uint8_t psz__no_of_matches[VL53L5_PSZ_DEF__MAX_TARGETS];
};

struct vl53l5_psz__temporal__meta_t {

	int16_t psz__new__range_mm;

	int16_t psz__last__range_mm;

	int16_t psz__output__range_mm;

	uint8_t psz__output__no_of_matches;

	uint8_t psz__new__range_found;
};

struct vl53l5_psz__temporal__data_t {

	int16_t psz__range_mm[VL53L5_PSZ_DEF__MAX_TEMPORAL_DEPTH];

	uint8_t psz__no_of_matches[VL53L5_PSZ_DEF__MAX_TEMPORAL_DEPTH];
};

struct vl53l5_psz__range__data_t {

	uint32_t psz__amb_rate_kcps_per_spad;

	uint32_t psz__peak_rate_kcps_per_spad;

	uint32_t psz__effective_spad_count;

	uint16_t psz__amb_dmax_mm;

	uint16_t psz__range_sigma_mm;

	int16_t psz__median_range_mm;

	uint8_t psz__target_status;

	uint8_t psz__no_of_matches;

	uint8_t psz__rng_data__spare_0;

	uint8_t psz__rng_data__spare_1;

	uint8_t psz__rng_data__spare_2;

	uint8_t psz__rng_data__spare_3;
};

#ifdef __cplusplus
}
#endif

#endif
