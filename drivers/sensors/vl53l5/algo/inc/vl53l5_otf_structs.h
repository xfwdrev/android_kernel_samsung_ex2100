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

#ifndef __VL53L5_OTF_STRUCTS_H__
#define __VL53L5_OTF_STRUCTS_H__

#include "vl53l5_otf_defs.h"
#include "vl53l5_otf_luts.h"
#include "vl53l5_otf_structs.h"
#include "vl53l5_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct vl53l5_otf__general__cfg_t {

	uint32_t otf__tgt_status_op_enables;

	uint8_t otf__rotation_sel;

	uint8_t otf__mode;

	uint8_t otf__single_zone_op_en;

	uint8_t otf__range_clip_en;
};

struct vl53l5_otf__op__zone_order__cfg_t {

	int16_t otf__no_of_zones;

	int16_t otf__grid_size;

	int16_t otf__col_m;

	int16_t otf__col_x;

	int16_t otf__row_m;

	int16_t otf__row_x;
};

struct vl53l5_otf__op__zone_data_t {

	uint32_t amb_rate_kcps_per_spad;

	uint32_t rng__effective_spad_count;

	uint16_t amb_dmax_mm;

	uint8_t rng__no_of_targets;

	uint8_t rng__zone_id;
};

struct vl53l5_otf__op__target_data_t {

	uint32_t peak_rate_kcps_per_spad[VL53L5_OTF_DEF__MAX_TARGETS_PER_ZONE];

	uint16_t range_sigma_mm[VL53L5_OTF_DEF__MAX_TARGETS_PER_ZONE];

	int16_t median_range_mm[VL53L5_OTF_DEF__MAX_TARGETS_PER_ZONE];

	uint8_t target_status[VL53L5_OTF_DEF__MAX_TARGETS_PER_ZONE];
};

#ifdef __cplusplus
}
#endif

#endif
