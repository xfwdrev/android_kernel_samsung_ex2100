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

#ifndef __VL53L5_PSZ_MAP_H__
#define __VL53L5_PSZ_MAP_H__

#include "common_datatype_defs.h"
#include "common_datatype_structs.h"
#include "vl53l5_psz_defs.h"
#include "vl53l5_psz_structs.h"
#include "packing_structs.h"
#include "vl53l5_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct vl53l5_psz_dev_t {

	struct vl53l5_psz__general__cfg_t psz_general_cfg;

	struct vl53l5_psz__tol_lut__cfg_t psz_rng_tol_lut;

	struct vl53l5_psz__zone_list__cfg_t psz_zone_id_list;

	struct vl53l5_psz__tgt_status__cfg_t psz_tgt_status_list_t0;

	struct vl53l5_psz__tgt_status__cfg_t psz_tgt_status_list_t1;

	struct vl53l5_psz__filtered__data_t psz_filtered_data;

	struct vl53l5_psz__tol__limits_t psz_rng_tol_limits;

	struct vl53l5_psz__sum__data_t psz_sum_data;

	struct vl53l5_psz__tgt__meta_t psz_tgt_meta;

	struct vl53l5_psz__tgt__data_t psz_tgt_data;

	struct vl53l5_psz__temporal__data_t psz_temporal_data;

	struct vl53l5_psz__range__data_t psz_rng_new;

	struct vl53l5_psz__range__data_t psz_rng_last;

	struct vl53l5_psz__range__data_t psz_rng_output;

	struct common_grp__status_t psz_error_status;

	struct common_grp__status_t psz_warning_status;

};

#ifdef __cplusplus
}
#endif

#endif
