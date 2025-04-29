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
#include "vl53l5_pseudo_single_zone__set_cfg_main.h"

void vl53l5_pseudo_single_zone__set_cfg__zone_ids__clear(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	uint32_t i = 0U;

	for (i = 0U; i < VL53L5_PSZ_DEF__MAX_ZONES; i++)
		pdev->psz_zone_id_list.psz__zone_id[i] = 0;

	pdev->psz_general_cfg.psz__zone_list__nb_entries = 0;
}

void vl53l5_pseudo_single_zone__set_cfg__zone_ids__4x4(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__zone_ids__clear(pdev);

	pdev->psz_general_cfg.psz__zone_list__nb_entries = 4;

	pdev->psz_zone_id_list.psz__zone_id[0] = 5;
	pdev->psz_zone_id_list.psz__zone_id[1] = 6;
	pdev->psz_zone_id_list.psz__zone_id[2] = 9;
	pdev->psz_zone_id_list.psz__zone_id[3] = 10;
}

void vl53l5_pseudo_single_zone__set_cfg__zone_ids__8x8(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__zone_ids__clear(pdev);

	pdev->psz_general_cfg.psz__zone_list__nb_entries = 4;
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
	pdev->psz_zone_id_list.psz__zone_id[0] = 27;
	pdev->psz_zone_id_list.psz__zone_id[1] = 28;
	pdev->psz_zone_id_list.psz__zone_id[2] = 35;
	pdev->psz_zone_id_list.psz__zone_id[3] = 36;
#endif
#ifdef STM_VL53L5_SUPPORT_SEC_CODE
	pdev->psz_zone_id_list.psz__zone_id[0] = 19;
	pdev->psz_zone_id_list.psz__zone_id[1] = 20;
	pdev->psz_zone_id_list.psz__zone_id[2] = 27;
	pdev->psz_zone_id_list.psz__zone_id[3] = 28;
#endif
}

void vl53l5_pseudo_single_zone__set_cfg__tol_lut__clear(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	uint32_t i = 0;

	for (i = 0; i < VL53L5_PSZ_DEF__MAX_TOLERANCE_LUT_SIZE; i++) {
		pdev->psz_rng_tol_lut.psz__lut__tolerance_pc[i] = 0;
		pdev->psz_rng_tol_lut.psz__lut__tolerance_mm[i] = 0;
		pdev->psz_rng_tol_lut.psz__lut__tolerance_sigma[i] = 0;
		pdev->psz_rng_tol_lut.psz__lut__range_mm[i] = 0;
	}

	pdev->psz_general_cfg.psz__tol_lut__nb_entries = 0;
}

void vl53l5_pseudo_single_zone__set_cfg__tol_lut__percent(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__tol_lut__clear(pdev);

	pdev->psz_general_cfg.psz__tol_lut__nb_entries = 2;

	pdev->psz_rng_tol_lut.psz__lut__tolerance_pc[0] = 15;
	pdev->psz_rng_tol_lut.psz__lut__tolerance_mm[0] = 20 * 4;

	pdev->psz_rng_tol_lut.psz__lut__range_mm[0] = 250 * 4;

	pdev->psz_rng_tol_lut.psz__lut__tolerance_pc[1] = 15;
	pdev->psz_rng_tol_lut.psz__lut__tolerance_mm[1] = 100 * 4;

	pdev->psz_rng_tol_lut.psz__lut__range_mm[1] = (250 * 4) + 1;

}

void vl53l5_pseudo_single_zone__set_cfg__tol_lut__sigma(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__tol_lut__clear(pdev);

	pdev->psz_general_cfg.psz__tol_lut__nb_entries = 2;

	pdev->psz_rng_tol_lut.psz__lut__tolerance_mm[0] = 10 * 4;

	pdev->psz_rng_tol_lut.psz__lut__tolerance_sigma[0] = 7 * 256;

	pdev->psz_rng_tol_lut.psz__lut__range_mm[0] = 250 * 4;

	pdev->psz_rng_tol_lut.psz__lut__tolerance_mm[1] = 10 * 4;

	pdev->psz_rng_tol_lut.psz__lut__tolerance_sigma[1] = 7 * 256;

	pdev->psz_rng_tol_lut.psz__lut__range_mm[1] = (250 * 4) + 1;

}

void vl53l5_pseudo_single_zone__set_cfg__tgt_status__clear(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	uint32_t i = 0;

	for (i = 0; i < VL53L5_PSZ_DEF__MAX_TGT_STATUS_LIST_SIZE; i++) {
		pdev->psz_tgt_status_list_t0.psz__tgt_status[i] = 0;
		pdev->psz_tgt_status_list_t1.psz__tgt_status[i] = 0;
	}

}

void vl53l5_pseudo_single_zone__set_cfg__tgt_status__default(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__tgt_status__clear(pdev);

	pdev->psz_tgt_status_list_t0.psz__tgt_status[0] =
		DCI_TARGET_STATUS__RANGECOMPLETE;
	pdev->psz_tgt_status_list_t0.psz__tgt_status[1] =
		DCI_TARGET_STATUS__RANGECOMPLETE_NO_WRAP_CHECK;
	pdev->psz_tgt_status_list_t0.psz__tgt_status[2] =
		DCI_TARGET_STATUS__RANGECOMPLETE_MERGED_PULSE;

	pdev->psz_tgt_status_list_t1.psz__tgt_status[0] =
		DCI_TARGET_STATUS__RANGECOMPLETE;
	pdev->psz_tgt_status_list_t1.psz__tgt_status[1] =
		DCI_TARGET_STATUS__RANGECOMPLETE_NO_WRAP_CHECK;
	pdev->psz_tgt_status_list_t1.psz__tgt_status[2] =
		DCI_TARGET_STATUS__RANGECOMPLETE_MERGED_PULSE;
	pdev->psz_tgt_status_list_t1.psz__tgt_status[3] =
		DCI_TARGET_STATUS__TARGETDUETOBLUR;
}

void vl53l5_pseudo_single_zone__set_cfg__tgt_status__inc_wrap(
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__tgt_status__default(pdev);

	pdev->psz_tgt_status_list_t0.psz__tgt_status[3] =
		DCI_TARGET_STATUS__PHASECONSISTENCY;

	pdev->psz_tgt_status_list_t1.psz__tgt_status[4] =
		DCI_TARGET_STATUS__PHASECONSISTENCY;
}

void vl53l5_pseudo_single_zone__set_cfg__default(
	uint8_t                       psz__mode,
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	pdev->psz_general_cfg.psz__mode = psz__mode;

	pdev->psz_general_cfg.psz__zone__idx             = 251U;
	pdev->psz_general_cfg.psz__temporal_filter_depth =   5U;
	pdev->psz_general_cfg.psz__gen_cfg__spare_0      =   0U;
	pdev->psz_general_cfg.psz__gen_cfg__spare_1      =   0U;
	pdev->psz_general_cfg.psz__gen_cfg__spare_2      =   0U;

	vl53l5_pseudo_single_zone__set_cfg__tol_lut__percent(pdev);

	vl53l5_pseudo_single_zone__set_cfg__zone_ids__8x8(pdev);

	vl53l5_pseudo_single_zone__set_cfg__tgt_status__default(pdev);
}

void vl53l5_pseudo_single_zone__set_cfg__tuning_0(
	uint8_t                       psz__mode,
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__default(
		psz__mode,
		pdev);

	vl53l5_pseudo_single_zone__set_cfg__zone_ids__4x4(pdev);

}

void vl53l5_pseudo_single_zone__set_cfg__tuning_2(
	uint8_t                       psz__mode,
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__default(
		psz__mode,
		pdev);

	vl53l5_pseudo_single_zone__set_cfg__tol_lut__sigma(pdev);
}

void vl53l5_pseudo_single_zone__set_cfg__tuning_3(
	uint8_t                       psz__mode,
	struct vl53l5_psz_cfg_dev_t  *pdev)
{

	vl53l5_pseudo_single_zone__set_cfg__default(
		psz__mode,
		pdev);

	vl53l5_pseudo_single_zone__set_cfg__tgt_status__inc_wrap(pdev);
}

void vl53l5_pseudo_single_zone__set_cfg_main(
	uint32_t                      psz_cfg_preset,
	struct vl53l5_psz_cfg_dev_t  *pcfg_dev,
	struct vl53l5_psz_err_dev_t  *perr_dev)
{

	struct  common_grp__status_t  *pstatus = &(perr_dev->psz_error_status);

	pstatus->status__group = VL53L5_ALGO_PSEUDO_SINGLE_ZONE_GROUP_ID;
	pstatus->status__stage_id = VL53L5_PSZ_STAGE_ID__SET_CFG;
	pstatus->status__type = VL53L5_PSZ_ERROR_NONE;
	pstatus->status__debug_0 = 0U;
	pstatus->status__debug_1 = 0U;
	pstatus->status__debug_2 = 0U;

	switch (psz_cfg_preset) {
	case VL53L5_PSZ_CFG_PRESET__NONE:
		vl53l5_pseudo_single_zone__set_cfg__default(
			VL53L5_PSZ_MODE__DISABLED,
			pcfg_dev);
		break;
	case VL53L5_PSZ_CFG_PRESET__TUNING_0:
		vl53l5_pseudo_single_zone__set_cfg__tuning_0(
			VL53L5_PSZ_MODE__ENABLED,
			pcfg_dev);
		break;
	case VL53L5_PSZ_CFG_PRESET__TUNING_1:
		vl53l5_pseudo_single_zone__set_cfg__default(
			VL53L5_PSZ_MODE__ENABLED,
			pcfg_dev);
		break;
	case VL53L5_PSZ_CFG_PRESET__TUNING_2:
		vl53l5_pseudo_single_zone__set_cfg__tuning_2(
			VL53L5_PSZ_MODE__ENABLED,
			pcfg_dev);
		break;
	case VL53L5_PSZ_CFG_PRESET__TUNING_3:
		vl53l5_pseudo_single_zone__set_cfg__tuning_3(
			VL53L5_PSZ_MODE__ENABLED,
			pcfg_dev);
		break;
	default:
		pstatus->status__type =
			VL53L5_PSZ_ERROR_INVALID_PARAMS;
		break;
	}
}
