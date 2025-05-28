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

#include "dci_defs.h"
#include "dci_luts.h"
#include "vl53l5_ltf_defs.h"
#include "vl53l5_ltf_luts.h"
#include "vl53l5_long_tail_filter__set_cfg_main.h"

void vl53l5_long_tail_filter__set_cfg__lut_default(
	uint8_t                       ltf__mode,
	struct vl53l5_ltf_cfg_dev_t  *pdev)
{

	uint32_t i = 0U;

	pdev->ltf_general_cfg.ltf__mode = ltf__mode;

	for (i = 0; i < VL53L5_LTF_DEF__MAX_THRESHOLD_LUT_SIZE; i++) {
		pdev->ltf_thresh_lut_cfg.ltf__lut__slope[i] = 0;
		pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[i] = 0;
		pdev->ltf_thresh_lut_cfg.ltf__lut__knee_point[i] = 0;
		pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[i] = 0;
	}

	pdev->ltf_general_cfg.ltf__range_offset_mm = 600;
	pdev->ltf_general_cfg.ltf__thresh_lut__nb_entries = 1U;

	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[0] = 768;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[0] = -67;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[0] = 320;

	for (i = 0; i < VL53L5_LTF_DEF__MAX_TGT_STATUS_LIST_SIZE; i++)
		pdev->ltf_tgt_status_cfg.ltf__tgt_status_list[i] = 0;

	pdev->ltf_general_cfg.ltf__tgt_status__nb_entries = 4U;

	pdev->ltf_tgt_status_cfg.ltf__tgt_status_list[0] =
		DCI_TARGET_STATUS__RANGECOMPLETE;
	pdev->ltf_tgt_status_cfg.ltf__tgt_status_list[1] =
		DCI_TARGET_STATUS__RANGECOMPLETE_NO_WRAP_CHECK;
	pdev->ltf_tgt_status_cfg.ltf__tgt_status_list[2] =
		DCI_TARGET_STATUS__RANGECOMPLETE_MERGED_PULSE;
	pdev->ltf_tgt_status_cfg.ltf__tgt_status_list[3] =
		DCI_TARGET_STATUS__TARGETDUETOBLUR;
}

void vl53l5_long_tail_filter__set_cfg__lut_range_tuning_1(
	uint8_t                       ltf__mode,
	struct vl53l5_ltf_cfg_dev_t  *pdev)
{

	vl53l5_long_tail_filter__set_cfg__lut_default(
		ltf__mode,
		pdev);

	pdev->ltf_general_cfg.ltf__range_offset_mm = 600;
	pdev->ltf_general_cfg.ltf__thresh_lut__nb_entries = 1;

	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[0] = 492;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[0] = 112;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[0] = 320;

}

void vl53l5_long_tail_filter__set_cfg__lut_range_tuning_2(
	uint8_t                       ltf__mode,
	struct vl53l5_ltf_cfg_dev_t  *pdev)
{

	vl53l5_long_tail_filter__set_cfg__lut_default(
		ltf__mode,
		pdev);

	pdev->ltf_general_cfg.ltf__range_offset_mm = 600;
	pdev->ltf_general_cfg.ltf__thresh_lut__nb_entries = 8;

	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[0] = 1469;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[1] = 906;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[2] = 805;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[3] = 723;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[4] = 644;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[5] = 617;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[6] = 590;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[7] = 566;

	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[0] = 64;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[1] = 58;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[2] = 52;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[3] = 47;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[4] = 41;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[5] = 32;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[6] = -16;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[7] = -16;

	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[0] = 200;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[1] = 400;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[2] = 600;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[3] = 800;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[4] = 1000;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[5] = 1200;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[6] = 1400;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[7] = 1600;
}

void vl53l5_long_tail_filter__set_cfg__lut_range_tuning_3(
	uint8_t                       ltf__mode,
	struct vl53l5_ltf_cfg_dev_t  *pdev)
{

	vl53l5_long_tail_filter__set_cfg__lut_default(
		ltf__mode,
		pdev);

	pdev->ltf_general_cfg.ltf__range_offset_mm = 400;
	pdev->ltf_general_cfg.ltf__thresh_lut__nb_entries = 8;

	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[0] = 1469;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[1] = 906;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[2] = 805;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[3] = 723;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[4] = 644;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[5] = 617;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[6] = 590;
	pdev->ltf_thresh_lut_cfg.ltf__lut__slope[7] = 566;

	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[0] = -80;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[1] = -86;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[2] = -92;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[3] = -97;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[4] = -103;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[5] = -112;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[6] = -160;
	pdev->ltf_thresh_lut_cfg.ltf__lut__y_intercept[7] = -160;

	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[0] = 200;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[1] = 400;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[2] = 600;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[3] = 800;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[4] = 1000;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[5] = 1200;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[6] = 1400;
	pdev->ltf_thresh_lut_cfg.ltf__lut__range_mm[7] = 1600;

}

void vl53l5_long_tail_filter__set_cfg_main(
	uint32_t                      ltf_cfg_preset,
	struct vl53l5_ltf_cfg_dev_t  *pcfg_dev,
	struct vl53l5_ltf_err_dev_t  *perr_dev)
{

	struct  common_grp__status_t  *pstatus = &(perr_dev->ltf_error_status);

	pstatus->status__group = VL53L5_ALGO_LONG_TAIL_FILTER_GROUP_ID;
	pstatus->status__stage_id = VL53L5_LTF_STAGE_ID__SET_CFG;
	pstatus->status__type = VL53L5_LTF_ERROR_NONE;
	pstatus->status__debug_0 = 0U;
	pstatus->status__debug_1 = 0U;
	pstatus->status__debug_2 = 0U;

	switch (ltf_cfg_preset) {
	case VL53L5_LTF_CFG_PRESET__NONE:
		vl53l5_long_tail_filter__set_cfg__lut_default(
			VL53L5_LTF_MODE__DISABLED,
			pcfg_dev);
		break;
	case VL53L5_LTF_CFG_PRESET__LUT_RANGE_TUNING_0:
		vl53l5_long_tail_filter__set_cfg__lut_default(
			VL53L5_LTF_MODE__LUT__RANGE,
			pcfg_dev);
		break;
	case VL53L5_LTF_CFG_PRESET__LUT_RANGE_TUNING_1:
		vl53l5_long_tail_filter__set_cfg__lut_range_tuning_1(
			VL53L5_LTF_MODE__LUT__RANGE,
			pcfg_dev);
		break;
	case VL53L5_LTF_CFG_PRESET__LUT_RANGE_TUNING_2:
		vl53l5_long_tail_filter__set_cfg__lut_range_tuning_2(
			VL53L5_LTF_MODE__LUT__RANGE,
			pcfg_dev);
		break;
	case VL53L5_LTF_CFG_PRESET__LUT_RANGE_TUNING_3:
		vl53l5_long_tail_filter__set_cfg__lut_range_tuning_3(
			VL53L5_LTF_MODE__LUT__RANGE,
			pcfg_dev);
		break;
	default:
		pstatus->status__type =
			VL53L5_LTF_ERROR_INVALID_PARAMS;
		break;
	}
}
