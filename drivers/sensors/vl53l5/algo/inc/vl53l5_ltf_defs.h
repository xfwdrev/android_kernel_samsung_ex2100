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

#ifndef __VL53L5_LTF_DEFS_H__
#define __VL53L5_LTF_DEFS_H__

#include "vl53l5_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L5_ALGO_LONG_TAIL_FILTER_GROUP_ID \
	((int16_t) 17)

#define VL53L5_LTF_DEF__MAX_STRING_LENGTH \
	((uint32_t) 256U)

#define VL53L5_LTF_DEF__MAX_THRESHOLD_LUT_SIZE \
	((uint32_t) 8U)

#define VL53L5_LTF_DEF__MAX_TGT_STATUS_LIST_SIZE \
	((uint32_t) 8U)

#define VL53L5_LTF_DEF__SLOPE_INTEGER_BITS \
	((uint32_t) 12U)

#define VL53L5_LTF_DEF__SLOPE_FRAC_BITS \
	((uint32_t) 12U)

#define VL53L5_LTF_DEF__SLOPE_MAX_VALUE \
	((int32_t) 8388607)

#define VL53L5_LTF_DEF__SLOPE_MIN_VALUE \
	((int32_t) -8388608UL)

#define VL53L5_LTF_DEF__SLOPE_UNITY_VALUE \
	((uint32_t) 4096U)

#define VL53L5_LTF_DEF__RATIO_INTEGER_BITS \
	((uint32_t) 16U)

#define VL53L5_LTF_DEF__RATIO_FRAC_BITS \
	((uint32_t) 4U)

#define VL53L5_LTF_DEF__RATIO_MAX_VALUE \
	((int32_t) 524287)

#define VL53L5_LTF_DEF__RATIO_MIN_VALUE \
	((int32_t) -524288UL)

#define VL53L5_LTF_DEF__RATIO_UNITY_VALUE \
	((uint32_t) 16U)

#define VL53L5_LTF_DEF__THRESH_INTEGER_BITS \
	((uint32_t) 16U)

#define VL53L5_LTF_DEF__THRESH_FRAC_BITS \
	((uint32_t) 4U)

#define VL53L5_LTF_DEF__THRESH_MAX_VALUE \
	((int32_t) 524287)

#define VL53L5_LTF_DEF__THRESH_MIN_VALUE \
	((int32_t) -524288UL)

#define VL53L5_LTF_DEF__THRESH_UNITY_VALUE \
	((uint32_t) 16U)

#define VL53L5_LTF_DEF__DIFF_MM_INTEGER_BITS \
	((uint32_t) 14U)

#define VL53L5_LTF_DEF__DIFF_MM_FRAC_BITS \
	((uint32_t) 2U)

#define VL53L5_LTF_DEF__DIFF_MM_MAX_VALUE \
	((int32_t) 32767)

#define VL53L5_LTF_DEF__DIFF_MM_MIN_VALUE \
	((int32_t) -32768)

#define VL53L5_LTF_DEF__DIFF_MM_UNITY_VALUE \
	((uint32_t) 4U)

#ifdef __cplusplus
}
#endif

#endif
