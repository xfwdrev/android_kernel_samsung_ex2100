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

#ifndef __VL53L5_OTF_LUTS_H__
#define __VL53L5_OTF_LUTS_H__

#include "vl53l5_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L5_OTF_MODE__INVALID \
	((uint8_t) 0U)

#define VL53L5_OTF_MODE__DISABLED \
	((uint8_t) 1U)

#define VL53L5_OTF_MODE__ENABLED \
	((uint8_t) 2U)

#define VL53L5_OTF_ROTATION_SEL__NONE \
	((uint8_t) 0U)

#define VL53L5_OTF_ROTATION_SEL__CW_90 \
	((uint8_t) 1U)

#define VL53L5_OTF_ROTATION_SEL__CW_180 \
	((uint8_t) 2U)

#define VL53L5_OTF_ROTATION_SEL__CW_270 \
	((uint8_t) 3U)

#define VL53L5_OTF_ROTATION_SEL__MIRROR_X \
	((uint8_t) 4U)

#define VL53L5_OTF_ROTATION_SEL__MIRROR_Y \
	((uint8_t) 5U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__NOUPDATE \
	((uint32_t) 1U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__MSRCNOTARGET \
	((uint32_t) 2U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__RANGEPHASECHECK \
	((uint32_t) 4U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__SIGMATHRESHOLDCHECK \
	((uint32_t) 8U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__PHASECONSISTENCY \
	((uint32_t) 16U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__RANGECOMPLETE \
	((uint32_t) 32U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__RANGECOMPLETE_NO_WRAP_CHECK \
	((uint32_t) 64U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__EVENTCONSISTENCY \
	((uint32_t) 128U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__MINSIGNALEVENTCHECK \
	((uint32_t) 256U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__RANGECOMPLETE_MERGED_PULSE \
	((uint32_t) 512U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__PREV_RANGE_NO_TARGETS \
	((uint32_t) 1024U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__RANGEIGNORETHRESHOLD \
	((uint32_t) 2048U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__TARGETDUETOBLUR \
	((uint32_t) 4096U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__TARGETDUETOLONGTAIL \
	((uint32_t) 8192U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__TARGETSEPARATIONTOOSMALL \
	((uint32_t) 16384U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__TARGETDUETOBLUR_NO_WRAP_CHECK \
	((uint32_t) 32768U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__TARGETDUETOBLUR_MERGED_PULSE \
	((uint32_t) 65536U)

#define VL53L5_OTF__TGT_OP_FILTER_EN__RANGECOMPLETE_PILE_UP \
	((uint32_t) 131072U)

#define VL53L5_OTF__TGT_OP_FILTER_FLAGS__REMOVE_BLURRED \
	((uint32_t) 1916U)
#define VL53L5_OTF__TGT_OP_FILTER_FLAGS__VALID_ONLY \
	((uint32_t) 608U)
#define VL53L5_OTF__TGT_OP_FILTER_FLAGS__ALL \
	((uint32_t) 262142U)
#define VL53L5_OTF__TGT_OP_FILTER_FLAGS \
	((uint32_t) 608U)

#define VL53L5_OTF_STAGE_ID__INVALID \
	((int16_t) 0)

#define VL53L5_OTF_STAGE_ID__SET_CFG \
	((int16_t) 1)

#define VL53L5_OTF_STAGE_ID__INIT \
	((int16_t) 2)

#define VL53L5_OTF_STAGE_ID__INIT_ZONE_ORDER_COEFFS \
	((int16_t) 3)

#define VL53L5_OTF_STAGE_ID__INIT_ZONE_ORDER_SEQ \
	((int16_t) 4)

#define VL53L5_OTF_STAGE_ID__NEW_SEQUENCE_IDX \
	((int16_t) 5)

#define VL53L5_OTF_STAGE_ID__TARGET_FILTER \
	((int16_t) 6)

#define VL53L5_OTF_STAGE_ID__INIT_SINGLE_ZONE_DATA \
	((int16_t) 7)

#define VL53L5_OTF_STAGE_ID__MAIN \
	((int16_t) 8)

#define VL53L5_OTF_ERROR_NONE \
	((int16_t) 0)

#define VL53L5_OTF_ERROR_INVALID_PARAMS \
	((int16_t) 1)

#define VL53L5_OTF_ERROR_DIVISION_BY_ZERO \
	((int16_t) 2)

#define VL53L5_OTF_ERROR_NOT_IMPLEMENTED \
	((int16_t) 3)

#define VL53L5_OTF_ERROR_UNDEFINED \
	((int16_t) 4)

#define VL53L5_OTF_ERROR_UNSUPPORTED_NO_OF_ZONES \
	((int16_t) 5)

#define VL53L5_OTF_ERROR_INVALID_ROTATION_SEL \
	((int16_t) 6)

#define VL53L5_OTF_ERROR_INVALID_ZONE_ID \
	((int16_t) 7)

#define VL53L5_OTF_ERROR_INVALID_SINGLE_ZONE_ID \
	((int16_t) 8)

#define VL53L5_OTF_WARNING_NONE \
	((int16_t) 0)

#define VL53L5_OTF_WARNING_DISABLED \
	((int16_t) 1)

#ifdef __cplusplus
}
#endif

#endif
