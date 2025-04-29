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
#include "vl53l5_ltf_version_defs.h"
#include "vl53l5_ltf_defs.h"
#include "vl53l5_ltf_luts.h"
#include "vl53l5_long_tail_filter_funcs.h"
#include "vl53l5_long_tail_filter_main.h"
#ifdef STM_VL53L5_SUPPORT_LEGACY_CODE
void vl53l5_ltf_get_version(
	struct  common_grp__version_t  *pversion)
{

	pversion->version__major = VL53L5_LTF_DEF__VERSION__MAJOR;
	pversion->version__minor = VL53L5_LTF_DEF__VERSION__MINOR;
	pversion->version__build = VL53L5_LTF_DEF__VERSION__BUILD;
	pversion->version__revision = VL53L5_LTF_DEF__VERSION__REVISION;
}
#endif
void vl53l5_long_tail_filter_main(
	struct vl53l5_ltf_cfg_dev_t  *pcfg_dev,
	struct vl53l5_range_results_t          *presults,
	struct vl53l5_ltf_err_dev_t  *perr_dev)
{

	int32_t  status = VL53L5_LTF_ERROR_NONE;

	int32_t  zone = 0;
	int32_t  no_of_zones =
		(int32_t)presults->common_data.rng__no_of_zones;
	int32_t  max_targets =
		(int32_t)presults->common_data.rng__max_targets_per_zone;
	int32_t  no_of_targets = 0;

	vl53l5_ltf_clear_data(perr_dev);

	for (zone = 0; zone < no_of_zones && status == VL53L5_LTF_ERROR_NONE; zone++) {

		no_of_targets =
			(int32_t)presults->per_zone_results.rng__no_of_targets[zone];

		if (no_of_targets > 1) {
			switch (pcfg_dev->ltf_general_cfg.ltf__mode) {
			case VL53L5_LTF_MODE__DISABLED:

				break;
			case VL53L5_LTF_MODE__LUT__RANGE:
				vl53l5_ltf_zone_lut_range(
					zone,
					max_targets,
					no_of_targets,
					pcfg_dev,
					presults,
					&(perr_dev->ltf_error_status));
				break;
			default:
				perr_dev->ltf_error_status.status__type =
					VL53L5_LTF_ERROR_INVALID_PARAMS;
				break;
			}
		}

	}
}
