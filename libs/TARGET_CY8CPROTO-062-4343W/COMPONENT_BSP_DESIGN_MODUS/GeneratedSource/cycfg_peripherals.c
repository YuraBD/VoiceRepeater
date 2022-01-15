/*******************************************************************************
* File Name: cycfg_peripherals.c
*
* Description:
* Peripheral Hardware Block configuration
* This file was automatically generated and should not be modified.
* Tools Package 2.4.0.5972
* mtb-pdl-cat1 2.3.1.11964
* personalities 6.0.0.0
* udd 3.0.0.1525
*
********************************************************************************
* Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#include "cycfg_peripherals.h"

#define tcpwm_1_cnt_0_INPUT_DISABLED 0x7U

cy_stc_csd_context_t cy_csd_0_context = 
{
    .lockKey = CY_CSD_NONE_KEY,
};
static const cy_stc_csdadc_ch_pin_t CYBSP_CSD_csdadc_channel_list[] = 
{
    [0] = {
            .ioPcPtr = GPIO_PRT6, 
            .pin = 5u, 
          },
};
static const cy_stc_csdidac_pin_t CYBSP_CSD_csdidac_a_pin = 
{
    .ioPcPtr = GPIO_PRT5,
    .pin = 7u,
};
const cy_stc_csdadc_config_t CYBSP_CSD_csdadc_config = 
{
    .ptrPinList = &CYBSP_CSD_csdadc_channel_list[0u],
    .base = CYBSP_CSD_HW,
    .cpuClk = 100000000u,
    .periClk = 100000000u,
    .vref = -1,
    .vdda = 3300u,
    .calibrInterval = 30u,
    .range = CY_CSDADC_RANGE_VDDA,
    .resolution = CY_CSDADC_RESOLUTION_10BIT,
    .periDivTyp = CY_SYSCLK_DIV_8_BIT,
    .numChannels = 1u,
    .idac = 31u,
    .operClkDivider = 2u,
    .azTime = 5u,
    .acqTime = 10u,
    .csdInitTime = 25u,
    .idacCalibrationEn = 0u,
    .periDivInd = 0u,
    .csdCxtPtr = &cy_csd_0_context,
};
const cy_stc_csdidac_config_t CYBSP_CSD_csdidac_config = 
{
    .base = CYBSP_CSD_HW,
    .csdCxtPtr = &cy_csd_0_context,
    .configA = CY_CSDIDAC_GPIO,
    .configB = CY_CSDIDAC_JOIN,
    .ptrPinA = (const cy_stc_csdidac_pin_t *) &CYBSP_CSD_csdidac_a_pin,
    .ptrPinB = NULL,
    .cpuClk = 100000000u,
    .csdInitTime = 25u,
};
const cy_stc_tcpwm_counter_config_t tcpwm_1_cnt_0_config = 
{
    .period = 125,
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_DOWN,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE,
    .compare0 = 0,
    .compare1 = 16384,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_ON_CC,
    .captureInputMode = tcpwm_1_cnt_0_INPUT_DISABLED & 0x3U,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = tcpwm_1_cnt_0_INPUT_DISABLED & 0x3U,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = tcpwm_1_cnt_0_INPUT_DISABLED & 0x3U,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = tcpwm_1_cnt_0_INPUT_DISABLED & 0x3U,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = tcpwm_1_cnt_0_INPUT_DISABLED & 0x3U,
    .countInput = CY_TCPWM_INPUT_1,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t tcpwm_1_cnt_0_obj = 
    {
        .type = CYHAL_RSC_TCPWM,
        .block_num = 1U,
        .channel_num = 0U,
    };
#endif //defined (CY_USING_HAL)


void init_cycfg_peripherals(void)
{
    Cy_SysClk_PeriphAssignDivider(PCLK_CSD_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);

    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM1_CLOCKS0, CY_SYSCLK_DIV_16_BIT, 0U);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&tcpwm_1_cnt_0_obj);
#endif //defined (CY_USING_HAL)
}
