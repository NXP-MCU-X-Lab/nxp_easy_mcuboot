/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CH_LIB_CTIMER_H__
#define __CH_LIB_CTIMER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define HW_CTIMER_PWN_CH0       (0)
#define HW_CTIMER_PWN_CH1       (1)
#define HW_CTIMER_PWN_CH2       (2)
#define HW_CTIMER_PWN_CH3       (3)
     
void CTIMER_PWM_Init(uint32_t instance, uint32_t pwm_chl, uint32_t freq);
void CTIMER_PWM_SetDuty(uint32_t instance, uint32_t pwm_chl, uint32_t duty);

#ifdef __cplusplus
}
#endif

#endif

