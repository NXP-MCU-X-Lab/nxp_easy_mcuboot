/**
  ******************************************************************************
  * @file    ctimer.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.05.31
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_CTIMER_H__
#define __CH_LIB_CTIMER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif
     
#define HW_CTIMER0      (0)
#define HW_CTIMER1      (1)
#define HW_CTIMER2      (2)
#define HW_CTIMER3      (3)
     
#define HW_CTIMER_CH0       (0)
#define HW_CTIMER_CH1       (1)
#define HW_CTIMER_CH2       (2)
#define HW_CTIMER_CH3       (3)
     
#define HW_CTIMER_CAP_INT_EVT_RE        (0)
#define HW_CTIMER_CAP_INT_EVT_FE        (1)
#define HW_CTIMER_CAP_INT_EVT_ALL       (2)


/* using CTIMER as a normal timer */
void CTIMER_TC_Init(uint32_t instance, uint32_t us);
void CTIMER_TC_SetIntMode(uint32_t instance, uint32_t chl, bool val);
void CTIMER_SetCounter(uint32_t instance, uint32_t val);
uint32_t CTIMER_GetCounter(uint32_t instance);
void CTIMER_Stop(uint32_t instance);
void CTIMER_Start(uint32_t instance);
     
/* using CTIMER as a PWM */
void CTIMER_PWM_Init(uint32_t instance, uint32_t pwm_chl, uint32_t freq);
void CTIMER_PWM_SetDuty(uint32_t instance, uint32_t pwm_chl, uint32_t duty);

/* CTIMER_CAP API */
void CTIMER_CAP_Init(uint32_t instance, uint32_t chl);
void CIMER_CAP_SetITandLoadMode(uint32_t instance, uint32_t chl, uint32_t mode);
void CTIMER_CAP_SetIntMode(uint32_t instance, uint32_t chl, bool val);
uint32_t CTIMER_GetCAPCounter(uint32_t instance, uint32_t chl);
void CTIMER_SetTimerClearEvt(uint32_t instance, uint32_t chl, uint32_t mode, bool val);
#endif


