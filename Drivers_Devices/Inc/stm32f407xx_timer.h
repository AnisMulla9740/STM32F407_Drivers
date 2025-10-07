/*
 * stm32f407xx_timer.h
 *
 *  Created on: Oct 6, 2025
 *      Author: Anis
 */

#ifndef INC_STM32F407XX_TIMER_H_
#define INC_STM32F407XX_TIMER_H_

#include "stm32f407xx.h"
#include <stdint.h>

#define ENABLE							1
#define DISABLE							0

/* Timer Counter Modes */
#define TIMER_COUNTER_UP				0
#define TIMER_COUNTER_DOWN				1
#define TIMER_COUNTER_CENTER1			2
#define TIMER_COUNTER_CENTER2			3
#define TIMER_COUNTER_CENTER3			4

/* Auto-relaoad preload */
#define TIMER_AUTORELOAD_PRELOAD_ENABLE		ENABLE
#define TIMER_AUTORELOAD_PRELOAD_DISABLE	DISABLE

/* update request source */
#define TIMER_URS_ANY_EVENT				0
#define TIMER_URS_ONLY_OVERFLOW			1

/* Capture/Compare Channels */
#define TIMER_CHANNEL_1					1
#define TIMER_CHANNEL_2					2
#define TIMER_CHANNEL_3					3
#define TIMER_CHANNEL_4					4

/* Output Compare Modes (for PWM) */
#define TIMER_OCMODE_FROZEN				0
#define TIMER_OCMODE_ACTIVE_ON_MATCH	1
#define TIMER_OCMODE_INACTIVE_ON_MATCH	2
#define TIMER_OCMODE_TOGGLE				3
#define TIMER_OCMODE_FORCE_INACTIVE		4
#define TIMER_OCMODE_FORCE_ACTIVE		5
#define TIMER_OCMODE_PWM1				6
#define TIMER_OCMODE_PWM2				7

/* Output Polarity */
#define TIMER_OC_POLARITY_HIGH			0
#define TIMER_OC_POLARITY_LOW			1

/* Capture Polarity */
#define TIMER_IC_POLARITY_RISING		0
#define TIMER_IC_POLARITY_FALLING		1
#define TIMER_IC_POLARITY_BOTH			2

/* Capture Prescalers (ICPSC) */
#define TIMER_ICPSC_DIV1				0
#define TIMER_ICPSC_DIV2				1
#define TIMER_ICPSC_DIV4				2
#define TIMER_ICPSC_DIV8				3

/* Interrupt event masks (for callbacks) */
#define TIMER_EVENT_UPDATE				(1U << 0)
#define TIMER_EVENT_CC1					(1U << 1)
#define TIMER_EVENT_CC2					(1U << 2)
#define TIMER_EVENT_CC3					(1U << 3)
#define TIMER_EVENT_CC4					(1U << 4)

/* PWM helper constraints */
#define TIMER_MAX_PSC					0xFFFFU
#define TIMER_MAX_ARR					0xFFFFU

/* Application callback type; 'evennt' uses above masks; 'channel' for CC events (1,...,4) */
typedef void (*TIMER_AppCallback_t)(uint32_t event, uint8_t channel, uint32_t captureValue);

/* Basic timer configuration */
typedef struct {
	uint32_t Prescaler;
	uint32_t Period;
	uint8_t  CounterMode;
	uint8_t  AutoReloadPreload;
	uint8_t  OnePulseMode;
	uint8_t  URS;
}TIMER_Config_t;

/* PWM Channel configurations */
typedef struct {
	uint8_t OCMode;
	uint8_t OCPolarity;
	uint8_t OCIdleState;
	uint8_t PreloadEnable;
}TIMER_PWMConfig_t;

/* Input Capture Configurations */
typedef struct {
	uint8_t ICPolarity;
	uint8_t ICSelection;
	uint8_t ICPrescaler;
	uint8_t ICFilter;
}TIMER_ICConfig_t;

/* Handle for Timer */
typedef struct {
	TIM_TypeDef *pTIMx;
	TIMER_Config_t *pTimConfig;
	TIMER_AppCallback_t pAppCallback;
}TIMER_Handle_t;

/* Peripheral clock control (enable/disable) */
void TIMER_PeriClockControl(TIM_TypeDef *pTIMx, uint8_t EnOrDi);

/* Init / DeInit */
void TIMER_Init(TIMER_Handle_t *pTIMHandle);
void TIMER_DeInit(TIM_TypeDef *pTIMx);

/* Counter control */
void TIMER_Start(TIMER_Handle_t *pTIMHandle);
void TIMER_Stop(TIMER_Handle_t *pTIMHandle);
void TIMER_ResetCounter(TIM_TypeDef *pTIMx);

/* ARR / PSC helpers (safe writes) */
void TIMER_SetPrescaler(TIM_TypeDef *pTIMx, uint32_t psc);
void TIMER_SetAutoReload(TIM_TypeDef *pTIMx, uint32_t arr);
uint32_t TIMER_GetCounter(TIM_TypeDef *pTIMx);

/* Interrupt (NVIC) config helpers */
void TIMER_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void TIMER_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/* Per-timer interrupt enable/disable (in DIER) */
void TIMER_EnableIt_Update(TIM_TypeDef *pTIMx, uint8_t EnOrDi);
void TIMER_EnableIt_CC(TIM_TypeDef *pTIMx, uint8_t channel, uint8_t EnOrDi);

/* Flag helpers */
uint8_t TIMER_GetFlagStatus(TIM_TypeDef *pTIMx, uint32_t flagMask);
void TIMER_ClearFlag(TIM_TypeDef *pTIMx, uint32_t flagMask);

/* Unified IRQ handler to be called inside ISR for a given TIMx handle */
void TIMER_IRQHandling(TIMER_Handle_t *pTIMHandle);

/* Callback registration */
void TIMER_RegisterCallback(TIMER_Handle_t *pTIMHandle, TIMER_AppCallback_t pCallback);

/* ========== PWM / Output Compare APIs ========== */
/* Initialize channel for PWM / OC */
void TIMER_PWMInit(TIM_TypeDef *pTIMx, uint8_t channel, TIMER_PWMConfig_t *pwmConfig);

/* Set duty in percentage (0..100). Uses current ARR to compute CCRx. */
void TIMER_SetPWMDutyCycle(TIM_TypeDef *pTIMx, uint8_t channel, float duty_percent);

/* Start PWM on channel (enables capture/compare output and counter if needed) */
void TIMER_PWMStart(TIMER_Handle_t *pTIMHandle, uint8_t channel);

/* Stop PWM on channel */
void TIMER_PWMStop(TIMER_Handle_t *pTIMHandle, uint8_t channel);

/* Change PWM frequency (by updating PSC and ARR). Duty persists as CCRx/ARR ratio */
void TIMER_ChangePWMFrequency(TIM_TypeDef *pTIMx, uint32_t newPSC, uint32_t newARR);

/* ========== Input Capture APIs ========== */
/* Initialize input-capture on a channel */
void TIMER_ICInit(TIM_TypeDef *pTIMx, uint8_t channel, TIMER_ICConfig_t *icConfig);

/* Read captured value (CCR1..CCR4) */
uint32_t TIMER_ICReadCapture(TIM_TypeDef *pTIMx, uint8_t channel);

/* Clear capture flag for channel */
void TIMER_ICClearFlag(TIM_TypeDef *pTIMx, uint8_t channel);

#endif /* INC_STM32F407XX_TIMER_H_ */
