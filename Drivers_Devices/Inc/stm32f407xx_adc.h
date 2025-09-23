/*
 * stm32f407xx_adc.h
 *
 *  Created on: Sep 22, 2025
 *      Author: Anis
 */

#ifndef INC_STM32F407XX_ADC_H_
#define INC_STM32F407XX_ADC_H_

#include "stm32f407xx.h"
#include <stdint.h>

/* ADC operation macros */
#define ENABLE						1
#define DISABLE						0

/* ADC resolution */
#define ADC_RES_12BITS				0
#define ADC_RES_10BITS				1
#define ADC_RES_8BITS				2
#define ADC_RES_6BITS				3

/* Data Alignment */
#define ADC_ALIGN_RIGHT				0
#define ADC_ALIGN_LEFT				1

/* Conversion modes */
#define ADC_CONTINUOUS_DISABLE		0
#define ADC_CONTINUOUS_ENABLE		1
#define ADC_EOC_AFTER_SEQ			1

/* EOC selection */
#define ADC_EOC_AFTER_EACH_CONV		0
#define ADC_EOC_AFTER_SEQ			1

/* ADC sample times (mapped to SMPRx encodings) - common values */
#define ADC_SAMPLETIME_3CYCLES		0
#define ADC_SAMPLETIME_15CYCLES		1
#define ADC_SAMPLETIME_28CYCLES		2
#define ADC_SAMPLETIME_56CYCLES		3
#define ADC_SAMPLETIME_84CYCLES		4
#define ADC_SAMPLETIME_112CYCLES	5
#define ADC_SAMPLETIME_144CYCLES	6
#define ADC_SAMPLETIME_480CYCLES	7

/* ADC event macross for callback */
#define ADC_EVENT_EOC				1
#define ADC_EVENT_OVR				2

/* API status codes */
#define ADC_OK						0
#define ADC_ERR						-1

typedef void (*ADC_AppCallback_t)(uint8_t event);

/* ADC configuration structure */
typedef struct
{
	uint8_t ADC_Resoultion;
	uint8_t ADC_ContinuousConv;
	uint8_t ADC_DataAlign;
	uint8_t ADC_EOCSelection;
}ADC_Config_t;

/* ADC handle */
typedef struct
{
	ADC_TypeDef *pADCx;
	ADC_Config_t *pADCConfig;
	ADC_AppCallback_t pAppCallback;
}ADC_Handle_t;


/* Peripheral clock control */
void ADC_PeriClockControl(ADC_TypeDef *pADCx, uint8_t EnOrDi);

/* Init / DeInit */
void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(ADC_TypeDef *pADCx);

/* Blocking single conversion on a single regular channel */
uint16_t ADC_ReadSingleConversion(ADC_Handle_t *pADCHandle, uint8_t channel);

/* Start / Stop conversion (regular group) */
void ADC_StartConversion(ADC_Handle_t *pADCHandle);
void ADC_StopConversion(ADC_Handle_t *pADCHandle);

/* IRQ configuration (NVIC) & handler */
void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void ADC_IRQHandling(ADC_Handle_t *pADCHandle);

/* Utility */
void ADC_SetChannelSampleTime(ADC_TypeDef *pADCx, uint8_t channel, uint8_t sampleTime);
void ADC_SelectRegularChannel(ADC_TypeDef *pADCx, uint8_t channel, uint8_t rank);

/* API for registering callback */
void ADC_RegisterCallback(ADC_Handle_t *pADCHandle, ADC_AppCallback_t pCallback);

void ADC_ConfigRegularSequence(ADC_TypeDef *pADCx, uint8_t *channels, uint8_t length);


#endif /* INC_STM32F407XX_ADC_H_ */
