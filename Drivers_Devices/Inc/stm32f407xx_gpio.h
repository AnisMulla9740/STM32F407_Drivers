/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Sep 10, 2025
 *      Author: Anis
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t PinSpeed;
	uint8_t PinPuPdControl;
	uint8_t PinOptType;
	uint8_t PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct {
	GPIO_TypeDef *pGPIOx;
	GPIO_PinConfig_t PinConfig;
} GPIO_Handle_t;


/*
 * GPIO PIN NUMBER
 */
#define GPIO_PIN_0						0
#define GPIO_PIN_1						1
#define GPIO_PIN_2						2
#define GPIO_PIN_3						3
#define GPIO_PIN_4						4
#define GPIO_PIN_5						5
#define GPIO_PIN_6						6
#define GPIO_PIN_7						7
#define GPIO_PIN_8						8
#define GPIO_PIN_9						9
#define GPIO_PIN_10						10
#define GPIO_PIN_11						11
#define GPIO_PIN_12						12
#define GPIO_PIN_13						13
#define GPIO_PIN_14						14
#define GPIO_PIN_15						15

/*
 * GPIO PIN MODES
 */
#define GPIO_MODE_INPUT					0
#define GPIO_MODE_OUTPUT				1
#define GPIO_MODE_ALTFN					2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4
#define GPIO_MODE_IT_RT					5
#define GPIO_MODE_IT_RFT				6

/*
 * GPIO PIN SPEEDS
 */
#define GPIO_SPEED_LOW					0
#define GPIO_SPEED_MED					1
#define GPIO_SPEED_HIGH					2
#define GPIO_SPEED_VHIGH				3

/*
 * GPIO PULL UP PULL DOWN CONTROL
 */

#define GPIO_PUPDCONTROL_NOPUPD			0
#define GPIO_PUPDCONTROL_PU				1
#define GPIO_PUPDCONTROL_PD				2

/*
 * GPIO OUTPUT TYPE REGISTER
 */
#define GPIO_OPTYPE_PP					0
#define GPIO_OPTYPE_OD					1

/*
 * GPIO ALTFN MODE REGISTERS
 */
#define GPIO_ALTFN_AF0					0
#define GPIO_ALTFN_AF1					1
#define GPIO_ALTFN_AF2					2
#define GPIO_ALTFN_AF3					3
#define GPIO_ALTFN_AF4					4
#define GPIO_ALTFN_AF5					5
#define GPIO_ALTFN_AF6					6
#define GPIO_ALTFN_AF7					7
#define GPIO_ALTFN_AF8					8
#define GPIO_ALTFN_AF9					9
#define GPIO_ALTFN_AF10					10
#define GPIO_ALTFN_AF11					11
#define GPIO_ALTFN_AF12					12
#define GPIO_ALTFN_AF13					13
#define GPIO_ALTFN_AF14 				14
#define GPIO_ALTFN_AF15      			15

/*
 * GPIO API PROTOTYPES
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnOrDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_H_ */
