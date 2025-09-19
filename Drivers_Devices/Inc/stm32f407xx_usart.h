/*
 * stm32f407xx_usart.h
 *
 *  Created on: Sep 19, 2025
 *      Author: Anis
 */

#ifndef INC_STM32F407XX_USART_H_
#define INC_STM32F407XX_USART_H_

#include "stm32f407xx.h"
#include <stdint.h>

#define ENABLE								1
#define DISABLE								0
/* USART modes */
#define USART_MODE_ONLY_TX					0
#define USART_MODE_ONLY_RX					1
#define USART_MODE_TXRX						2

/* word length */
#define USART_WORDLEN_8BITS					0
#define USART_WORDLEN_9BITS					1

/* Parity Control */
#define USART_PARITY_DISABLE				0
#define USART_PARITY_EN_EVEN				1
#define USART_PARITY_EN_ODD					2

/* Stop Bits */
#define USART_STOPBITS_1					0
#define USART_STOPBITS_0_5					1
#define USART_STOPBITS_2					2
#define USART_STOPBITS_1_5					3

/* Hardware Flow Control */
#define USART_HW_FLOW_CTRL_NONE				0
#define USART_HW_FLOW_CTRL_RTS				1
#define USART_HW_FLOW_CTRL_CTS				2
#define USART_HW_FLOW_CTRL_RTS_CTS			3

/* USART states */
#define USART_READY							0
#define USART_BUSY_IN_RX					1
#define USART_BUSY_IN_TX					2

/* Application events (for callback) */
#define USART_EVENT_TX_CMPLT				0
#define USART_EVENT_RX_CMPLT				1
#define USART_EVENT_IDLE					2
#define USART_EVENT_CTS						3
#define USART_EVENT_PE						4
#define USART_EVENT_FE						5
#define USART_EVENT_ORE						6

/*-----------------------Structures----------------------*/

/*Configuration of structure*/
typedef struct
{
	uint32_t USART_Mode;
	uint32_t USART_Baud;
	uint32_t USART_WordLength;
	uint32_t USART_StopBits;
	uint32_t USART_Parity;
	uint32_t USART_HWFlowCtrl;
}USART_Config_t;

/*Handle Structure*/
typedef struct
{
	USART_TypeDef *pUSARTx;
	USART_Config_t *pConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

/*---------------------APIs-------------------*/

/* Peripheral clock control */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi);

/* Init / Deinit */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);

/* Blocking mode */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/* Interrupt mode */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ config and handling */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/* Peripheral control */
void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi);

/* Flag utilities */
uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_TypeDef *pUSARTx, uint32_t FlagName);

/* Weak application callback (user overrides) */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pHandle, uint8_t AppEv);
#endif /* INC_STM32F407XX_USART_H_ */
