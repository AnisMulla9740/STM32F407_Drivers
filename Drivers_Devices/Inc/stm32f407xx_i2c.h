/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Anis
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"
#include <stdint.h>

#define ENABLE						1
#define DISABLE						0

/* Return states for I2C_GetFlagStatus */
#define SET    1
#define RESET  0

/* I2C flags (SR1) */
#define I2C_FLAG_SB     			(1 << 0)
#define I2C_FLAG_ADDR   			(1 << 1)
#define I2C_FLAG_BTF    			(1 << 2)
#define I2C_FLAG_ADD10  			(1 << 3)
#define I2C_FLAG_STOPF  			(1 << 4)
#define I2C_FLAG_RXNE   			(1 << 6)
#define I2C_FLAG_TXE    			(1 << 7)
#define I2C_FLAG_BERR   			(1 << 8)
#define I2C_FLAG_ARLO   			(1 << 9)
#define I2C_FLAG_AF     			(1 << 10)
#define I2C_FLAG_OVR    			(1 << 11)
#define I2C_FLAG_TIMEOUT 			(1 << 14)

/* I2C CR2 interrupt bits (positions) */
#define I2C_CR2_ITERREN   			(1 << 8)
#define I2C_CR2_ITEVTEN   			(1 << 9)
#define I2C_CR2_ITBUFEN   			(1 << 10)

/* I2C flags (SR2) */
#define I2C_FLAG_BUSY      			(1 << 1)

/* configuration structure for I2C Peripheral */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint32_t I2C_DeviceAddress;
	uint32_t I2C_AckControl;
	uint32_t I2C_FMDutyCycle;
}I2C_Config_t;

/* Handle structure for I2C peripherals */
typedef struct
{
	I2C_TypeDef *pI2Cx;				/*!< holds base address of I2Cx (I2C1, I2C2, I2C3) */
	I2C_Config_t *pI2C_Config;		/*!< holds I2C configuration settings */
	uint8_t *pTxBuffer;				/*!< application Tx buffer */
	uint8_t *pRxBuffer;				/*!< application Rx buffer */
	uint32_t TXLen;					/*!< Tx length */
	uint32_t RXLen;					/*!< Rx length */
	uint8_t TXRXState;				/*!< communication state */
	uint8_t DevAddr;				/*!< slave address */
	uint8_t RXSize;					/*!< Rx size (for repeated start) */
	uint8_t Sr;						/*!< repeated start value */
}I2C_Handle_t;

/* I2C Possible SCL Speed */
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM2K			200000
#define I2C_SCL_SPEED_FM4K			400000

/* ACK Control */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/* FM Duty Cycle */
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

/* I2C application states */
#define I2C_READY					0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2

/* I2C application events */
#define I2C_EV_TX_CMPLT				0
#define I2C_EV_RX_CMPLT				1
#define I2C_EV_STOP					2
#define I2C_ERROR_AF				3
#define I2C_ERROR_ARLO				4
#define I2C_ERROR_BERR				5
#define I2C_ERROR_OVR				6
#define I2C_ERROR_TIMEOUT			7

/* Application events for slave */
#define I2C_EV_DATA_REQ     		8   	/* Master wants data -> Slave should send */
#define I2C_EV_DATA_RCV      		9   	/* Master has sent data -> Slave should read */
#define I2C_EV_SLAVE_STOP    		10  	/* Master ended communication with STOP */
#define I2C_EV_SLAVE_ADDR    		11  	/* Address matched */

/* Clock control */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi);

/* Init and Deinit */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);

/* Blocking APIs (master mode) */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/* Non-blocking (IT) APIs */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/* Blocking APIs (slave mode) */
void I2C_SlaveSendData(I2C_TypeDef *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_TypeDef *pI2Cx);

/* IRQ-based Slave control (enable/disable callbacks) */
void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef *pI2Cx, uint8_t EnOrDi);

/* IRQ APIs */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/* Other peripheral controls */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_TypeDef *pI2Cx, uint8_t EnOrDi);

/* Utility to get runtime APB1 clock (used for CCR/TRISE). You can replace/extend for full PLL support. */
uint32_t RCC_GetPCLK1Value(void);

/* Bus recovery */
void I2C_BusRecovery(I2C_Handle_t *pI2CHandle);

/* Application callback */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_H_ */
