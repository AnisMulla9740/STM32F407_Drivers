/*
 * stm32f407xx_spi.h
 *
 *  Created on: Sep 17, 2025
 *      Author: Anis
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_


#include "stm32f407xx.h"
#include <stdint.h>

#define ENABLE 						1
#define DISABLE						0


/* Configuration structure for SPI peripheral */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/* Handle structure for SPI peripheral */
typedef struct
{
	SPI_TypeDef *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TXLen;
	uint32_t RXLen;
	uint8_t TXState;
	uint8_t RXState;
}SPI_Handle_t;

/* Device Mode */
#define SPI_DEVICE_MODE_MASTER				0
#define SPI_DEVICE_MODE_SLAVE				1

/* Bus Configuration */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/* SCLK speed option or  Configuration */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/* Data Frame Format */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/* SSM - software slave management */
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/* Possible SPI application events (for IRQ callbacks) */
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4

/* Return states for IRQ mode (internal) */
#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/* API Prototypes */
/* Peripheral clock control */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnOrDi);

/* Init & De-Init */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);

/* Data send / receive - blocking */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* Data send / receive - interrupt (non-blocking); returns 0 if accepted else non-zero */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ configuration and handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/* Other peripheral controls */
void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName);
void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t EnOrDi);

/* Close/abort APIs for IRQ mode */
void SPI_CloseTransmitter(SPI_Handle_t *pHandle);
void SPI_CloseReceiver(SPI_Handle_t *pHandle);

/* Application callback (weak; user can override) */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t AppEv);



#endif /* INC_STM32F407XX_SPI_H_ */
