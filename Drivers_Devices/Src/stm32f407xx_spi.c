/*
 * stm32f407xx_spi.c
 *
 *  Created on: Sep 17, 2025
 *      Author: Anis
 */


#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"
#include <stdint.h>


void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	 if (EnOrDi == ENABLE) {
	        if (pSPIx == SPI1) {
	            RCC->APB2ENR |= (1 << 12);   /* SPI1EN = bit12 */
	        } else if (pSPIx == SPI2) {
	            RCC->APB1ENR |= (1 << 14);   /* SPI2EN = bit14 */
	        } else if (pSPIx == SPI3) {
	            RCC->APB1ENR |= (1 << 15);   /* SPI3EN = bit15 */
	        }
	    } else {
	        if (pSPIx == SPI1) {
	            RCC->APB2ENR &= ~(1 << 12);
	        } else if (pSPIx == SPI2) {
	            RCC->APB1ENR &= ~(1 << 14);
	        } else if (pSPIx == SPI3) {
	            RCC->APB1ENR &= ~(1 << 15);
	        }
	    }
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

//	1. Configure device mode (MSTR)
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2); 		/* SPI_CR1_MSTR is bit 2 */

//	2. Configure bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg |= (1 << 10);									 	/* RXONLY = bit10 in CR1*/
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempreg |= (1 << 15);										/* Set BIDIMODE bit15 */
	}

//	3. Configure SCLK speed (BR[2:0] BITS 5:3)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

//	4. Configure DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

//	5. Configure SSM
	 tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);
	 tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);

//	6. Configure SSM (software slave management) bit 9
	 tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << 9);

//	Write to CR1
	 pSPIHandle->pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_TypeDef *pSPIx)
{
    if (pSPIx == SPI1) {
        RCC->APB2RSTR |= (1 << 12);
        RCC->APB2RSTR &= ~(1 << 12);
    } else if (pSPIx == SPI2) {
        RCC->APB1RSTR |= (1 << 14);
        RCC->APB1RSTR &= ~(1 << 14);
    } else if (pSPIx == SPI3) {
        RCC->APB1RSTR |= (1 << 15);
        RCC->APB1RSTR &= ~(1 << 15);
    }
}


void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* Wait until Tx Buffer is Empty */
		while(! (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE)));

		/* Check DFF : 16 bit or 8 bit */
		if((pSPIx->CR1 & (1 << 11)))
		{
			/* write 16 bits to DR */
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len -= 2;
			(uint16_t*) pTxBuffer++;
		}
		else
		{
			/* 8 bit */
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

	/* Wait until BSY flag is reset -> transfer complete */
	while(SPI_GetFlagStatus(pSPIx, SPI_SR_BSY));
}

void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* wait until RXNE = 1 */
		while (! (SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE)));

		/* Check DFF : 16 bit or 8 bit */
		if((pSPIx->CR1 & (1 << 11)))
		{
			/* read from DR */
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;
			(uint16_t*) pRxBuffer++;
		}
		else
		{
			/* 8 bit */
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TXState;

	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TXLen = Len;
		pSPIHandle->TXState = SPI_BUSY_IN_TX;

		/* Enable TXEIE control bit; CR2 TXEIE = bit7 */
		pSPIHandle->pSPIx->CR2 |= (1<<7);
		return 0;
	}
	return 1;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RXState;

	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RXLen = Len;
		pSPIHandle->RXState = SPI_BUSY_IN_RX;

		/* Enable TXEIE control bit; CR2 RXNE = bit6 */
		pSPIHandle->pSPIx->CR2 |= (1<<6);
		return 0;
	}
	return 1;
}

void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= SPI_CR1_SPE;			//ENABLEING SPI
	}
	else
	{
		pSPIx->CR1 &= ~SPI_CR1_SPE;			//DISABLING SPI
	}
}

uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return 1;
	}
	return 0;
}

void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= SPI_CR1_SSI;
	}
	else
	{
		pSPIx->CR1 &= ~SPI_CR1_SSI;
	}
}
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= SPI_CR2_SSOE;
	}
	else
	{
		pSPIx->CR2 &= ~SPI_CR2_SSOE;
	}
}

void SPI_CloseTransmitter(SPI_Handle_t *pHandle)
{
    /* Clear TXEIE */
    pHandle->pSPIx->CR2 &= ~(1 << 7);
    pHandle->pTxBuffer = 0;
    pHandle->TXLen = 0;
    pHandle->TXState = SPI_READY;
}

void SPI_CloseReceiver(SPI_Handle_t *pHandle)
{
    /* Clear RXNEIE */
    pHandle->pSPIx->CR2 &= ~(1 << 6);
    pHandle->pRxBuffer = 0;
    pHandle->RXLen = 0;
    pHandle->RXState = SPI_READY;
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber <= 63) {
            *NVIC_ISER1 |= (1 << (IRQNumber - 32));
        } else if (IRQNumber <= 95) {
            *NVIC_ISER2 |= (1 << (IRQNumber - 64));
        }
    } else {
        if (IRQNumber <= 31) {
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber <= 63) {
            *NVIC_ICER1 |= (1 << (IRQNumber - 32));
        } else if (IRQNumber <= 95) {
            *NVIC_ICER2 |= (1 << (IRQNumber - 64));
        }
    }
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* NVIC set priority: each IPR is 8-bit; PR_BASE is pointer in header */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *((uint32_t*) (NVIC_PR_BASE_ADDR + iprx)) &= ~(0xFF << (8 * iprx_section));
    *((uint32_t*) (NVIC_PR_BASE_ADDR + iprx)) |= (IRQPriority << shift_amount);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint32_t temp1, temp2;

	/* check for TXE */
	temp1 = pHandle->pSPIx->SR & SPI_SR_TXE;
	temp2 = pHandle->pSPIx->CR2 & (1<<7); 		/* TXEIE */

	if(temp1 && temp2)
	{
		/* Handle TXE: send next data */
		if(pHandle->TXLen > 0)
		{
			if(pHandle->pSPIx->CR1 & (1<<11))
			{
				//16 bit
				pHandle->pSPIx->DR = *((uint16_t*) pHandle->pTxBuffer);
				pHandle->TXLen -= 2;
				(uint16_t*)pHandle->pTxBuffer;
			}
			else
			{
				pHandle->pSPIx->DR = *pHandle->pTxBuffer;
				pHandle->TXLen--;
				pHandle->pTxBuffer++;
			}
		}
		if(pHandle->TXLen == 0)
		{
			/* Tx Complete : clear TXEIE and notify app */
			SPI_CloseTransmitter(pHandle);
			SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
		}
	}

    /* Check for RXNE */
    temp1 = pHandle->pSPIx->SR & SPI_SR_RXNE;
    temp2 = pHandle->pSPIx->CR2 & (1 << 6); /* RXNEIE */
    if (temp1 && temp2) {
        if (pHandle->RXLen > 0) {
            if (pHandle->pSPIx->CR1 & (1 << 11)) { /* 16-bit */
                *((uint16_t*)pHandle->pRxBuffer) = (uint16_t) pHandle->pSPIx->DR;
                pHandle->RXLen -= 2;
                (uint16_t*)pHandle->pRxBuffer++;
            } else {
                *pHandle->pRxBuffer = (uint8_t) pHandle->pSPIx->DR;
                pHandle->RXLen--;
                pHandle->pRxBuffer++;
            }
        }
        if (pHandle->RXLen == 0) {
            SPI_CloseReceiver(pHandle);
            SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
        }
    }

    /* Check for Overrun (OVR) */
    temp1 = pHandle->pSPIx->SR & SPI_SR_OVR;
    temp2 = pHandle->pSPIx->CR2 & (1 << 5); /* ERRIE maybe; but we just detect OVR */
    if (temp1) {
        /* Clear OVR by reading DR and SR (per reference manual) */
        volatile uint32_t temp;
        temp = pHandle->pSPIx->DR;
        temp = pHandle->pSPIx->SR;
        (void)temp;
        SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
    }
}


/* Weak callback; user can override in application */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t AppEv)
{
    /* weak: user implements */
}
