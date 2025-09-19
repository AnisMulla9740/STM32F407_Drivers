/*
 * stm32f407xx_usart.c
 *
 *  Created on: Sep 19, 2025
 *      Author: Anis
 */

#include "stm32f407xx_usart.h"
#include "stm32f407xx.h"
#include <stdint.h>

/* Compute BRR register (over-sampling by 16 assumed unless OVER8 set) */
static uint32_t USART_ComputeBRR(USART_TypeDef *pUSARTx, uint32_t pclk, uint32_t baud)
{
    uint32_t over8 = (pUSARTx->CR1 & USART_CR1_OVER8) ? 1U : 0U;
    uint32_t usartdiv_times = over8 ? 8U : 16U;

    uint32_t mantissa = (pclk) / (usartdiv_times * baud);
    uint32_t fraction = ((pclk) % (usartdiv_times * baud)) * 16U / (usartdiv_times * baud);

    return ((mantissa << 4) | (fraction & 0xF));
}


/* Get APB clock appropriate for USART instance */
static uint32_t USART_GetPCLK(USART_TypeDef *pUSARTx)
{
    /* This function approximates RCC_GetPCLK1/PCLK2 values from RCC->CFGR. */
    uint32_t sysclk;
    uint32_t sws = (RCC->CFGR >> 2) & 0x3;
    if (sws == 0) sysclk = 16000000U;       /* HSI */
    else if (sws == 1) sysclk = 8000000U;   /* HSE */
    else sysclk = 168000000U;               /* PLL */

    /* AHB prescaler */
    uint32_t hpre = (RCC->CFGR >> 4) & 0xF;
    uint32_t ahb_div = 1;
    if (hpre & 0x8) {
        const uint16_t ahb_table[] = {2,4,8,16,64,128,256,512};
        ahb_div = ahb_table[hpre & 0x7];
    }
    uint32_t hclk = sysclk / ahb_div;

    /* APB1 prescaler */
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7;
    uint32_t apb1_div = 1;
    if (ppre1 & 0x4) {
        const uint16_t apb1_table[] = {2,4,8,16};
        apb1_div = apb1_table[ppre1 & 0x3];
    }
    uint32_t pclk1 = hclk / apb1_div;

    /* APB2 prescaler */
    uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7;
    uint32_t apb2_div = 1;
    if (ppre2 & 0x4) {
        const uint8_t apb2_table[] = {2,4,8,16};
        apb2_div = apb2_table[ppre2 & 0x3];
    }
    uint32_t pclk2 = hclk / apb2_div;

    if (pUSARTx == USART1 || pUSARTx == USART6)
        return pclk2;
    else
        return pclk1;
}



void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pUSARTx == USART1) RCC->APB2ENR |= (1 << 4);   /* USART1EN */
        else if (pUSARTx == USART6) RCC->APB2ENR |= (1 << 5); /* USART6EN */
        else if (pUSARTx == USART2) RCC->APB1ENR |= (1 << 17); /* USART2EN */
        else if (pUSARTx == USART3) RCC->APB1ENR |= (1 << 18); /* USART3EN */
        else if (pUSARTx == UART4)  RCC->APB1ENR |= (1 << 19);
        else if (pUSARTx == UART5)  RCC->APB1ENR |= (1 << 20);
    }
    else
    {
        if (pUSARTx == USART1) RCC->APB2ENR &= ~(1 << 4);
        else if (pUSARTx == USART6) RCC->APB2ENR &= ~(1 << 5);
        else if (pUSARTx == USART2) RCC->APB1ENR &= ~(1 << 17);
        else if (pUSARTx == USART3) RCC->APB1ENR &= ~(1 << 18);
        else if (pUSARTx == UART4)  RCC->APB1ENR &= ~(1 << 19);
        else if (pUSARTx == UART5)  RCC->APB1ENR &= ~(1 << 20);
    }
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	//1. Enable clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//2. Configure CR1: word length, parity, TE/RE
	tempreg = 0;
	if(pUSARTHandle->pConfig->USART_WordLength == USART_WORDLEN_9BITS)
	{
		tempreg |= USART_CR1_M;
	}

	if(pUSARTHandle->pConfig->USART_Parity == USART_PARITY_EN_EVEN)
	{
		tempreg |= USART_CR1_PCE;
	}
	else if(pUSARTHandle->pConfig->USART_Parity == USART_PARITY_EN_ODD)
	{
		tempreg |= (USART_CR1_PCE | USART_CR1_PS);
	}

	if(pUSARTHandle->pConfig->USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= USART_CR1_TE;
	}
	else if(pUSARTHandle->pConfig->USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= USART_CR1_RE;
	}
	else {
		tempreg |= (USART_CR1_TE | USART_CR1_RE);
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	//3. Configure CR2: stop bits
	pUSARTHandle->pUSARTx->CR2 &= ~(0x3000);	/* clear stop bits*/
	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->pConfig->USART_StopBits << 12);

	//4. Configure CR3: flow control
	tempreg = 0;
	if(pUSARTHandle->pConfig->USART_HWFlowCtrl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= USART_CR3_RTSE;
	}
	else if(pUSARTHandle->pConfig->USART_HWFlowCtrl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= USART_CR3_CTSE;
	}
	else if(pUSARTHandle->pConfig->USART_HWFlowCtrl == USART_HW_FLOW_CTRL_RTS_CTS) {
		tempreg |= (USART_CR3_RTSE | USART_CR3_CTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	//5. Configure BRR
	uint32_t pclk = USART_GetPCLK(pUSARTHandle->pUSARTx);
	pUSARTHandle->pUSARTx->BRR = USART_ComputeBRR(pUSARTHandle->pUSARTx, pclk, pUSARTHandle->pConfig->USART_Baud);

	//6. EN USART
	pUSARTHandle->pUSARTx->CR1 |= USART_CR1_UE;
}


void USART_DeInit(USART_TypeDef *pUSARTx)
{
    if (pUSARTx == USART1)
    {
        RCC->APB2RSTR |= (1 << 4);
        RCC->APB2RSTR &= ~(1 << 4);
    }
    else if (pUSARTx == USART2)
    {
        RCC->APB1RSTR |= (1 << 17);
        RCC->APB1RSTR &= ~(1 << 17);
    }
    else if (pUSARTx == USART3)
    {
        RCC->APB1RSTR |= (1 << 18);
        RCC->APB1RSTR &= ~(1 << 18);
    }
    /* Add resets for UART4, UART5, USART6 as needed */
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	while(Len > 0)
	{
		/* wait until TXE = 1 */
		while(!(pUSARTHandle->pUSARTx->SR & USART_SR_TXE));

		/* check the word length */
		if(pUSARTHandle->pConfig->USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & 0x1FF);

			if(pUSARTHandle->pConfig->USART_Parity == USART_PARITY_DISABLE)
			{
				pTxBuffer += 2;
				Len -= 2;
			}
			else
			{
				pTxBuffer += 1;
				Len -= 1;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & 0xFF);
			pTxBuffer++;
			Len--;
		}
	}

	 /* wait for TC flag before returning */
	while(!(pUSARTHandle->pUSARTx->SR & USART_SR_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint16_t *pdata;

    while (Len > 0)
    {
    	/*wait for RXNE = 1*/
    	while(!(pUSARTHandle->pUSARTx->SR & USART_SR_RXNE));

    	if(pUSARTHandle->pConfig->USART_WordLength == USART_WORDLEN_9BITS)
    	{
    		if(pUSARTHandle->pConfig->USART_Parity == USART_PARITY_DISABLE)
    		{
    			pdata = (uint16_t*)pRxBuffer;
    			*pdata = (uint16_t)(pUSARTHandle->pUSARTx->DR & 0x1FF);
    			pRxBuffer += 2;
    			Len -= 2;
    		}
    		else
    		{
    			/* Parity used -> only 8 bits of data delivered */
    			*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
    			pRxBuffer++;
    			Len--;
    		}
    	}
        else
        {
            if (pUSARTHandle->pConfig->USART_Parity == USART_PARITY_DISABLE)
            {
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
            }
            else
            {
                /* parity enabled: 7 data bits in DR */
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
            }
            pRxBuffer++;
            Len--;
        }
    }
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	if(pUSARTHandle->TxBusyState |= USART_BUSY_IN_TX)
	{
		/* save buffer and length in handle */
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		/* enable TXEIE control bit to get interrupt when TXE=1 */
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TXEIE;
		return 1;
	}
	return 0;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	if(pUSARTHandle->TxBusyState |= USART_BUSY_IN_RX)
	{
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        /* enable RXNEIE */
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RXNEIE;
        return 1;
	}
	return 0;
}

void USART_IRQHandling(USART_Handle_t *pHandle)
{
	uint32_t sr = pHandle->pUSARTx->SR;
	uint32_t cr1 = pHandle->pUSARTx->CR1;

	/* 1. TXE event */
	if((sr & USART_SR_TXE) && (cr1 && USART_CR1_TXEIE))
	{
		if(pHandle->TxLen > 0)
		{
			if (pHandle->pConfig->USART_WordLength == USART_WORDLEN_9BITS)
			{
				uint16_t *pdata = (uint16_t*)pHandle->pTxBuffer;
                pHandle->pUSARTx->DR = (*pdata & 0x1FF);

                if (pHandle->pConfig->USART_Parity == USART_PARITY_DISABLE)
                {
                    pHandle->pTxBuffer += 2;
                    pHandle->TxLen -= 2;
                }
                else
                {
                    pHandle->pTxBuffer += 1;
                    pHandle->TxLen -= 1;
                }
			}
            else
            {
                pHandle->pUSARTx->DR = (*pHandle->pTxBuffer & 0xFF);
                pHandle->pTxBuffer++;
                pHandle->TxLen--;
            }
		}
        if (pHandle->TxLen == 0)
        {
            /* TX completed, disable TXEIE, enable TCIE to get final completion */
            pHandle->pUSARTx->CR1 &= ~USART_CR1_TXEIE;
            pHandle->pUSARTx->CR1 |= USART_CR1_TCIE;
        }
	}

	/* 2. TC event */
	if((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE))
	{
		/* clear TCIE and inform application */
		pHandle->pUSARTx->CR1 &= ~(USART_CR1_TCIE);
		pHandle->TxBusyState = USART_READY;
		USART_ApplicationEventCallback(pHandle, USART_EVENT_TX_CMPLT);
	}

    /* 3. RXNE event */
    if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE))
    {
        if (pHandle->RxLen > 0)
        {
            if (pHandle->pConfig->USART_WordLength == USART_WORDLEN_9BITS)
            {
                if (pHandle->pConfig->USART_Parity == USART_PARITY_DISABLE)
                {
                    *((uint16_t*)pHandle->pRxBuffer) = (pHandle->pUSARTx->DR & 0x1FF);
                    pHandle->pRxBuffer += 2;
                    pHandle->RxLen -= 2;
                }
                else
                {
                    *pHandle->pRxBuffer = (uint8_t)(pHandle->pUSARTx->DR & 0xFF);
                    pHandle->pRxBuffer++;
                    pHandle->RxLen -= 1;
                }
            }
            else
            {
                *pHandle->pRxBuffer = (uint8_t)(pHandle->pUSARTx->DR & 0xFF);
                pHandle->pRxBuffer++;
                pHandle->RxLen--;
            }
        }

        if (pHandle->RxLen == 0)
        {
            /* reception complete */
            pHandle->pUSARTx->CR1 &= ~USART_CR1_RXNEIE;
            pHandle->RxBusyState = USART_READY;
            USART_ApplicationEventCallback(pHandle, USART_EVENT_RX_CMPLT);
        }
    }

    /* 4. Error handling */
    if (sr & USART_SR_ORE)
        USART_ApplicationEventCallback(pHandle, USART_EVENT_ORE);
    if (sr & USART_SR_FE)
        USART_ApplicationEventCallback(pHandle, USART_EVENT_FE);
    if (sr & USART_SR_PE)
        USART_ApplicationEventCallback(pHandle, USART_EVENT_PE);
}

/* Configure IRQ (enable/disable). IRQNumber = device IRQn number. */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        uint32_t iser_index = IRQNumber / 32;
        uint32_t iser_pos = IRQNumber % 32;
        NVIC_ISER0[iser_index] = (1U << iser_pos);
    }
    else
    {
        uint32_t icer_index = IRQNumber / 32;
        uint32_t icer_pos = IRQNumber % 32;
        NVIC_ICER0[icer_index] = (1U << icer_pos);
    }
}

/* Configure IRQ priority */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    NVIC_PR_BASE_ADDR[iprx] |= (IRQPriority << shift_amount);
}

void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
        pUSARTx->CR1 |= USART_CR1_UE;
    else
        pUSARTx->CR1 &= ~USART_CR1_UE;
}

uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName)
{
    return (pUSARTx->SR & FlagName) ? 1 : 0;
}

void USART_ClearFlag(USART_TypeDef *pUSARTx, uint32_t FlagName)
{
    /* Generic: many flags are cleared by reading SR followed by read/write of DR.
       For ORE: read SR then DR clears it. This helper injects the sequence for ORE. */
    volatile uint32_t tmp;
    if (FlagName == USART_SR_ORE)
    {
        tmp = pUSARTx->SR;
        (void)tmp;
        tmp = pUSARTx->DR;
        (void)tmp;
    }
    else
    {
        ;
    }
}




/* Weak application callback - user implements this in their application to get events */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pHandle, uint8_t AppEv)
{
    /* Default weak implementation. Override in user application. */
}






