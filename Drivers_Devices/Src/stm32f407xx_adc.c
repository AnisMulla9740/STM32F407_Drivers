/*
 * stm32f407xx_adc.c
 *
 *  Created on: Sep 22, 2025
 *      Author: Anis
 */

#include "stm32f407xx_adc.h"
#include "stm32f407xx.h"

void ADC_PeriClockControl(ADC_TypeDef *pADCx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pADCx == ADC1)
            RCC->APB2ENR |= (1 << 8);   /* ADC1EN bit (APB2ENR bit8) */
        else if (pADCx == ADC2)
            RCC->APB2ENR |= (1 << 9);   /* ADC2EN bit (APB2ENR bit9) */
        else if (pADCx == ADC3)
            RCC->APB2ENR |= (1 << 10);  /* ADC3EN bit (APB2ENR bit10) */
    }
    else
    {
        if (pADCx == ADC1)
            RCC->APB2ENR &= ~(1 << 8);
        else if (pADCx == ADC2)
            RCC->APB2ENR &= ~(1 << 9);
        else if (pADCx == ADC3)
            RCC->APB2ENR &= ~(1 << 10);
    }
}

void ADC_Init(ADC_Handle_t *pADCHandle)
{
	ADC_TypeDef *pADCx = pADCHandle->pADCx;

	/* EN peripheral clock */
	ADC_PeriClockControl(pADCx, ENABLE);

	/* CR1 : resolution bits (RES[1:0]) */
	uint32_t temp = pADCx->CR1;
	temp &= ~(0x03000000U);

	switch (pADCHandle->pADCConfig->ADC_Resoultion)
	{
		case ADC_RES_12BITS	:	temp |= (0U << 24); break;
		case ADC_RES_10BITS : 	temp |= (1U << 24); break;
		case ADC_RES_8BITS	:	temp |= (2U << 24); break;
		case ADC_RES_6BITS  : 	temp |= (3U << 24); break;
		default : 				temp |= (0U << 24); break;
	}

	pADCx->CR1 = temp;

	/* CR2: Continuous conversion, align, EOC selection */
	temp = pADCx->CR2;

	/* Continuous */
	if(pADCHandle->pADCConfig->ADC_ContinuousConv == ADC_CONTINUOUS_ENABLE)
	{
		temp |= ADC_CR2_CONT;
	}
	else
	{
		temp &= ~ADC_CR2_CONT;
	}

	/* ALIGN */
	if(pADCHandle->pADCConfig->ADC_DataAlign == ADC_ALIGN_LEFT)
	{
		temp |= ADC_CR2_ALIGN;
	}
	else
	{
		temp &= ~ADC_CR2_ALIGN;
	}

	/* EOC selection */
	if(pADCHandle->pADCConfig->ADC_EOCSelection == ADC_EOC_AFTER_SEQ)
	{
		temp |= ADC_CR2_EOCS;
	}
	else
	{
		temp &= ~ADC_CR2_EOCS;
	}
	pADCx->CR2 = temp;
}

void ADC_DeInit(ADC_TypeDef *pADCx)
{
	pADCx->CR2 &= ~ADC_CR2_ADON;

    /* clear relevant control registers to default-ish */
    pADCx->CR1 = 0x0;
    pADCx->CR2 = 0x0;
    pADCx->SMPR1 = 0x0;
    pADCx->SMPR2 = 0x0;
    pADCx->SQR1 = 0x0;
    pADCx->SQR2 = 0x0;
    pADCx->SQR3 = 0x0;
}

/* Blocking read : configure channel as rank1, start, poll, eoc, dr */
uint16_t ADC_ReadSingleConversion(ADC_Handle_t *pADCHandle, uint8_t channel)
{
	ADC_TypeDef *pADCx = pADCHandle->pADCx;

	/* ensure single conversion mode */
	pADCx->CR2 &= ~ADC_CR2_CONT;

	/* Set rank1 = channel */
	ADC_SelectRegularChannel(pADCx, channel, 1);

	/* set sequence length L=0 (1 conversion) */
	pADCx->SQR1 &= ~(0xF << 20);

	/* start conversion: ADON must be sent to turn on ADC. A first set to ADON wakes ADC,
	 * second set starts conversion in some chips,
	 * but setting SWSTART after ADON normally starts conversion. We'll set ADON then SWSTART
	 */
	pADCx->CR2 |= ADC_CR2_ADON;

	for(volatile uint32_t i=0; i<500; ++i)
		{(void) i;}

	/* software start conversion */
	pADCx->CR2 |= (1U << 30);

	/* wait for EOC */
	while(!(pADCx->SR & ADC_SR_EOC));

	/* read DR */
	uint16_t value = (uint16_t) (pADCx->DR & 0xFFFFU);

	/* clear EOC by reading DR */
	(void)pADCx->DR;

	return value;
}

void ADC_StartConversion(ADC_Handle_t *pADCHandle)
{
	ADC_TypeDef *pADCx = pADCHandle->pADCx;

	/* switch ADC on (ADON = 1) */
	pADCx->CR2 |= ADC_CR2_ADON;

	/* small delay after enabling ADC is recommended */
	for (volatile uint32_t i=0; i<1000; ++i) {(void)i;}

	/* SWSTART bit */
	pADCx->CR2 |= (1U << 30);
}

/* Stop conversion */
void ADC_StopConversion(ADC_Handle_t *pADCHandle)
{
	ADC_TypeDef *pADCx = pADCHandle->pADCx;

	/* stop continuous conversions by clearing CONT bit (if enabled) */
	pADCx->CR2 &= ~ADC_CR2_CONT;

	/* optionally clear SWSTART is not writable clear, but disabling ADON will stop further conversions */
    pADCx->CR2 &= ~ADC_CR2_ADON;
}

void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31)
            *NVIC_ISER0 |= (1U << IRQNumber);
        else if (IRQNumber <= 63)
            *NVIC_ISER1 |= (1U << (IRQNumber - 32));
        else if (IRQNumber <= 95)
            *NVIC_ISER2 |= (1U << (IRQNumber - 64));
    }
    else
    {
        if (IRQNumber <= 31)
            *NVIC_ICER0 |= (1U << IRQNumber);
        else if (IRQNumber <= 63)
            *NVIC_ICER1 |= (1U << (IRQNumber - 32));
        else if (IRQNumber <= 95)
            *NVIC_ICER2 |= (1U << (IRQNumber - 64));
    }
}

void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* ADC IRQ handling - basic skeleton clears flags and reads DR if EOC */
void ADC_IRQHandling(ADC_Handle_t *pADCHandle)
{
    ADC_TypeDef *pADCx = pADCHandle->pADCx;

    uint32_t status = pADCx->SR;

    /* End of conversion */
    if (status & ADC_SR_EOC)
    {
        /* read data to clear EOC */
        volatile uint32_t data = pADCx->DR;
        (void)data;


        if (pADCHandle->pAppCallback)
            pADCHandle->pAppCallback(ADC_EVENT_EOC);
    }

    /* Handle overrun */
    if (status & ADC_SR_OVR)
    {
        /* clear OVR by reading DR then reading SR (reference RM) or reset OVR flag by some sequence */
        volatile uint32_t dummy = pADCx->DR;
        (void)dummy;
        pADCx->SR &= ~ADC_SR_OVR;

        if (pADCHandle->pAppCallback)
            pADCHandle->pAppCallback(ADC_EVENT_OVR);
    }
}

void ADC_SetChannelSampleTime(ADC_TypeDef *pADCx, uint8_t channel, uint8_t sampleTime)
{
	uint32_t temp;
	if(channel <= 9)
	{
		/* smpr2 channel x uses 3 bits at position */
		temp = pADCx->SMPR2;
		temp &= ~(0x7U << (3 * channel));
		temp |= ((sampleTime & 0x7U) << (3 * channel));
		pADCx->SMPR2 = temp;
	}
	else if (channel <= 18)
	{
        uint8_t ch = channel - 10;
        temp = pADCx->SMPR1;
        temp &= ~(0x7U << (3 * ch));
        temp |= ((sampleTime & 0x7U) << (3 * ch));
        pADCx->SMPR1 = temp;
	}
}

void ADC_SelectRegularChannel(ADC_TypeDef *pADCx, uint8_t channel, uint8_t rank)
{
    /* SQRx registers hold sequence. SQR1 L bits give sequence length and SQR1,SQR2,SQR3 hold sequence slots */
    /* We will support rank 1..16 as follows:
       ranks 1..6 -> SQR3 slots
       ranks 7..12 -> SQR2 slots
       ranks 13..16 -> SQR1 slots
    */

    if (rank >= 1 && rank <= 6)
    {
        uint32_t shift = 5 * (rank - 1);
        pADCx->SQR3 &= ~(0x1FU << shift);
        pADCx->SQR3 |= ((channel & 0x1FU) << shift);
    }
    else if (rank >= 7 && rank <= 12)
    {
        uint32_t shift = 5 * (rank - 7);
        pADCx->SQR2 &= ~(0x1FU << shift);
        pADCx->SQR2 |= ((channel & 0x1FU) << shift);
    }
    else if (rank >= 13 && rank <= 16)
    {
        uint32_t shift = 5 * (rank - 13);
        pADCx->SQR1 &= ~(0x1FU << shift);
        pADCx->SQR1 |= ((channel & 0x1FU) << shift);
    }
}

void ADC_RegisterCallback(ADC_Handle_t *pADCHandle, ADC_AppCallback_t pCallback)
{
    pADCHandle->pAppCallback = pCallback;
}


void ADC_ConfigRegularSequence(ADC_TypeDef *pADCx, uint8_t *channels, uint8_t length)
{
	if(length == 0 || length > 16) return;

	/* set L = Length-1 in SQR1 bits [23:20] */
	pADCx->SQR1 &= ~(0xFU << 20);
	pADCx->SQR1 |= ((length - 1) << 20);

	uint8_t i;
	for(i = 0; i < length; ++i)
	{
		uint8_t ch = channels[i] & 0x1F;
		uint8_t rank = i + 1;

		if(rank <= 6)
		{
			uint32_t shift = 5 * (rank - 1);
			pADCx->SQR3 &= ~(0x1FU << shift);
			pADCx->SQR3 |= (ch << shift);
		}
        else if (rank <= 12)
        {
            uint32_t shift = 5 * (rank - 7);
            pADCx->SQR2 &= ~(0x1FU << shift);
            pADCx->SQR2 |= (ch << shift);
        }
        else /* 13..16 */
        {
            uint32_t shift = 5 * (rank - 13);
            pADCx->SQR1 &= ~(0x1FU << shift);
            pADCx->SQR1 |= (ch << shift);
        }
	}
}

/* extsel : 4-bit index selecting trigger source (see RM0090 table), exten: edge bits (00 disabled, 01 rising, 10 falling, 11 both) */
/* extsel: trigger source (0..15), exten: trigger edge (0=disabled,1=rising,2=falling,3=both) */
void ADC_ConfigExternalTrigger(ADC_TypeDef *pADCx, uint8_t extsel, uint8_t exten)
{
    /* EXTSEL = bits [17:24], EXTEN = bits [28:29] */
    pADCx->CR2 &= ~((0xFFU << 17) | (0x3U << 28));
    pADCx->CR2 |= ((extsel & 0xFFU) << 17);
    pADCx->CR2 |= ((exten & 0x3U) << 28);
}


























