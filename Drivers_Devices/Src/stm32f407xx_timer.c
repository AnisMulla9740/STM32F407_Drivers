/*
 * stm32f407xx_timer.c
 *
 *  Created on: Oct 6, 2025
 *      Author: Anis
 */

#include "stm32f407xx_timer.h"
#include "stm32f407xx.h"
#include <stddef.h>

void TIMER_PeriClockControl(TIM_TypeDef *pTIMx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
	    if (pTIMx == TIM2) RCC->APB1ENR |= (1 << 0);
	    else if (pTIMx == TIM3) RCC->APB1ENR |= (1 << 1);
	    else if (pTIMx == TIM4) RCC->APB1ENR |= (1 << 2);
	    else if (pTIMx == TIM5) RCC->APB1ENR |= (1 << 3);
	    else if (pTIMx == TIM6) RCC->APB1ENR |= (1 << 4);
	    else if (pTIMx == TIM7) RCC->APB1ENR |= (1 << 5);
	    else if (pTIMx == TIM12) RCC->APB1ENR |= (1 << 6);
	    else if (pTIMx == TIM13) RCC->APB1ENR |= (1 << 7);
	    else if (pTIMx == TIM14) RCC->APB1ENR |= (1 << 8);
	    else if (pTIMx == TIM1) RCC->APB2ENR |= (1 << 0);
	    else if (pTIMx == TIM8) RCC->APB2ENR |= (1 << 1);
	    else if (pTIMx == TIM9) RCC->APB2ENR |= (1 << 16);
	    else if (pTIMx == TIM10) RCC->APB2ENR |= (1 << 17);
	    else if (pTIMx == TIM11) RCC->APB2ENR |= (1 << 18);
	}
	else
	{
        if (pTIMx == TIM2) RCC->APB1ENR &= ~(1 << 0);
        else if (pTIMx == TIM3) RCC->APB1ENR &= ~(1 << 1);
        else if (pTIMx == TIM4) RCC->APB1ENR &= ~(1 << 2);
        else if (pTIMx == TIM5) RCC->APB1ENR &= ~(1 << 3);
        else if (pTIMx == TIM6) RCC->APB1ENR &= ~(1 << 4);
        else if (pTIMx == TIM7) RCC->APB1ENR &= ~(1 << 5);
        else if (pTIMx == TIM12) RCC->APB1ENR &= ~(1 << 6);
        else if (pTIMx == TIM13) RCC->APB1ENR &= ~(1 << 7);
        else if (pTIMx == TIM14) RCC->APB1ENR &= ~(1 << 8);
        else if (pTIMx == TIM1) RCC->APB2ENR &= ~(1 << 0);
        else if (pTIMx == TIM8) RCC->APB2ENR &= ~(1 << 1);
        else if (pTIMx == TIM9) RCC->APB2ENR &= ~(1 << 16);
        else if (pTIMx == TIM10) RCC->APB2ENR &= ~(1 << 17);
        else if (pTIMx == TIM11) RCC->APB2ENR &= ~(1 << 18);
	}
}

/* ------------------Init/De-Init--------------------------------*/
void TIMER_Init(TIMER_Handle_t *pTIMHandle)
{
	if(!pTIMHandle || !pTIMHandle->pTIMx || !pTIMHandle->pTimConfig) return;

	TIM_TypeDef *TIMx = pTIMHandle->pTIMx;
	TIMER_Config_t *cfg = pTIMHandle->pTimConfig;

	TIMER_PeriClockControl(TIMx, ENABLE);

	/* disable counter while configuring */
	TIMx->CR1 &= ~(1U << 0);

	/* PSC and ARR */
	if(cfg->Prescaler > TIMER_MAX_PSC) cfg->Prescaler = TIMER_MAX_PSC;
	if(cfg->Period > TIMER_MAX_ARR) cfg->Period = TIMER_MAX_ARR;
	TIMx->PSC = cfg->Prescaler & 0xFFFFU;
	TIMx->ARR = cfg->Period & 0xFFFFU;

    /* Counter mode: DIR bit (CR1[4]) for up/down. Center-aligned uses CMS bits (CR1[5:6]) */
    TIMx->CR1 &= ~((1U<<4) | (3U<<5)); /* clear DIR and CMS */
    if (cfg->CounterMode == TIMER_COUNTER_DOWN)
    {
        TIMx->CR1 |= (1U << 4); /* DIR = 1 */
    }
    else if (cfg->CounterMode == TIMER_COUNTER_CENTER1)
    {
        TIMx->CR1 |= (1U << 5);
    }
    else if (cfg->CounterMode == TIMER_COUNTER_CENTER2)
    {
        TIMx->CR1 |= (2U << 5);
    }
    else if (cfg->CounterMode == TIMER_COUNTER_CENTER3)
    {
        TIMx->CR1 |= (3U << 5);
    }

    /* Auto-reload preload (ARPE bit CR1[7] is normally in CR1 for some timers; but ARPE is in CR1? On F4 it's in CR1?
       Common approach: use CR1 ARPE for TIMx using CR1[7] — some MCUs set it in TIMx->CR1. */
    if (cfg->AutoReloadPreload == TIMER_AUTORELOAD_PRELOAD_ENABLE)
        TIMx->CR1 |= (1U << 7);
    else
        TIMx->CR1 &= ~(1U << 7);

    /* One-pulse mode (OPM bit CR1[3]) */
    if (cfg->OnePulseMode == ENABLE)
        TIMx->CR1 |= (1U << 3);
    else
        TIMx->CR1 &= ~(1U << 3);

    /* Update request source URS (CR1[2] or CR2? For TIM on F4 URS is CR1[2] for legacy. We'll set CR1[2] accordingly.) */
    if (cfg->URS == TIMER_URS_ONLY_OVERFLOW)
        TIMx->CR1 |= (1U << 2);
    else
        TIMx->CR1 &= ~(1U << 2);

    /* Generate an update to load the prescaler value immediately if necessary */
    TIMx->EGR |= (1U << 0); /* UG */
}

void TIMER_DeInit(TIM_TypeDef *pTIMx)
{
    if (!pTIMx) return;

    /* Use RCC reset registers (APB1RSTR / APB2RSTR) for deinit */
    if (pTIMx == TIM2) { RCC->APB1RSTR |= (1U << 0); RCC->APB1RSTR &= ~(1U << 0); }
    else if (pTIMx == TIM3) { RCC->APB1RSTR |= (1U << 1); RCC->APB1RSTR &= ~(1U << 1); }
    else if (pTIMx == TIM4) { RCC->APB1RSTR |= (1U << 2); RCC->APB1RSTR &= ~(1U << 2); }
    else if (pTIMx == TIM5) { RCC->APB1RSTR |= (1U << 3); RCC->APB1RSTR &= ~(1U << 3); }
    else if (pTIMx == TIM6) { RCC->APB1RSTR |= (1U << 4); RCC->APB1RSTR &= ~(1U << 4); }
    else if (pTIMx == TIM7) { RCC->APB1RSTR |= (1U << 5); RCC->APB1RSTR &= ~(1U << 5); }
    else if (pTIMx == TIM1) { RCC->APB2RSTR |= (1U << 0); RCC->APB2RSTR &= ~(1U << 0); }
    else if (pTIMx == TIM8) { RCC->APB2RSTR |= (1U << 1); RCC->APB2RSTR &= ~(1U << 1); }
    else if (pTIMx == TIM12) { RCC->APB1RSTR |= (1U << 6); RCC->APB1RSTR &= ~(1U << 6); }
    else if (pTIMx == TIM13) { RCC->APB1RSTR |= (1U << 7); RCC->APB1RSTR &= ~(1U << 7); }
    else if (pTIMx == TIM14) { RCC->APB1RSTR |= (1U << 8); RCC->APB1RSTR &= ~(1U << 8); }
}

/* ---------- Counter control ---------- */
void TIMER_Start(TIMER_Handle_t *pTIMHandle)
{
	if(!pTIMHandle || !pTIMHandle->pTIMx) return;
	TIM_TypeDef *TIMx = pTIMHandle->pTIMx;

	//generate an update to Load the ARR/PSC if ARPE used
	TIMx->EGR |= (1U << 0);

	/* clear the update flag */
	TIMx->SR &= ~(1U << 0);

	/* enable the counter */
	TIMx->CR1 |= (1U << 0);
}

void TIMER_Stop(TIMER_Handle_t *pTIMHandle)
{
	if(!pTIMHandle || !pTIMHandle->pTIMx) return;
	pTIMHandle->pTIMx->CR1 &= ~(1U << 0);
}

void TIMER_ResetCounter(TIM_TypeDef *pTIMx)
{
    if (!pTIMx) return;
    pTIMx->CNT = 0U;
}

void TIMER_SetPrescaler(TIM_TypeDef *pTIMx, uint32_t psc)
{
    if (!pTIMx) return;

    if (psc > TIMER_MAX_PSC) psc = TIMER_MAX_PSC;
    pTIMx->PSC = (uint16_t)(psc & 0xFFFFU);

    /* update generation to take effect */
    pTIMx->EGR |= (1U << 0);
}

void TIMER_SetAutoReload(TIM_TypeDef *pTIMx, uint32_t arr)
{
    if (!pTIMx) return;
    if (arr > TIMER_MAX_ARR) arr = TIMER_MAX_ARR;
    pTIMx->ARR = (uint16_t)(arr & 0xFFFFU);
    pTIMx->EGR |= (1U << 0);
}

uint32_t TIMER_GetCounter(TIM_TypeDef *pTIMx)
{
    if (!pTIMx) return 0;
    return pTIMx->CNT;
}

/* ---------- NVIC / IRQ helpers ---------- */
void TIMER_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31U) *NVIC_ISER0 |= (1UL << IRQNumber);
        else if (IRQNumber <= 63U) *NVIC_ISER1 |= (1UL << (IRQNumber - 32U));
        else if (IRQNumber <= 95U) *NVIC_ISER2 |= (1UL << (IRQNumber - 64U));
    }
    else
    {
        if (IRQNumber <= 31U) *NVIC_ICER0 |= (1UL << IRQNumber);
        else if (IRQNumber <= 63U) *NVIC_ICER1 |= (1UL << (IRQNumber - 32U));
        else if (IRQNumber <= 95U) *NVIC_ICER2 |= (1UL << (IRQNumber - 64U));
    }
}

void TIMER_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4U;
    uint8_t iprx_section = IRQNumber % 4U;
    uint8_t shift_amount = (8U * iprx_section) + (8U - NO_PR_BITS_IMPLEMENTED);
    NVIC_PR_BASE_ADDR[iprx] |= (IRQPriority << shift_amount);
}

/* ---------- DIER helpers: enable update / CC interrupts ---------- */
void TIMER_EnableIt_Update(TIM_TypeDef *pTIMx, uint8_t EnOrDi)
{
	if(!pTIMx) return;

	if(EnOrDi == ENABLE) pTIMx->DIER |= (1U << 0); //UIE
	else pTIMx->DIER &= ~(1U << 0);
}

void TIMER_EnableIt_CC(TIM_TypeDef *pTIMx, uint8_t channel, uint8_t EnOrDi)
{
    if (!pTIMx) return;
    switch (channel)
    {
        case TIMER_CHANNEL_1:
            if (EnOrDi == ENABLE) pTIMx->DIER |= (1U << 1); else pTIMx->DIER &= ~(1U << 1);
            break;
        case TIMER_CHANNEL_2:
            if (EnOrDi == ENABLE) pTIMx->DIER |= (1U << 2); else pTIMx->DIER &= ~(1U << 2);
            break;
        case TIMER_CHANNEL_3:
            if (EnOrDi == ENABLE) pTIMx->DIER |= (1U << 3); else pTIMx->DIER &= ~(1U << 3);
            break;
        case TIMER_CHANNEL_4:
            if (EnOrDi == ENABLE) pTIMx->DIER |= (1U << 4); else pTIMx->DIER &= ~(1U << 4);
            break;
        default: break;
    }
}

/* ------------------------------Flags----------------------------------*/
uint8_t TIMER_GetFlagStatus(TIM_TypeDef *pTIMx, uint32_t flagMask)
{
    if (!pTIMx) return 0;
    return ((pTIMx->SR & flagMask) ? 1U : 0U);
}

void TIMER_ClearFlag(TIM_TypeDef *pTIMx, uint32_t flagMask)
{
    if (!pTIMx) return;
    pTIMx->SR &= ~flagMask;
}

/* ---------- Unified IRQ handler ---------- */
void TIMER_IRQHandling(TIMER_Handle_t *pTIMHandle)
{
    if (!pTIMHandle || !pTIMHandle->pTIMx) return;
    TIM_TypeDef *TIMx = pTIMHandle->pTIMx;

    /* Update event (UIF) */
    if (TIMx->SR & (1U << 0))
    {
        /* clear flag */
        TIMx->SR &= ~(1U << 0);
        if (pTIMHandle->pAppCallback)
            pTIMHandle->pAppCallback(TIMER_EVENT_UPDATE, 0, 0);
    }

    /* CC1 */
    if (TIMx->SR & (1U << 1))
    {
        uint32_t cap = TIMx->CCR1;
        TIMx->SR &= ~(1U << 1);
        if (pTIMHandle->pAppCallback)
            pTIMHandle->pAppCallback(TIMER_EVENT_CC1, 1, cap);
    }

    /* CC2 */
    if (TIMx->SR & (1U << 2))
    {
        uint32_t cap = TIMx->CCR2;
        TIMx->SR &= ~(1U << 2);
        if (pTIMHandle->pAppCallback)
            pTIMHandle->pAppCallback(TIMER_EVENT_CC2, 2, cap);
    }

    /* CC3 */
    if (TIMx->SR & (1U << 3))
    {
        uint32_t cap = TIMx->CCR3;
        TIMx->SR &= ~(1U << 3);
        if (pTIMHandle->pAppCallback)
            pTIMHandle->pAppCallback(TIMER_EVENT_CC3, 3, cap);
    }

    /* CC4 */
    if (TIMx->SR & (1U << 4))
    {
        uint32_t cap = TIMx->CCR4;
        TIMx->SR &= ~(1U << 4);
        if (pTIMHandle->pAppCallback)
            pTIMHandle->pAppCallback(TIMER_EVENT_CC4, 4, cap);
    }
}

/* Callback registration */
void TIMER_RegisterCallback(TIMER_Handle_t *pTIMHandle, TIMER_AppCallback_t pCallback)
{
    if (!pTIMHandle) return;
    pTIMHandle->pAppCallback = pCallback;
}

/* ---------- PWM / Output Compare APIs ---------- */

/* Helper to set CCMR and CCER for a channel */
static void _timer_configure_oc(TIM_TypeDef *TIMx, uint8_t channel, TIMER_PWMConfig_t *cfg)
{
    if (!TIMx || !cfg) return;

    /* CCMR1: CC1/CC2, CCMR2: CC3/CC4 */
    if (channel == TIMER_CHANNEL_1)
    {
        /* Clear OC1M (bits 6:4) and OC1PE (bit 3), OC1CE( bit7 maybe) */
        TIMx->CCMR1 &= ~((7U<<4) | (1U<<3));
        TIMx->CCMR1 |= ((cfg->OCMode & 0x7U) << 4);
        if (cfg->PreloadEnable == ENABLE) TIMx->CCMR1 |= (1U<<3);
        else TIMx->CCMR1 &= ~(1U<<3);

        /* Capture/Compare enable in CCER */
        if (cfg->OCPolarity == TIMER_OC_POLARITY_HIGH) TIMx->CCER &= ~(1U<<1); /* CC1P = 0 => active high */
        else TIMx->CCER |= (1U<<1);
        TIMx->CCER |= (1U<<0); /* CC1E: enable output (but don't enable if using OC only?) */
    }
    else if (channel == TIMER_CHANNEL_2)
    {
        TIMx->CCMR1 &= ~((7U<<12) | (1U<<11));
        TIMx->CCMR1 |= ((cfg->OCMode & 0x7U) << 12);
        if (cfg->PreloadEnable == ENABLE) TIMx->CCMR1 |= (1U<<11);
        else TIMx->CCMR1 &= ~(1U<<11);

        if (cfg->OCPolarity == TIMER_OC_POLARITY_HIGH) TIMx->CCER &= ~(1U<<5);
        else TIMx->CCER |= (1U<<5);
        TIMx->CCER |= (1U<<4); /* CC2E */
    }
    else if (channel == TIMER_CHANNEL_3)
    {
        TIMx->CCMR2 &= ~((7U<<4) | (1U<<3));
        TIMx->CCMR2 |= ((cfg->OCMode & 0x7U) << 4);
        if (cfg->PreloadEnable == ENABLE) TIMx->CCMR2 |= (1U<<3);
        else TIMx->CCMR2 &= ~(1U<<3);

        if (cfg->OCPolarity == TIMER_OC_POLARITY_HIGH) TIMx->CCER &= ~(1U<<9);
        else TIMx->CCER |= (1U<<9);
        TIMx->CCER |= (1U<<8); /* CC3E */
    }
    else if (channel == TIMER_CHANNEL_4)
    {
        TIMx->CCMR2 &= ~((7U<<12) | (1U<<11));
        TIMx->CCMR2 |= ((cfg->OCMode & 0x7U) << 12);
        if (cfg->PreloadEnable == ENABLE) TIMx->CCMR2 |= (1U<<11);
        else TIMx->CCMR2 &= ~(1U<<11);

        if (cfg->OCPolarity == TIMER_OC_POLARITY_HIGH) TIMx->CCER &= ~(1U<<13);
        else TIMx->CCER |= (1U<<13);
        TIMx->CCER |= (1U<<12); /* CC4E */
    }

    /* If using preload, enable ARR preload already handled in TIMER_Init; to update CCRx from preload, we use EGR UG when needed */
}

/* Initialize a channel for PWM/OC */
void TIMER_PWMInit(TIM_TypeDef *pTIMx, uint8_t channel, TIMER_PWMConfig_t *pwmConfig)
{
    if (!pTIMx || !pwmConfig) return;
    _timer_configure_oc(pTIMx, channel, pwmConfig);
}

/* Set CCRx from duty percent */
void TIMER_SetPWMDutyCycle(TIM_TypeDef *pTIMx, uint8_t channel, float duty_percent)
{
    if (!pTIMx) return;
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t arr = pTIMx->ARR;
    uint32_t ccr = (uint32_t)((duty_percent / 100.0f) * (float)arr + 0.5f);

    switch (channel)
    {
        case TIMER_CHANNEL_1: pTIMx->CCR1 = ccr; break;
        case TIMER_CHANNEL_2: pTIMx->CCR2 = ccr; break;
        case TIMER_CHANNEL_3: pTIMx->CCR3 = ccr; break;
        case TIMER_CHANNEL_4: pTIMx->CCR4 = ccr; break;
        default: break;
    }
}


/* Start PWM on given channel: enable output and counter (if needed) */
void TIMER_PWMStart(TIMER_Handle_t *pTIMHandle, uint8_t channel)
{
    if (!pTIMHandle || !pTIMHandle->pTIMx) return;
    TIM_TypeDef *TIMx = pTIMHandle->pTIMx;

    /* Enable capture/compare output (already set by _timer_configure_oc) */
    /* For advanced timers (TIM1/TIM8), BDTR may be required to enable main output (MOE bit in BDTR). Not all timers have BDTR.
       We'll set MOE bit if BDTR exists (BDTR at offset 0x44) - TIM_TypeDef contains BDTR field, so we set it for TIM1/TIM8. */
    if (TIMx == TIM1 || TIMx == TIM8)
    {
        TIMx->BDTR |= (1U << 15); /* MOE bit */
    }

    /* Ensure ARR/PSC loaded */
    TIMx->EGR |= (1U << 0);

    /* Enable counter */
    TIMx->CR1 |= (1U << 0);
}

/* Stop PWM on given channel by disabling corresponding CCxE bit */
void TIMER_PWMStop(TIMER_Handle_t *pTIMHandle, uint8_t channel)
{
    if (!pTIMHandle || !pTIMHandle->pTIMx) return;
    TIM_TypeDef *TIMx = pTIMHandle->pTIMx;

    switch (channel)
    {
        case TIMER_CHANNEL_1: TIMx->CCER &= ~(1U<<0); break;
        case TIMER_CHANNEL_2: TIMx->CCER &= ~(1U<<4); break;
        case TIMER_CHANNEL_3: TIMx->CCER &= ~(1U<<8); break;
        case TIMER_CHANNEL_4: TIMx->CCER &= ~(1U<<12); break;
        default: break;
    }
}

/* Change PWM frequency by updating PSC/ARR. Duty remains relative to ARR; user may want to re-program CCRx accordingly. */
void TIMER_ChangePWMFrequency(TIM_TypeDef *pTIMx, uint32_t newPSC, uint32_t newARR)
{
    if (!pTIMx) return;
    if (newPSC > TIMER_MAX_PSC) newPSC = TIMER_MAX_PSC;
    if (newARR > TIMER_MAX_ARR) newARR = TIMER_MAX_ARR;

    /* read current duty ratios to restore them later (as CCR/ARR) for each channel */
    uint32_t oldARR = pTIMx->ARR;
    uint32_t c1 = pTIMx->CCR1;
    uint32_t c2 = pTIMx->CCR2;
    uint32_t c3 = pTIMx->CCR3;
    uint32_t c4 = pTIMx->CCR4;

    float d1 = (oldARR == 0) ? 0.0f : ((float)c1 / (float)oldARR);
    float d2 = (oldARR == 0) ? 0.0f : ((float)c2 / (float)oldARR);
    float d3 = (oldARR == 0) ? 0.0f : ((float)c3 / (float)oldARR);
    float d4 = (oldARR == 0) ? 0.0f : ((float)c4 / (float)oldARR);

    TIMER_SetPrescaler(pTIMx, newPSC);
    TIMER_SetAutoReload(pTIMx, newARR);

    /* restore CCR values to the same duty ratio */
    pTIMx->CCR1 = (uint32_t)(d1 * (float)newARR + 0.5f);
    pTIMx->CCR2 = (uint32_t)(d2 * (float)newARR + 0.5f);
    pTIMx->CCR3 = (uint32_t)(d3 * (float)newARR + 0.5f);
    pTIMx->CCR4 = (uint32_t)(d4 * (float)newARR + 0.5f);

    pTIMx->EGR |= (1U << 0);
}

/* ---------- Input Capture APIs ---------- */

/* Helper to configure input capture */
void TIMER_ICInit(TIM_TypeDef *pTIMx, uint8_t channel, TIMER_ICConfig_t *icConfig)
{
    if (!pTIMx || !icConfig) return;

    /* We configure CCMRx and CCER for input mapping */
    if (channel == TIMER_CHANNEL_1)
    {
        /* CC1S bits (CCMR1[1:0]) = 01 to map TI1 as input */
        pTIMx->CCMR1 &= ~(3U << 0);
        pTIMx->CCMR1 |= ((icConfig->ICSelection & 0x3U) << 0); /* 00: CC1 channel is output, 01: TI1, 10: TI2, 11: TRC */

        /* set input filter (IC1F bits [7:4]) */
        pTIMx->CCMR1 &= ~(0xFU << 4);
        pTIMx->CCMR1 |= ((icConfig->ICFilter & 0xFU) << 4);

        /* set prescaler (IC1PSC [3:2]) */
        pTIMx->CCMR1 &= ~(3U << 2);
        pTIMx->CCMR1 |= ((icConfig->ICPrescaler & 0x3U) << 2);

        /* polarity */
        if (icConfig->ICPolarity == TIMER_IC_POLARITY_RISING) pTIMx->CCER &= ~(1U<<1);
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_FALLING) { pTIMx->CCER |= (1U<<1); pTIMx->CCER &= ~(1U<<3); }
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_BOTH) pTIMx->CCER |= (1U<<1) | (1U<<3);

        /* enable capture on CC1 */
        pTIMx->CCER |= (1U<<0);
    }
    else if (channel == TIMER_CHANNEL_2)
    {
        pTIMx->CCMR1 &= ~(3U << 8);
        pTIMx->CCMR1 |= ((icConfig->ICSelection & 0x3U) << 8);

        pTIMx->CCMR1 &= ~(0xFU << 12);
        pTIMx->CCMR1 |= ((icConfig->ICFilter & 0xFU) << 12);

        pTIMx->CCMR1 &= ~(3U << 10);
        pTIMx->CCMR1 |= ((icConfig->ICPrescaler & 0x3U) << 10);

        if (icConfig->ICPolarity == TIMER_IC_POLARITY_RISING) pTIMx->CCER &= ~(1U<<5);
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_FALLING) { pTIMx->CCER |= (1U<<5); pTIMx->CCER &= ~(1U<<7); }
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_BOTH) pTIMx->CCER |= (1U<<5) | (1U<<7);

        pTIMx->CCER |= (1U<<4);
    }
    else if (channel == TIMER_CHANNEL_3)
    {
        pTIMx->CCMR2 &= ~(3U << 0);
        pTIMx->CCMR2 |= ((icConfig->ICSelection & 0x3U) << 0);

        pTIMx->CCMR2 &= ~(0xFU << 4);
        pTIMx->CCMR2 |= ((icConfig->ICFilter & 0xFU) << 4);

        pTIMx->CCMR2 &= ~(3U << 2);
        pTIMx->CCMR2 |= ((icConfig->ICPrescaler & 0x3U) << 2);

        if (icConfig->ICPolarity == TIMER_IC_POLARITY_RISING) pTIMx->CCER &= ~(1U<<9);
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_FALLING) { pTIMx->CCER |= (1U<<9); pTIMx->CCER &= ~(1U<<11); }
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_BOTH) pTIMx->CCER |= (1U<<9) | (1U<<11);

        pTIMx->CCER |= (1U<<8);
    }
    else if (channel == TIMER_CHANNEL_4)
    {
        pTIMx->CCMR2 &= ~(3U << 8);
        pTIMx->CCMR2 |= ((icConfig->ICSelection & 0x3U) << 8);

        pTIMx->CCMR2 &= ~(0xFU << 12);
        pTIMx->CCMR2 |= ((icConfig->ICFilter & 0xFU) << 12);

        pTIMx->CCMR2 &= ~(3U << 10);
        pTIMx->CCMR2 |= ((icConfig->ICPrescaler & 0x3U) << 10);

        if (icConfig->ICPolarity == TIMER_IC_POLARITY_RISING) pTIMx->CCER &= ~(1U<<13);
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_FALLING) { pTIMx->CCER |= (1U<<13); pTIMx->CCER &= ~(1U<<15); }
        else if (icConfig->ICPolarity == TIMER_IC_POLARITY_BOTH) pTIMx->CCER |= (1U<<13) | (1U<<15);

        pTIMx->CCER |= (1U<<12);
    }
}

/* Read captured CCRx value */
uint32_t TIMER_ICReadCapture(TIM_TypeDef *pTIMx, uint8_t channel)
{
    if (!pTIMx) return 0;
    switch (channel)
    {
        case TIMER_CHANNEL_1: return pTIMx->CCR1;
        case TIMER_CHANNEL_2: return pTIMx->CCR2;
        case TIMER_CHANNEL_3: return pTIMx->CCR3;
        case TIMER_CHANNEL_4: return pTIMx->CCR4;
        default: return 0;
    }
}

void TIMER_ICClearFlag(TIM_TypeDef *pTIMx, uint8_t channel)
{
    if (!pTIMx) return;
    switch (channel)
    {
        case TIMER_CHANNEL_1: pTIMx->SR &= ~(1U<<1); break;
        case TIMER_CHANNEL_2: pTIMx->SR &= ~(1U<<2); break;
        case TIMER_CHANNEL_3: pTIMx->SR &= ~(1U<<3); break;
        case TIMER_CHANNEL_4: pTIMx->SR &= ~(1U<<4); break;
        default: break;
    }
}
