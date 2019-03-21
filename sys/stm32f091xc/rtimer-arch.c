/*
 * \file
 * Implementation of the arch-specific rtimer functions for stm32f0xx
 *
 */
#include <stdint.h>
#include "stm32f091xc_conf.h"
#include "rtimer-arch.h"
#include "stm32f091xc.h"

#define APB1ENR_TIM2EN  				((uint32_t)0x01)
#define ARCTIMER_OC_ISR_BIT				((uint32_t)(1 << 1))
#define ARCTIMER_CLK_DIV				((uint32_t)(CPU_MAIN_CLK / RTIMER_ARCH_SECOND))
#define ARCTIMER_CNT_OVERFLOW			(0xFFFFFFFF)
#define ARCTIMER_NVIC_PRIORITY 			((uint32_t)0x00)

#define INTERRUPTS_ENABLE()  			__enable_irq()
#define INTERRUPTS_DISABLE() 			__disable_irq()
/*---------------------------------------------------------------------------*/
static volatile rtimer_clock_t    		next_trigger;

/*---------------------------------------------------------------------------*/
/**
 * \brief
 */
void
rtimer_arch_init(void)
{
	volatile uint32_t tmpreg;

	/* Enable TIM2 CLK */
	RCC->APB1ENR |= APB1ENR_TIM2EN;
	/* Delay after an RCC peripheral clock enabling */
	tmpreg = (RCC->APB1ENR & APB1ENR_TIM2EN);
	(void)tmpreg;

	/* TIM2 CHANNEL 1 CONFIGURATION */
	/* UP COUNTER */
	TIM2->CR1 &= ~(uint32_t)(1 << 4);
	/* CC1 INTERRUPT ENABLED */
	TIM2->DIER |= (uint32_t)2;
	/* CAPTURE OUTPUT, NO PRELOAD REGISTER */
	TIM2->CCMR1 &= ~(uint32_t)3;
	TIM2->CCMR1 &= ~(uint32_t)(1 << 4);
	/* OVERFLOW VALUE */
	TIM2->ARR = ARCTIMER_CNT_OVERFLOW;
	/* PRESCALER */
	TIM2->PSC = ARCTIMER_CLK_DIV - 1;
	TIM2->EGR = 1;
	/* COMPARE VALUE */
	TIM2->CCR1 = ARCTIMER_CNT_OVERFLOW;
	/* ENABLE */
	TIM2->CR1 |= (uint32_t)1;

	// TODO: VERIFY INTERRUPT PRIORITY
	NVIC_SetPriority(TIM2_IRQn, ARCTIMER_NVIC_PRIORITY);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Schedules an rtimer task to be triggered at time t
 * \param t The time when the task will need executed. This is an absolute
 *          time, in other words the task will be executed AT time \e t,
 *          not IN \e t ticks
 */
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  rtimer_clock_t now;

  INTERRUPTS_DISABLE();

  now = RTIMER_NOW();

  if((int32_t)(t - now) < 7) {
    t = now + 7;
  }

  /* SET COMPARE VALUE */
  TIM2->CCR1 = t;

  INTERRUPTS_ENABLE();

  /* Store the value. The LPM module will query us for it */
  next_trigger = t;

  NVIC_EnableIRQ(TIM2_IRQn);
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_next_trigger()
{
  return next_trigger;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the current real-time clock time
 * \return The current rtimer time in ticks
 */
rtimer_clock_t
rtimer_arch_now()
{
  rtimer_clock_t rv;

  rv = TIM2->CNT;

  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief The rtimer ISR
 *
 *        Interrupts are only turned on when we have an rtimer task to schedule
 *        Once the interrupt fires, the task is called and then interrupts no
 *        longer get acknowledged until the next task needs scheduled.
 */
void
rtimer_isr()
{
  /*
   * If we were in PM1+, call the wake-up sequence first. This will make sure
   * that the 32MHz OSC is selected as the clock source. We need to do this
   * before calling the next rtimer_task, since the task may need the RF.
   */
 // lpm_exit();

  next_trigger = 0;

  NVIC_ClearPendingIRQ(TIM2_IRQn);
  NVIC_DisableIRQ(TIM2_IRQn);

  rtimer_run_next();
}

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void
TIM2_IRQHandler(void)
{
  if(TIM2->SR & ARCTIMER_OC_ISR_BIT) {
	  TIM2->SR &= ~ARCTIMER_OC_ISR_BIT;
	  rtimer_isr();
  }
}
/*---------------------------------------------------------------------------*/
/** @} */
