/**
 * Implementation of the clock module for the stm32f091xc
 *
 * To implement the clock functionality, we use the SysTick peripheral on the
 * cortex-M3. We run the system clock at a configurable speed and set the 
 * SysTick to give us 128 interrupts / sec. However, the Sleep Timer counter 
 * value is used for the number of elapsed ticks in order to avoid a 
 * significant time drift caused by PM1/2. Contrary to the Sleep Timer, the 
 * SysTick peripheral is indeed frozen during PM1/2, so adjusting upon wake-up 
 * a tick counter based on this peripheral would hardly be accurate.
 * @{
 *
 * \file
 * Clock driver implementation for the TI cc2538
 */
#include "stm32f091xc.h"
#include "core_cm0.h"
#include "rtimer-arch.h"
#include "etimer.h"
#include "rtimer.h"
#include <stdint.h>
/*---------------------------------------------------------------------------*/
/**
 * \name Macros and typedefs
 *
 * Those values are not meant to be modified by the user
 * @{
 */
/* Clock (time) comparison macro */
#define CLOCK_LT(a, b)  ((signed long)((a) - (b)) < 0)

#define RTIMER_CLOCK_TICK_RATIO (RTIMER_SECOND / CLOCK_SECOND)

/* Prescaler for GPT0:Timer A used for clock_delay_usec(). */
#if SYS_CTRL_SYS_CLOCK < SYS_CTRL_1MHZ
#error System clock speeds below 1MHz are not supported
#endif
//#define PRESCALER_VALUE         (SYS_CTRL_SYS_CLOCK / SYS_CTRL_1MHZ - 1)

/* Period of the SysTick counter expressed as a number of ticks */
#if SYS_CTRL_SYS_CLOCK % CLOCK_SECOND
/* Too low clock speeds will lead to reduced accurracy */
#error System clock speed too slow for CLOCK_SECOND, accuracy reduced
#endif
#define SYSTICK_PERIOD          (CPU_MAIN_CLK / CLOCK_SECOND)

static volatile uint64_t rt_ticks_startup = 0, rt_ticks_epoch = 0;
/*---------------------------------------------------------------------------*/
/**
 * \brief Arch-specific implementation of clock_init for the cc2538
 *
 * We initialise the SysTick to fire 128 interrupts per second, giving us a
 * value of 128 for CLOCK_SECOND
 *
 * We also initialise GPT0:Timer A, which is used by clock_delay_usec().
 * We use 16-bit range (individual), count-down, one-shot, no interrupts.
 * The prescaler is computed according to the system clock in order to get 1
 * tick per usec.
 */
void
clock_init(void)
{
  SysTick_Config(SYSTICK_PERIOD);
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return rt_ticks_startup / RTIMER_CLOCK_TICK_RATIO;
}
/*---------------------------------------------------------------------------*/
void
clock_set_seconds(unsigned long sec)
{
  rt_ticks_epoch = (uint64_t)sec * RTIMER_SECOND;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  return rt_ticks_epoch / RTIMER_SECOND;
}
/*---------------------------------------------------------------------------*/
void
clock_wait(clock_time_t i)
{
  clock_time_t start;

  start = clock_time();
  while(clock_time() - start < (clock_time_t)i);
}
/*---------------------------------------------------------------------------*/
/*
 * Arch-specific implementation of clock_delay_usec
 */
void
clock_delay_usec(uint16_t dt)
{
	(void)dt;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Obsolete delay function but we implement it here since some code
 * still uses it
 */
void
clock_delay(unsigned int i)
{
  clock_delay_usec(i);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Update the software clock ticks and seconds
 *
 * This function is used to update the software tick counters whenever the
 * system clock might have changed, which can occur upon a SysTick ISR or upon
 * wake-up from PM1/2.
 *
 * For the software clock ticks counter, the Sleep Timer counter value is used
 * as the base tick value, and extended to a 64-bit value thanks to a detection
 * of wraparounds.
 *
 * For the seconds counter, the changes of the Sleep Timer counter value are
 * added to the reference time, which is either the startup time or the value
 * passed to clock_set_seconds().
 *
 * This function polls the etimer process if an etimer has expired.
 */
static void
update_ticks(void)
{
  rtimer_clock_t now;
  uint64_t prev_rt_ticks_startup, cur_rt_ticks_startup;
  uint32_t cur_rt_ticks_startup_hi;

  now = RTIMER_NOW();
  prev_rt_ticks_startup = rt_ticks_startup;

  cur_rt_ticks_startup_hi = prev_rt_ticks_startup >> 32;
  if(now < (rtimer_clock_t)prev_rt_ticks_startup) {
    cur_rt_ticks_startup_hi++;
  }
  cur_rt_ticks_startup = (uint64_t)cur_rt_ticks_startup_hi << 32 | now;
  rt_ticks_startup = cur_rt_ticks_startup;

  rt_ticks_epoch += cur_rt_ticks_startup - prev_rt_ticks_startup;

  /*
   * Inform the etimer library that the system clock has changed and that an
   * etimer might have expired.
   */
  if(etimer_pending()) {
    etimer_request_poll();
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Adjust the clock following missed SysTick ISRs
 *
 * This function is useful when coming out of PM1/2, during which the system
 * clock is stopped. We adjust the clock counters like after any SysTick ISR.
 *
 * \note This function is only meant to be used by lpm_exit(). Applications
 * should really avoid calling this
 */
void
clock_adjust(void)
{
  /* Halt the SysTick while adjusting */
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

  update_ticks();

  /* Re-Start the SysTick */
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief The clock Interrupt Service Routine
 *
 * It polls the etimer process if an etimer has expired. It also updates the
 * software clock tick and seconds counter.
 */
void
clock_isr(void)
{
  update_ticks();
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 * @}
 */
