/**
 * Implementation of the rtimer module for the stm32f091xc
 *
 * The rtimer runs on the TIM2 with 32kHz (32768 Hz) clock.
 *
 */
#ifndef RTIMER_ARCH_H_
#define RTIMER_ARCH_H_

#include "rtimer.h"
#include "stm32f091xc_conf.h"

/* Do the math in 32bits to save precision.
 * Round to nearest integer rather than truncate. */
#define US_TO_RTIMERTICKS(US)  ((US) >= 0 ?                        \
                               (((int32_t)(US) * (RTIMER_ARCH_SECOND) + 500000) / 1000000L) :      \
                               ((int32_t)(US) * (RTIMER_ARCH_SECOND) - 500000) / 1000000L)

#define RTIMERTICKS_TO_US(T)   ((T) >= 0 ?                     \
                               (((int32_t)(T) * 1000000L + ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND)) : \
                               ((int32_t)(T) * 1000000L - ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND))

/* A 64-bit version because the 32-bit one cannot handle T >= 4295 ticks.
   Intended only for positive values of T. */
#define RTIMERTICKS_TO_US_64(T)  ((uint32_t)(((uint64_t)(T) * 1000000 + ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND)))

/** \sa RTIMER_NOW() */
rtimer_clock_t rtimer_arch_now(void);

/**
 * \brief Get the time of the next scheduled rtimer trigger
 * \return The time next rtimer ISR is scheduled for
 */
rtimer_clock_t rtimer_arch_next_trigger(void);

#endif /* RTIMER_ARCH_H_ */

/**
 * @}
 * @}
 */
