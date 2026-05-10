/*
 * dumb_timer.h
 *
 *  Created on: May 8, 2026
 *      Author: Dhruv'
 */

#ifndef SRC_DUMB_TIMER_H_
#define SRC_DUMB_TIMER_H_

#include <stdint.h>
#include <math.h>

#define SEC_TO_MS(sec) ((uint32_t)roundf((float)(sec) * 1000.0f))
#define MIN_TO_MS(min) ((uint32_t)roundf((float)(min) * 60000.0f))

struct Dumb_Timer {
	uint32_t start_time_ms;
	uint32_t time_ms;
	uint8_t started;
};

void dumb_timer_init(struct Dumb_Timer* timer, uint32_t time_ms);
void dumb_timer_start(struct Dumb_Timer* timer);
void dumb_timer_clear(struct Dumb_Timer* timer);
uint8_t dumb_timer_done(struct Dumb_Timer* timer);

#endif /* SRC_DUMB_TIMER_H_ */
