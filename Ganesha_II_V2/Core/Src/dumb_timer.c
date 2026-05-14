/*
 * dumb_timer.c
 *
 *  Created on: May 8, 2026
 *      Author: Dhruv Shah
 */

#include "dumb_timer.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

void dumb_timer_init(struct Dumb_Timer* timer, uint32_t time_ms) {
    timer->start_time_ms = 0;
	timer->time_ms = time_ms;
	timer->started = 0;
}

void dumb_timer_start(struct Dumb_Timer* timer) {
	timer->start_time_ms = HAL_GetTick();
	timer->started = 1;
}

void dumb_timer_clear(struct Dumb_Timer* timer) {
    timer->start_time_ms = 0;
    timer->started = 0;
}

uint8_t dumb_timer_done(struct Dumb_Timer* timer) {
	if ((HAL_GetTick() - timer->start_time_ms >= timer->time_ms) && timer->started) {
		return 1;
	} else {
		return 0;
	}
}
