/*
 * us_timer.c
 *
 *  Created on: Apr 27, 2026
 *      Author: Dhruv Shah
 */

#include "main.h"

uint32_t get_time_us() {
	return __HAL_TIM_GET_COUNTER(&htim2);
}

void delay_us(uint32_t us) {
	uint32_t start_time_us = get_time_us();

	while (get_time_us() - start_time_us < us) {}
}
