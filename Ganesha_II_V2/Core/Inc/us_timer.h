/*
 * us_timer.h
 *
 *  Created on: Apr 27, 2026
 *      Author: Dhruv Shah
 */

#ifndef INC_US_TIMER_H_
#define INC_US_TIMER_H_

#include <stdint.h>

void us_timer_init();
uint32_t get_time_us();
void delay_us(uint32_t us);

#endif /* INC_US_TIMER_H_ */
