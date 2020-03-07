/*
  Copyright (C) 2020 Kenneth Berry <rosskberry@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.
*/

#ifndef __NRF24_UTILS_H__
#define __NRF24_UTILS_H__

enum { LOW = 0,
	   HIGH = 1 };

volatile uint64_t ticks;
bool uart_enabled;

void delay_microseconds(uint32_t microseconds);

void delay_milliseconds(uint32_t milliseconds);

uint64_t get_runtime_milliseconds(void);

void init_console(void);

void init_systick(void);

uint8_t min(uint8_t val1, uint8_t val2);

extern "C"
void systick_interrupt_handler(void);

#endif // __NRF24_UTILS_H__
