/*
  Copyright (C) 2020 Kenneth Berry <rosskberry@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.
*/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "nrf24_utils.h"

/**
 * Microsecond delay
 */
void delay_microseconds(uint32_t microseconds)
{
    SysCtlDelay(SysCtlClockGet() / 1000 / 1000 / 3 * microseconds);
}

/**
 * Millisecond delay
 */
void delay_milliseconds(uint32_t milliseconds)
{
	SysCtlDelay(SysCtlClockGet() / 1000 / 3 * milliseconds);
}

/**
 * Return the current runtime from the start of the run in milliseconds.
 */
uint64_t get_runtime_milliseconds(void)
{
	return ticks;
}

/**
 * Set up UART0 to be used for a console to display information.
 */
void init_console(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    // ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    // ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    // ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    // ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

    uart_enabled = true;
}

/**
 * Initialize SysTick
 */
void init_systick(void)
{
    //
    // Set the clocking to run directly from the external crystal/oscillator.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    SYSCTL_XTAL_16MHZ);

    //
    // Set up and enable the SysTick timer.  It will be used as a reference
    // for delay loops.  The SysTick timer period will be set up for one
    // microsecond.
    //
    SysTickPeriodSet(SysCtlClockGet()/1000);
    //
    // Enable interruptions, because we will use them for delays
    //
    SysTickIntEnable();
    SysTickEnable();
}

/**
 * Determine the minimum of two values.
 * @param val1 The first of the two values to compare
 * @param val2 The second of the two values to compare
 * @return The minimum of the two values
 */
uint8_t min(uint8_t val1, uint8_t val2)
{
    return val1 < val2 ? val1 : val2;
}

/**
 * SysTick interrupt handler.
 * The default setting is 1000 times per second (1ms period)
 */
void systick_interrupt_handler(void)
{
    ticks++;
}
