/*
 * ping_pair_ack.cpp
 * Ping pair example using auto-Acknowledge payloads
 *
 * written by: Kenneth Berry
 *
 * The program has two roles or states, ping out and pong back.
 * The role variable has to be changed to the desired role/state.
 * TODO: Add ability to change role/state through
 * commands received through UART, and on-board button
 * press.
 */

// Standard library
#include <cstdio>
#include <stdbool.h>
#include <stdint.h>
// TivaWare library
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"
// nRF24 driver library
#include "nrf24l01.h"
#include "nrf24_utils.h"

// Pins
const uint32_t CE_PERIPH = SYSCTL_PERIPH_GPIOA;
const uint32_t CE_PIN_BASE = GPIO_PORTA_BASE;
const uint8_t CE_PIN = GPIO_PIN_6;
const uint32_t CSN_PERIPH = SYSCTL_PERIPH_GPIOA;
const uint32_t CSN_PIN_BASE = GPIO_PORTA_BASE;
const uint8_t CSN_PIN = GPIO_PIN_7;
const uint32_t SPI_PERIPH = SYSCTL_PERIPH_SSI0;


// RF24 pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xABCDABCD11LL, 0xABCDABCD22LL};

// Roles
const bool PING_OUT = 1;
const bool PONG_BACK = 0;
bool role;

// A single byte to keep track of the data being sent back and forth
uint8_t counter = 1;

// nRF24 Tranceiver initialization
NRF24L01 radio(CE_PERIPH,
               CE_PIN_BASE,
               CE_PIN,
               CSN_PERIPH,
               CSN_PIN_BASE,
               CSN_PIN,
               SPI_PERIPH);

/**
 * Initialization
 */
void setup(void)
{
    init_systick();

    init_console();

    radio.begin();

    radio.set_auto_ack(true);
    radio.enable_ack_payload();
    radio.set_retries(5, 15);
    radio.set_payload_size(1);

    // Set role
    role = PING_OUT;

    if (role == PING_OUT)
    {
        radio.open_writing_pipe(pipes[0]);
        radio.open_reading_pipe(1, pipes[1]);
    }
    else
    {
        radio.open_writing_pipe(pipes[1]);
        radio.open_reading_pipe(1, pipes[0]);
    }

    radio.start_listening();
}

int main(void)
{
    setup();

    // Print preamble to UART
    UARTprintf("\npingpair_ack Example\n\n");

    UARTprintf("\nDetail Info\n");
    radio.print_details();

        // Main loop
        while(true)
        {
            /*
             * Ping Out role
             */
            if (role == PING_OUT)
            {
                radio.stop_listening();

                UARTprintf("Now sending %d as payload\n", counter);

                uint8_t got_byte;
                uint32_t send_time = get_runtime_milliseconds();

                if (!radio.write(&counter, 1))
                {
                    UARTprintf("Failed!\n");
                }
                else
                {
                    if (!radio.available())
                    {
                        UARTprintf("Blank Payload Received.\n");
                    }
                    else
                    {
                        while (radio.available())
                        {
                            uint32_t receive_time = get_runtime_milliseconds();
                            radio.read(&got_byte, 1);
                            UARTprintf("Got response %d, round-trip delay: %d ms\n\r", got_byte, receive_time - send_time);
                            counter++;
                        }
                    }
                }

                // Start listening again just in case role is switched.
                radio.start_listening();

                // Try again later
                delay_milliseconds(1000);
            }

        /*
         * Pong Back role
         */
        if (role == PONG_BACK)
        {
            uint8_t pipe_number;
            uint8_t got_byte;

            // If there is a data payload
            while (radio.available(&pipe_number))
            {
                radio.read(&got_byte, 1);
                radio.write_ack_payload(pipe_number, &got_byte, 1);
                UARTprintf("Received %d as payload\n", got_byte);
            }
        }
    }
}

