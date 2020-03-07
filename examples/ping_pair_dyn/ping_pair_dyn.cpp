/*
 * ping_pair_dyn.cpp
 * Ping pair example using dynamic payload sizes.
 *
 * written by: Kenneth Berry
 *
 * The program has two roles or states, ping out and pong back.
 * The role variable has to be changed to the desired role/state
 * TODO: Add ability to change role/state through commands
 * received through UART, and on-board button press.
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

// Payload variables
const int min_payload_size = 4;
const int max_payload_size = 32;
const int payload_size_increments_by = 1;
int next_payload_size = min_payload_size;
char receive_payload[max_payload_size + 1]; // +1 to allow room for a terminating NULL char

// Roles
const bool PING_OUT = 1;
const bool PONG_BACK = 0;
bool role;

// nRF24 Tranceiver class
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
    radio.enable_dynamic_payloads();
    radio.set_retries(5, 15);

    // Set role
    role = PING_OUT;
}

int main(void)
{
    setup();
    UARTprintf("\npingpair_dyn Example\n\n");

    if (role == PING_OUT)
    {
        radio.open_writing_pipe(pipes[0]);
        radio.open_reading_pipe(1, pipes[1]);
    }
    else
    {
        radio.open_writing_pipe(pipes[1]);
        radio.open_reading_pipe(1, pipes[0]);
        radio.start_listening();
    }

    UARTprintf("\nDetail Info\n");
    radio.print_details();

    // Main loop
    while(1)
    {
        /*
         * Ping Out role - send out packets and listen for response
         */
        if (role == PING_OUT)
        {
            // The payload will always be the same, what will change is how much of it we send.
            static char send_payload[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ789012";

            // First, stop listening so we can talk.
            radio.stop_listening();

            // Take the time, and send it.  This will block until complete
            UARTprintf("Now sending length %i...", next_payload_size);
            radio.write(send_payload, next_payload_size);

            // Now, continue listening
            radio.start_listening();

            // Wait here until we get a response, or timeout
            unsigned long started_waiting_at = get_runtime_milliseconds();
            bool timeout = false;
            while (!radio.available() && !timeout)
            {
                if (get_runtime_milliseconds() - started_waiting_at > 500)
                {
                    timeout = true;
                }
            }

            // Wait beofore sending next payload
            delay_milliseconds(100);

            if (timeout)
            {
                UARTprintf("Failed, response timed out.\n\r");
            }
            else
            {
                // Grab the response, compare, and send to debugging spew
                uint8_t len = radio.get_dynamic_payload_size();
                radio.read(receive_payload, len);

                // Put a zero at the end for easy printing
                receive_payload[len] = 0;

                // Spew it
                UARTprintf("Got response size=%i value=%s\n\r", len, receive_payload);
            }

            // Update size for next time.
            next_payload_size += payload_size_increments_by;
            if (next_payload_size > max_payload_size)
            {
                next_payload_size = min_payload_size;
            }

            // Try again after a delay
            delay_milliseconds(100);
        }

        /*
         * Pong Back role - receive each packet and send it back
         */
        if (role == PONG_BACK)
        {
            // If there is a data payload
            if (radio.available())
            {
                // Get payload size
                uint8_t len;
                len = radio.get_dynamic_payload_size();

                // Read payload
                radio.read(receive_payload, len);

                // Put a zero at the end for easy printing
                receive_payload[len] = 0;

                // First, stop listening so we can talk
                radio.stop_listening();

                UARTprintf("\nGot payload \n\r");
                UARTprintf(receive_payload);

                // Send the final one back.
                radio.write(receive_payload, len);
                UARTprintf("\nSent response\n\r");

                // Resume listening so we catch the next packets
                radio.start_listening();
            }
        }
    }
}

