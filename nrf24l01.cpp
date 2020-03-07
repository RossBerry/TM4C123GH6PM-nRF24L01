/*
  Copyright (C) 2020 Kenneth Berry <rosskberry@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.
*/

#include <stdint.h>
#include <string.h>
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "nrf24l01.h"
#include "nrf24_utils.h"

/********* Constants *********/

static const char CRC_LENGTH_0[] = "Disabled";
static const char CRC_LENGTH_1[] = "8 bits";
static const char CRC_LENGTH_2[] = "16 bits" ;
static const char * const CRC_LENGTHS[] =
{
    CRC_LENGTH_0,
    CRC_LENGTH_1,
    CRC_LENGTH_2,
};

static const char DATA_RATE_0[] = "1MBPS";
static const char DATA_RATE_1[] = "2MBPS";
static const char DATA_RATE_2[] = "250KBPS";
static const char * const DATA_RATES[] =
{
    DATA_RATE_0,
    DATA_RATE_1,
    DATA_RATE_2,
};

static const char MODEL_0[] = "nRF24L01";
static const char MODEL_1[] = "nRF24L01+";
static const char * const MODELS[] =
{
    MODEL_0,
    MODEL_1,
};

static const char PA_SETTING_0[] = "PA_MIN";
static const char PA_SETTING_1[] = "PA_LOW";
static const char PA_SETTING_2[] = "LA_MED";
static const char PA_SETTING_3[] = "PA_HIGH";
static const char * const PA_SETTINGS[] =
{
    PA_SETTING_0,
    PA_SETTING_1,
    PA_SETTING_2,
    PA_SETTING_3,
};

static const uint8_t CHILD_PIPE[] =
{
    RX_ADDR_P0,
    RX_ADDR_P1,
    RX_ADDR_P2,
    RX_ADDR_P3,
    RX_ADDR_P4,
    RX_ADDR_P5
};

static const uint8_t CHILD_PAYLOAD_SIZE[] =
{
    RX_PW_P0,
    RX_PW_P1,
    RX_PW_P2,
    RX_PW_P3,
    RX_PW_P4,
    RX_PW_P5
};

static const uint8_t CHILD_PIPE_ENABLE[] =
{
    ERX_P0,
    ERX_P1,
    ERX_P2,
    ERX_P3,
    ERX_P4,
    ERX_P5
};

/********* Protected Member Functions *********/

/**
 * Empty the receive buffer
 *
 * @return Current value of status register
 */
uint8_t NRF24L01::flush_rx(void)
{
    uint8_t status;

    set_csn(LOW);
    status = spi_transfer(FLUSH_RX);
    set_csn(HIGH);

    return status;
}

/**
 * Empty the transmit buffer
 *
 * @return Current value of status register
 */
uint8_t NRF24L01::flush_tx(void)
{
    uint8_t status;

    set_csn(LOW);
    status = spi_transfer(FLUSH_TX);
    set_csn(HIGH);

    return status;
}

/**
 * Retrieve the current status of the nRF24 chip
 *
 * @return Current value of status register
 */
uint8_t NRF24L01::get_chip_status(void)
{
    uint8_t chip_status;

    set_csn(LOW);
    chip_status = spi_transfer(NOP);
    set_csn(HIGH);

    return chip_status;
}

/**
 * Enable control pins.
 */
void NRF24L01::init_pins(void)
{
    // Enable SSI peripheral
    SysCtlPeripheralEnable(spi_periph);

    uint32_t data_rx;

    // Enable GPIO peripheral and Configure pin muxing for SPI pins
    if (spi_periph == SYSCTL_PERIPH_SSI0)
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinConfigure(GPIO_PA2_SSI0CLK);
        GPIOPinConfigure(GPIO_PA3_SSI0FSS);
        GPIOPinConfigure(GPIO_PA4_SSI0RX);
        GPIOPinConfigure(GPIO_PA5_SSI0TX);
        GPIOPinTypeSSI(GPIO_PORTA_BASE,
                           GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
        ssi_base = SSI0_BASE;
    }
    else if (spi_periph == SYSCTL_PERIPH_SSI1)
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinConfigure(GPIO_PF2_SSI1CLK);
        GPIOPinConfigure(GPIO_PF3_SSI1FSS);
        GPIOPinConfigure(GPIO_PF0_SSI1RX);
        GPIOPinConfigure(GPIO_PF1_SSI1TX);
        GPIOPinTypeSSI(GPIO_PORTF_BASE,
                           GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_2);
        ssi_base = SSI1_BASE;
    }
    else if (spi_periph == SYSCTL_PERIPH_SSI2)
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinConfigure(GPIO_PB4_SSI2CLK);
        GPIOPinConfigure(GPIO_PB5_SSI2FSS);
        GPIOPinConfigure(GPIO_PB6_SSI2RX);
        GPIOPinConfigure(GPIO_PB7_SSI2TX);
        GPIOPinTypeSSI(GPIO_PORTB_BASE,
                           GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);
        ssi_base = SSI2_BASE;
    }
    else if (spi_periph == SYSCTL_PERIPH_SSI3)
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        GPIOPinConfigure(GPIO_PD0_SSI3CLK);
        GPIOPinConfigure(GPIO_PD1_SSI3FSS);
        GPIOPinConfigure(GPIO_PD2_SSI3RX);
        GPIOPinConfigure(GPIO_PD3_SSI3TX);
        GPIOPinTypeSSI(GPIO_PORTD_BASE,
                           GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
        ssi_base = SSI3_BASE;
    }

    // Configure and enable the SSI port for TI master mode.  Use SSI0, system
    // clock supply, master mode, 1MHz SSI frequency, and 8-bit data.
    //
    SSIConfigSetExpClk(ssi_base, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    // Enable the SSI module.
    SSIEnable(ssi_base);

    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the TI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(ssi_base, &data_rx)) {}

    // Enable the GPIO ports for CE and CSN pins
    SysCtlPeripheralEnable(ce_periph);
    SysCtlPeripheralEnable(csn_periph);

    // Enable the GPIO pins for the CE and CSN pins
    GPIOPinTypeGPIOOutput(ce_pin_base, ce_pin);
    GPIOPinTypeGPIOOutput(csn_pin_base, csn_pin);
}

/**
 * Print the name and value of a 40-bit address register to UART
 *
 * @param name Name of the register
 * @param reg Which register. Use constants from nRF24L01.h
 * @param qty How many successive registers to print
 *
 * @warning Does nothing if uart_enabled is false.
 */
void NRF24L01::print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
    if (uart_enabled)
    {
        char extra_tab = strlen(name) < 8 ? '\t' : 0;
        UARTprintf("%s\t%c =", name, extra_tab);
        while (qty--)
        {
            uint8_t buffer[5];
            read_register(reg++, buffer, sizeof buffer);

            UARTprintf(" 0x");
            uint8_t* bufptr = buffer + sizeof buffer;
            while (--bufptr >= buffer)
            {
                UARTprintf("%02x", *bufptr);
            }
        }
        UARTprintf("\r\n");
    }
}

/**
 * Print the name and value of an 8-bit register to UART
 *
 * @param name Name of the register
 * @param reg Which register. Use constants from nRF24L01.h
 * @param qty How many successive registers to print
 *
 * @warning Does nothing if uart_enabled is false.
 */
void NRF24L01::print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
    if (uart_enabled)
    {
        char extra_tab = strlen(name) < 8 ? '\t' : 0;
        UARTprintf("%s\t%c =", name, extra_tab);
        while (qty--)
        {
            UARTprintf(" 0x%02x", read_register(reg++));
        }
        UARTprintf("\r\n");
    }
}

/**
 * Decode and print the given status to UART
 *
 * @param chip_status Status value to print
 *
 * @warning Does nothing if uart_enabled is false.
 */
void NRF24L01::print_chip_status(uint8_t chip_status)
{
    if (uart_enabled)
    {
        UARTprintf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
                   chip_status,
                   (chip_status & (1 << RX_DR)) ? 1 : 0,
                   (chip_status & (1 << TX_DS)) ? 1 : 0,
                   (chip_status & (1 << MAX_RT)) ? 1 : 0,
                   ((chip_status >> RX_P_NO) & 0x7),
                   (chip_status & (1 << TX_FULL)) ? 1 : 0);
    }
}

/**
 * Decode and print the given 'observe_tx' value to UART
 *
 * @param value The observe_tx value to print
 *
 * @warning Does nothing if uart_enabled is false.
 */
void NRF24L01::print_observe_tx(uint8_t value)
{
    if (uart_enabled)
    {
        UARTprintf("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n",
                 value,
                 (value >> PLOS_CNT) & 0xF,
                 (value >> ARC_CNT) & 0xF);
    }
}

/**
 * Read the receive payload
 *
 * The size of data read is the fixed payload size, see get_payload_size()
 *
 * @param buf Where to put the data
 * @param len Maximum number of bytes to read
 * @return Current value of status register
 */
uint8_t NRF24L01::read_payload(void* buf, uint8_t len)
{
    uint8_t status;
    uint8_t* current = reinterpret_cast<uint8_t*>(buf);
    uint8_t data_len = min(len,payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    set_csn(LOW);
    status = spi_transfer(R_RX_PAYLOAD);
    while (data_len--)
    {
        *current++ = spi_transfer(0xff);
    }
    while (blank_len--)
    {
        spi_transfer(0xff);
    }
    set_csn(HIGH);

    return status;
}

/**
 * Read single byte from a register
 *
 * @param reg Which register.
 * @return Current value of register @p reg
 */
uint8_t NRF24L01::read_register(uint8_t reg)
{
    set_csn(LOW);
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t result = spi_transfer(0xff);
    set_csn(HIGH);

    return result;
}

/**
 * Read a chunk of data in from a register
 *
 * @param reg Which register.
 * @param buf Where to put the data
 * @param len How many bytes of data to transfer
 * @return Current value of status register
 */
uint8_t NRF24L01::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t status;

    set_csn(LOW);
    status = spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    while (len--)
    {
        *buf++ = spi_transfer(0xff);
    }
    set_csn(HIGH);

    return status;
}

/**
 * Set chip enable
 *
 * @param level HIGH to actively begin transmission or LOW to put in standby.
 */
void NRF24L01::set_ce(int level)
{
    GPIOPinWrite(ce_pin_base, ce_pin, (level ? ce_pin : 0));
}

/**
 * Set chip select pin
 *
 * @param mode HIGH to take this unit off the SPI bus, LOW to put it on
 */
void NRF24L01::set_csn(int mode)
{
    // Minimum ideal SPI bus speed is 2x data rate
    // If we assume 2Mbs data rate and 16Mhz clock, a
    // divider of 4 is the minimum we want.
    // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
    GPIOPinWrite(csn_pin_base, csn_pin, (mode ? csn_pin : 0));
}

/**
 * Transfer data to and from nRF24 chip via SPI
 *
 * @param data The data to send to chip over SPI
 * @return The data received from the chip over SPI
 */
uint32_t NRF24L01::spi_transfer(uint32_t data)
{
    SSIDataPut(ssi_base, data);

    if(!SSIDataGetNonBlocking(ssi_base, &data))
    {
        SSIDataGet(ssi_base, &data);
    }

    return data & 0xFF;
}

/**
 * Turn on or off the special features of the nRF24 chip
 *
 * The chip has certain features which are only available when the features
 * are enabled.  See the datasheet for details.
 */
void NRF24L01::toggle_features(void)
{
    set_csn(LOW);
    spi_transfer(ACTIVATE);
    spi_transfer(0x73);
    set_csn(HIGH);
}

/**
 * Write the transmit payload
 *
 * The size of data written is the fixed payload size, see get_payload_size()
 *
 * @param buf Where to get the data
 * @param len Number of bytes to be sent
 * @return Current value of status register
 */
uint8_t NRF24L01::write_payload(const void* buf, uint8_t len)
{
    uint8_t status;
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);
    uint8_t data_len = min(len,payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    set_csn(LOW);
    status = spi_transfer(W_TX_PAYLOAD);
    while (data_len--)
    {
        spi_transfer(*current++);
    }
    while (blank_len--)
    {
        spi_transfer(0);
    }
    set_csn(HIGH);

    return status;
}

/**
 * Write a single byte to a register
 *
 * @param reg Which register.
 * @param value The new value to write
 * @return Current value of status register
 */
uint8_t NRF24L01::write_register(uint8_t reg, uint8_t value)
{
    uint8_t status;

    set_csn(LOW);
    status = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
    set_csn(HIGH);

    return status;
}

/**
 * Write a chunk of data to a register
 *
 * @param reg Which register.
 * @param buf Where to get the data
 * @param len How many bytes of data to transfer
 * @return Current value of status register
 */
uint8_t NRF24L01::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    uint8_t status;

    set_csn(LOW);
    status = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    while (len--)
    {
        spi_transfer(*buf++);
    }
    set_csn(HIGH);

    return status;
}

/********* Public Member Functions *********/

/**
 * NRF24L01 Constructor
 *
 * @param _ce_periph The peripheral to enable
 * @param _ce_pin_base The base address of the GPIO port
 * @param _ce_pin The pin attached to Chip Enable on the RF module
 * @oaran _csn_periph The peripheral to enable
 * @param _csn_pin_base The base address of the GPIO port
 * @param _csn_pin The pin attached to Chip Select
 * @param _spi_periph The SPI/SSI peripheral pins to enable (SPI0-SPI3)
 */
NRF24L01::NRF24L01(uint32_t _ce_periph, uint32_t _ce_pin_base, uint8_t _ce_pin,
             uint32_t _csn_periph, uint32_t _csn_pin_base, uint8_t _csn_pin,
             uint32_t _spi_periph)
	         : ce_periph(_ce_periph),
	           ce_pin_base(_ce_pin_base),
	           ce_pin(_ce_pin),
	           csn_periph(_csn_periph),
	           csn_pin_base(_csn_pin_base),
	           csn_pin(_csn_pin),
	           spi_periph(_spi_periph),
	           wide_band(true),
	           p_variant(false),
	           payload_size(32),
	           ack_payload_available(false),
	           dynamic_payloads_enabled(false),
	           pipe0_reading_address(0) {}

/**
 * Test whether there are bytes available to be read
 *
 * @return True if there is a payload available, false if none
 */
bool NRF24L01::available(void)
{
    return available(NULL);
}

/**
 * Test whether there are bytes available to be read
 *
 * Use this function to discover on which pipe the message
 * arrived.
 *
 * @param[out] pipe_num Which pipe has the payload available
 * @return True if there is a payload available, false if none is
 */
bool NRF24L01::available(uint8_t* pipe_num)
{
    uint8_t chip_status = get_chip_status();


    bool result = (chip_status & (1 << RX_DR));

    if (result)
    {
        // Include pipe number if it exists
        if (pipe_num)
        {
            *pipe_num = (chip_status >> RX_P_NO) & 0x7;
        }

        // Handle ack payload receipt
        if (chip_status & (1 << TX_DS))
        {
            write_register(STATUS, (1 << TX_DS));
        }
    }

    return result;
}

/**
 * Begin operation of the chip
 *
 * @warning Call this before calling any other methods.
 */
void NRF24L01::begin(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();

    init_pins();

    set_ce(LOW);
    set_csn(HIGH);

    // Must allow the radio time to settle else configuration bits may not bet set.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // 4.5ms + 14us is required, but delay for 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay_milliseconds(5);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts.
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See datasheet for a more complete explanation.
    write_register(SETUP_RETR, (0x4 << ARD) | (0xF << ARC));

    // Restore default PA level
    set_pa_level(NRF24L01_PA_MAX);

    // Determine if this is a p or non-p RF24 module and then
    // reset data rate back to default value. This works
    // because a non-P variant won't allow the data rate to
    // be set to 250Kbps.
    if (set_data_rate(NRF24L01_250KBPS))
    {
        p_variant = true;
    }

    // Set the data rate to the slowest (and most reliable) speed supported by
    // P-variant and non-P models.
    set_data_rate(NRF24L01_1MBPS);

    // Initialize CRC and request 2-byte (16bit) CRC
    set_crc_length(NRF24L01_CRC_16);

    // Disable dynamic payloads, to match dynamic_payloads_enabled setting
    write_register(DYNPD, 0);

    // Reset current status
    write_register(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    // Set default channel configuration.
    set_channel(76);

    // Flush buffers
    flush_rx();
    flush_tx();
}

/**
 * Disable CRC validation
 *
 */
void NRF24L01::disable_crc(void)
{
    uint8_t disable = read_register(CONFIG) & ~(1 << EN_CRC);
    write_register(CONFIG, disable);
}

/**
 * Enable custom payloads on the acknowledge packets
 *
 * Ack payloads are a handy way to return data back to senders without
 * manually changing the radio modes on both units.
 */
void NRF24L01::enable_ack_payload(void)
{
    // Enable ack payload and dynamic payload features
    write_register(FEATURE, read_register(FEATURE) | (1 << EN_ACK_PAY) | (1 << EN_DPL));

    // Check if features are enabled
    if (!read_register(FEATURE))
    {
        // Enable features and try again
        toggle_features();
        write_register(FEATURE, read_register(FEATURE) | (1 << EN_ACK_PAY) | (1 << EN_DPL) );
    }

    // Enable dynamic payload on pipes 0 and 1
    write_register(DYNPD, read_register(DYNPD) | (1 << DPL_P1) | (1 << DPL_P0));
}

/**
 * Enable dynamically-sized payloads
 *
 * This way you don't always have to send large packets.
 *
 * @warning This enables dynamic payloads on ALL pipes.
 */
void NRF24L01::enable_dynamic_payloads(void)
{
    // Enable dynamic payload
    write_register(FEATURE,read_register(FEATURE) | (1 << EN_DPL));

    // Check if feature is enabled
    if (!read_register(FEATURE))
    {
        // Enable features and try again
        toggle_features();
        write_register(FEATURE, read_register(FEATURE) | (1 << EN_DPL));
    }

    // Enable dynamic payload on all pipes.
    write_register(DYNPD,
                   read_register(DYNPD)
                   | (1 << DPL_P5)
                   | (1 << DPL_P4)
                   | (1 << DPL_P3)
                   | (1 << DPL_P2)
                   | (1 << DPL_P1)
                   | (1 << DPL_P0));
    dynamic_payloads_enabled = true;
}

/**
 * Get the CRC length
 *
 * @return NRF24L01_DISABLED if disabled or NRF24L01_CRC_8 for 8-bit or NRF24L01_CRC_16 for 16-bit
 */
nrf24l01_crc_length NRF24L01::get_crc_length(void)
{
    nrf24l01_crc_length result = NRF24L01_CRC_DISABLED;
    uint8_t config = read_register(CONFIG) & ((1 << CRCO) | (1 << EN_CRC));

    if (config & (1 << EN_CRC ))
    {
        if (config & (1 << CRCO))
        {
            result = NRF24L01_CRC_16;
        }
        else
        {
            result = NRF24L01_CRC_8;
        }
    }

    return result;
}

/**
 * Fetches the transmission data rate
 *
 * @return Returns the hardware's currently configured data rate. The value
 * is either 250kbs, NRF24L01_1MBPS for 1Mbps, or NRF24L01_2MBPS; as defined in the
 * nrf24l01_data_rate enum.
 */
nrf24l01_data_rate NRF24L01::get_data_rate(void)
{
    nrf24l01_data_rate result ;
    uint8_t data_rate = read_register(RF_SETUP) & ((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));

    if (data_rate == (1 << RF_DR_LOW))
    {
        // '10' = 250KBPS
        result = NRF24L01_250KBPS;
    }
    else if (data_rate == (1 << RF_DR_HIGH))
    {
        // '01' = 2MBPS
        result = NRF24L01_2MBPS;
    }
    else
    {
        // '00' = 1MBPS
        result = NRF24L01_1MBPS;
    }

    return result;
}

/**
 * Get Dynamic Payload Size
 *
 * For dynamic payloads, this pulls the size of the payload off
 * the chip
 *
 * @return Payload length of last-received dynamic payload
 */
uint8_t NRF24L01::get_dynamic_payload_size(void)
{
    uint8_t result = 0;

    set_csn(LOW);
    spi_transfer(R_RX_PL_WID);
    result = spi_transfer(0xff);
    set_csn(HIGH);

    return result;
}

/**
 * Fetches the current PA level.
 *
 * @return Returns a value from the nrf24l01_pa_dbm enum describing
 * the current PA setting. Please remember, all values represented
 * by the enum mnemonics are negative dBm. See set_pa_level() for
 * return value descriptions.
 */
nrf24l01_pa_dbm NRF24L01::get_pa_level(void)
{
    nrf24l01_pa_dbm result = NRF24L01_PA_ERROR;
    uint8_t power = read_register(RF_SETUP) & ((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH));

    if (power == ((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH)))
    {
        result = NRF24L01_PA_MAX;
    }
    else if (power == (1 << RF_PWR_HIGH))
    {
        result = NRF24L01_PA_HIGH;
    }
    else if (power == (1 << RF_PWR_LOW))
    {
        result = NRF24L01_PA_LOW;
    }
    else
    {
        result = NRF24L01_PA_MIN;
    }

    return result;
}

/**
 * Get Static Payload Size
 *
 * @see set_payload_size()
 *
 * @return The number of bytes in the payload
 */
uint8_t NRF24L01::get_payload_size(void)
{
    return payload_size;
}

/**
 * Determine if an ack payload was received in the most recent call to
 * write().
 *
 * Call read() to retrieve the ack payload.
 *
 * @return True if an ack payload is available.
 */
bool NRF24L01::is_ack_payload_available(void)
{
    bool result = ack_payload_available;
    ack_payload_available = false;

    return result;
}

/**
 * Determine whether the hardware is listening or not.
 *
 * @return true if the hardware is listening and false if its not.
 */
bool NRF24L01::is_listening(void)
{
    return listening;
}

/**
 * Determine whether the hardware is an nRF24L01+ or not.
 *
 * @return true if the hardware is nRF24L01+ (or compatible) and false
 * if its not.
 */
bool NRF24L01::is_p_variant(void)
{
    return p_variant;
}

/**
 * Open a pipe for reading
 *
 * Up to 6 pipes can be open for reading at once.  Open all
 * reading pipes, and then call start_listening().
 *
 * @see open_writing_pipe()
 *
 * @warning Pipes 1-5 should share the first 32 bits.
 * Only the least significant byte should be unique, e.g.
 * @code
 *   open_reading_pipe(1, 0xF0F0F0F0AA);
 *   open_reading_pipe(2, 0xF0F0F0F066);
 * @endcode
 *
 * @warning Pipe 0 is also used by the writing pipe.  So if you open
 * pipe 0 for reading, and then start_listening(), it will overwrite the
 * writing pipe.
 *
 * @TODO Enforce the restriction that pipes 1-5 must share the top 32 bits
 *
 * @param child Which pipe# to open, 0-5.
 * @param address The 40-bit address of the pipe to open.
 */
void NRF24L01::open_reading_pipe(uint8_t child, uint64_t address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // open_writing_pipe() will overwrite the pipe 0 address, so
    // start_listening() will have to restore it.
    if (child == 0)
    {
        pipe0_reading_address = address;
    }
    if (child <= 6)
    {
        // For pipes 2-5, only write the LSB
        if ( child < 2 )
        {
            write_register(*(&CHILD_PIPE[child]), reinterpret_cast<const uint8_t*>(&address), 5);
        }
        else
        {
            write_register(*(&CHILD_PIPE[child]), reinterpret_cast<const uint8_t*>(&address), 1);
        }
        write_register(*(&CHILD_PAYLOAD_SIZE[child]),payload_size);

        // Set bits for open pipes
        write_register(EN_RXADDR, read_register(EN_RXADDR) | (1 << *(&CHILD_PIPE_ENABLE[child])));
    }
}

/**
 * Open a pipe for writing
 *
 * Only one pipe can be open at once, but you can change the pipe
 * you'll listen to.  Do not call this while actively listening, and
 * remember to stop_listening() first.
 *
 * Addresses are 40-bit hex values, e.g.:
 * @code
 *   open_writing_pipe(0xF0F0F0F0F0);
 * @endcode
 *
 * @param address The 40-bit address of the pipe to open.  This can be
 * any value whatsoever, as long as you are the only one writing to it
 * and only one other radio is listening to it.  Coordinate these pipe
 * addresses amongst nodes on the network.
 */
void NRF24L01::open_writing_pipe(uint64_t address)
{
    write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&address), 5);
    write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&address), 5);

    const uint8_t max_payload_size = 32;
    write_register(RX_PW_P0,min(payload_size, max_payload_size));
}

/**
 * Enter low-power mode
 *
 * To return to normal power mode, either write() some data or
 * start_listening(), or power_up().
 */
void NRF24L01::power_down(void)
{
    write_register(CONFIG, read_register(CONFIG) & ~(1 << PWR_UP));
}

/**
 * Leave low-power mode - making radio more responsive
 *
 * To return to low power mode, call power_down().
 */
void NRF24L01::power_up(void)
{
    write_register(CONFIG, read_register(CONFIG) | (1 << PWR_UP));
}

/**
 * Print debugging information to UART
 *
 * @warning Does nothing if uart_enabled false.
 */
void NRF24L01::print_details(void)
{
    if (uart_enabled)
    {
        print_chip_status(get_chip_status());

        print_address_register("RX_ADDR_P0-1", RX_ADDR_P0, 2);
        print_byte_register("RX_ADDR_P2-5", RX_ADDR_P2, 4);
        print_address_register("TX_ADDR", TX_ADDR);

        print_byte_register("RX_PW_P0-6", RX_PW_P0, 6);
        print_byte_register("EN_AA", EN_AA);
        print_byte_register("EN_RXADDR", EN_RXADDR);
        print_byte_register("RF_CH", RF_CH);
        print_byte_register("RF_SETUP", RF_SETUP);
        print_byte_register("CONFIG", CONFIG);
        print_byte_register("DYNPD/FEATURE", DYNPD, 2);

        UARTprintf("Data Rate\t = %s\r\n", *(&DATA_RATES[get_data_rate()]));
        UARTprintf("Model\t\t = %s\r\n", *(&MODELS[is_p_variant()]));
        UARTprintf("CRC Length\t = %s\r\n", *(&CRC_LENGTHS[get_crc_length()]));
        UARTprintf("PA Power\t = %s\r\n", *(&PA_SETTINGS[get_pa_level()]));
    }
}

/**
 * Read the payload
 *
 * Return the last payload received
 *
 * The size of data read is the fixed payload size, see get_payload_size()
 *
 * @param buf Pointer to a buffer where the data should be written
 * @param len Maximum number of bytes to read into the buffer
 * @return True if the payload was delivered successfully false if not
 */
bool NRF24L01::read(void* buf, uint8_t len)
{
    // Fetch the payload
    read_payload(buf, len);

    uint8_t chip_status = get_chip_status();

    // Clear the status bit
    write_register(STATUS, (1 << RX_DR));

    return read_register(FIFO_STATUS) & (1 << RX_EMPTY);
}

/**
 * Enable or disable auto-acknowlede packets
 *
 * This is enabled by default, so it's only needed if you want to turn
 * it off for some reason.
 *
 * @param enable Whether to enable (true) or disable (false)
 */
void NRF24L01::set_auto_ack(bool enable)
{
    if (enable)
    {
        write_register(EN_AA, 0x3F);
    }
    else
    {
        write_register(EN_AA, 0);
    }
}

/**
 * Enable or disable auto-acknowlede packets on a per pipeline basis.
 *
 * Auto-acknowlede is enabled by default, so it's only needed if you want
 * to turn it off/on for some reason on a per pipeline basis.
 *
 * @param pipe Which pipeline to modify
 * @param enable Whether to enable (true) or disable (false) auto-acks
 */
void NRF24L01::set_auto_ack(uint8_t pipe, bool enable)
{
    if (pipe <= 6)
    {
        uint8_t en_aa = read_register(EN_AA);
        if(enable)
        {
            en_aa |= (1 << pipe);
        }
        else
        {
            en_aa &= ~(1 << pipe);
        }
        write_register(EN_AA, en_aa);
    }
}

/**
 * Set RF communication channel
 *
 * @param channel Which RF channel to communicate on, 0-127
 */
void NRF24L01::set_channel(uint8_t channel)
{
    // TODO: This method could take advantage of the 'wide_band' calculation
    // done in set_channel() to require certain channel spacing.
    const uint8_t max_channel = 127;
    write_register(RF_CH, min(channel, max_channel));
}

/**
 * Set the CRC length
 *
 * @param length NRF24L01_CRC_8 for 8-bit or NRF24L01_CRC_16 for 16-bit
 */
void NRF24L01::set_crc_length(nrf24l01_crc_length length)
{
    uint8_t config = read_register(CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));

    if (length == NRF24L01_CRC_DISABLED)
    {
        // Do nothing
    }
    else if (length == NRF24L01_CRC_8)
    {
        config |= (1 << EN_CRC);
    }
    else
    {
        config |= (1 << EN_CRC);
        config |= (1 << CRCO);
    }
    write_register(CONFIG, config);
}

/**
 * Set the transmission data rate
 *
 * @warning setting NRF24L01_250KBPS will fail for nRF24L01
 *
 * @param speed NRF24L01_250KBPS for 250kbs, NRF24L01_1MBPS for 1Mbps, or NRF24L01_2MBPS for 2Mbps
 * @return true if the change was successful
 */
bool NRF24L01::set_data_rate(nrf24l01_data_rate speed)
{
    bool result = false;
    uint8_t setup = read_register(RF_SETUP);

    wide_band = false ;
    setup &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));
    if(speed == NRF24L01_250KBPS)
    {
        // Set the RF_DR_LOW to 1 (250 Kbs)
        wide_band = false ;
        setup |= (1 << RF_DR_LOW);
    }
    else
    {

        if (speed == NRF24L01_2MBPS)
        {   // Set RF_DR_HIGH to 1 (2Mbs)
            wide_band = true ;
            setup |= (1 << RF_DR_HIGH);
        }
        else
        {
            // 1Mbs
            wide_band = false;
        }
    }
    write_register(RF_SETUP,setup);

    // Verify result
    if (read_register(RF_SETUP) == setup)
    {
        result = true;
    }
    else
    {
        wide_band = false;
    }

    return result;
}

/**
 * Set Power Amplifier (PA) level to one of four levels.
 *
 * NRF24L01_PA_MIN=-18dBm, NRF24L01_PA_LOW=-12dBm,
 * NRF24L01_PA_MED=-6dBM, and NRF24L01_PA_HIGH=0dBm.
 *
 * @param level Desired PA level.
 */
void NRF24L01::set_pa_level(nrf24l01_pa_dbm level)
{
    uint8_t setup = read_register(RF_SETUP);
    setup &= ~((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH));

    if (level == NRF24L01_PA_MAX)
    {
        setup |= ((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH));
    }
    else if (level == NRF24L01_PA_HIGH)
    {
        setup |= (1 << RF_PWR_HIGH);
    }
    else if (level == NRF24L01_PA_LOW)
    {
        setup |= (1 << RF_PWR_LOW);
    }
    else if (level == NRF24L01_PA_MIN)
    {
        // Do nothing
    }
    else if (level == NRF24L01_PA_ERROR)
    {
        // If error, go to maximum PA
        setup |= ((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH));
    }

    write_register(RF_SETUP, setup);
}

/**
 * Set Static Payload Size
 *
 * If this method is never called, the driver will always
 * transmit the maximum payload size (32 bytes), no matter how much
 * was sent to write().
 *
 * @param size The number of bytes in the payload
 */
void NRF24L01::set_payload_size(uint8_t size)
{
    const uint8_t max_payload_size = 32;
    payload_size = min(size, max_payload_size);
}

/**
 * Set the number and delay of retries upon failed submit
 *
 * @param delay How long to wait between each retry, in multiples of 250us,
 * max is 15.  0 means 250us, 15 means 4000us.
 * @param count How many retries before giving up, max 15
 */
void NRF24L01::set_retries(uint8_t delay, uint8_t count)
{
    write_register(SETUP_RETR, (delay & 0xf)<<ARD | (count & 0xf)<<ARC);
}

/**
 * Start listening on the pipes opened for reading.
 *
 * Be sure to call open_reading_pipe() first.  Do not call write() while
 * in this mode, without first calling stop_listening().  Call
 * available() to check for incoming traffic, and read() to get it.
 */
void NRF24L01::start_listening(void)
{
    write_register(CONFIG, read_register(CONFIG) | (1 << PWR_UP) | (1 << PRIM_RX));
    write_register(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    // Restore the pipe0 address, if it exists
    if (pipe0_reading_address)
    {
        write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&pipe0_reading_address), 5);
    }

    // Flush buffers
    flush_rx();
    flush_tx();

    // Enable radio
    set_ce(HIGH);

    listening = true;

    // Wait for the radio to start up (130us actually only needed)
    delay_microseconds(130);
}

/**
 * Non-blocking write to the open writing pipe
 *
 * Just like write(), but it returns immediately.
 *
 * @see write()
 *
 * @param buf Pointer to the data to be sent
 * @param len Number of bytes to be sent
 * @return True if the payload was delivered successfully false if not
 */
void NRF24L01::start_write( const void* buf, uint8_t len )
{
    // Power-up transmitter
    write_register(CONFIG, (read_register(CONFIG) | (1 << PWR_UP) ) & ~(1 << PRIM_RX));
    delay_microseconds(150);

    // Send the payload
    write_payload(buf, len);

    // Enable radio and wait for payload to be transmitted
    set_ce(HIGH);
    delay_microseconds(15);

    // Disable radio
    set_ce(LOW);
}

/**
 * Stop listening for incoming messages
 *
 * Do this before calling write().
 */
void NRF24L01::stop_listening(void)
{
    set_ce(LOW);
    flush_tx();
    flush_rx();
}

/**
 * Test whether there was a carrier on the line for the
 * previous listening period.
 *
 * Useful to check for interference on the current channel.
 *
 * @return true if was carrier, false if not
 */
bool NRF24L01::test_carrier(void)
{
    return (read_register(CD) & 1);
}

/**
 * Test whether a signal (carrier or otherwise) greater than
 * or equal to -64dBm is present on the channel. Valid only
 * on nRF24L01P (+) hardware. On nRF24L01, use test_carrier().
 *
 * Useful to check for interference on the current channel and
 * channel hopping strategies.
 *
 * @return true if signal => -64dBm, false if not
 */
bool NRF24L01::test_rpd(void)
{
    return (read_register(RPD) & 1);
}

/**
 * Write to the open writing pipe
 *
 * Be sure to call open_writing_pipe() first to set the destination
 * of where to write to.
 *
 * This blocks until the message is successfully acknowledged by
 * the receiver or the timeout/retransmit limit is reached.  In
 * the current configuration, the max delay here is 60ms.
 *
 * The maximum size of data written is the fixed payload size, see
 * get_payload_size().  However, you can write less, and the remainder
 * will just be filled with zeroes.
 *
 * @param buf Pointer to the data to be sent
 * @param len Number of bytes to be sent
 * @return True if the payload was delivered successfully false if not
 */
bool NRF24L01::write(const void* buf, uint8_t len)
{
    // Begin the write
    start_write(buf, len);

    // Monitor the send
    uint8_t observe_tx;
    uint8_t status;
    uint32_t sent_at = get_runtime_milliseconds();
    const uint32_t timeout = 500; //ms to wait for timeout
    do
    {
        status = read_register(OBSERVE_TX, &observe_tx, 1);
    }
    while (!(status & ((1 << TX_DS) | (1 << MAX_RT)))
            && (get_runtime_milliseconds() - sent_at < timeout));

    // The status tells us three things
    // * The send was successful (TX_DS)
    // * The send failed, too many retries (MAX_RT)
    // * There is an ack packet waiting (RX_DR)
    // Update statuses
    bool tx_ok = status & (1 << TX_DS);
    bool tx_fail = status & (1 << MAX_RT);
    ack_payload_available = status & (1 << RX_DR);

    // Reset STATUS register if send failed
    if (tx_fail)
    {
        write_register(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    }

    // Get size of ack payload if available
    if (ack_payload_available)
    {
        ack_payload_length = get_dynamic_payload_size();
    }

    // Power down
    power_down();

    // Flush buffers
    flush_tx();

    return tx_ok;
}

/**
 * Write an ack payload for the specified pipe
 *
 * The next time a message is received on @p pipe, the data in @p buf will
 * be sent back in the acknowledgement.
 *
 * @warning According to the data sheet, only three of these can be pending
 * at any time.
 *
 * @param pipe Which pipe# (typically 1-5) will get this response.
 * @param buf Pointer to data that is sent
 * @param len Length of the data to send, up to 32 bytes max.  Not affected
 * by the static payload set by set_payload_size().
 */
void NRF24L01::write_ack_payload(uint8_t pipe, const void* buf, uint8_t len)
{
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    set_csn(LOW);
    spi_transfer(W_ACK_PAYLOAD | (pipe & 0x7));
    const uint8_t max_payload_size = 32;
    uint8_t data_len = min(len, max_payload_size);
    while (data_len--)
    {
        spi_transfer(*current);
    }
    set_csn(HIGH);
}
