/*
  Copyright (C) 2020 Kenneth Berry <rosskberry@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.
*/

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

// Mnemonic Bit Map for nRF24L01 and nRF24L01P
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
#define AW 0
#define ARD 4
#define ARC 0
#define PLL_LOCK 4
#define RF_DR 3
#define RF_PWR 6
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define TX_FULL 0
#define PLOS_CNT 4
#define ARC_CNT 0
#define TX_REUSE 6
#define FIFO_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0
#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0
#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

// Non-nRF24L01P omissions
#define LNA_HCURR 0

// nRF24L01P model Mnemonic Bit Map
#define RF_DR_LOW 5
#define RF_DR_HIGH 3
#define RF_PWR_LOW 1
#define RF_PWR_HIGH 2

// Memory Map for nRF24L01 and nRF24L01P
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

// nRF24L01P model Memory Map
#define RPD 0x09

// Commands
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

/**
 * Power Amplifier levels.
 *
 * For use with set_pa_level()
 */
typedef enum { NRF24L01_PA_MIN = 0,
               NRF24L01_PA_LOW,
               NRF24L01_PA_HIGH,
               NRF24L01_PA_MAX,
               NRF24L01_PA_ERROR } nrf24l01_pa_dbm;

/**
 * Data rates.  How fast data moves through the air.
 *
 * For use with set_data_rate()
 */
typedef enum { NRF24L01_1MBPS = 0,
               NRF24L01_2MBPS,
               NRF24L01_250KBPS } nrf24l01_data_rate;

/**
 * CRC Lengths.  How big (if any) of a CRC is included.
 *
 * For use with set_crc_length()
 */
typedef enum { NRF24L01_CRC_DISABLED = 0,
               NRF24L01_CRC_8,
               NRF24L01_CRC_16 } nrf24l01_crc_length;

/**
 * Driver class for nRF24L01 and nRF24L01P  2.4GHz wireless transceivers
 */
class NRF24L01
{

private:
    // Pins
    uint32_t ce_periph;             // GPIO port for CE pin
    uint32_t ce_pin_base;           // Base address for CE pin
    uint8_t ce_pin;                 // Chip Enable (CE) pin
    uint32_t csn_periph;            // GPIO port for CSN pin
    uint32_t csn_pin_base;          // Base address for CSN pin
    uint8_t csn_pin;                // Chip select (CSN) pin
    uint32_t spi_periph;            // SPI/SSI periphial
    uint32_t ssi_base;              // Base address for SSI pins

    bool ack_payload_available;     // Whether there is an ack payload waiting
    uint8_t ack_payload_length;     // Dynamic size of pending ack payload
    bool dynamic_payloads_enabled;  // Whether dynamic payloads are enabled
    uint8_t payload_size;           // Fixed size of payload
    uint64_t pipe0_reading_address; // Last address set on pipe 0 for reading
    bool p_variant;                 // False for nRF24L01 and true for nRF24L01P
    bool wide_band;                 // 2Mbs data rate in use
    bool listening;                 // Whether the chip is currently listening


protected:

    uint8_t flush_rx(void);

    uint8_t flush_tx(void);

    uint8_t get_chip_status(void);

    void init_pins(void);

    void print_address_register(const char* name, uint8_t reg, uint8_t qty = 1);

    void print_byte_register(const char* name, uint8_t reg, uint8_t qty = 1);

    void print_chip_status(uint8_t chip_status);

    void print_observe_tx(uint8_t value);

    uint8_t read_payload(void* buf, uint8_t len);

    uint8_t read_register(uint8_t reg);

    uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);

    void set_ce(int level);

    void set_csn(int mode);

    uint32_t spi_transfer(uint32_t data);

    void toggle_features(void);

    uint8_t write_payload(const void* buf, uint8_t len);

    uint8_t write_register(uint8_t reg, uint8_t value);

    uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);


public:

    NRF24L01(uint32_t _ce_periph,
          uint32_t _ce_pin_base,
          uint8_t _ce_pin,
          uint32_t _csn_periph,
          uint32_t _csn_pin_base,
          uint8_t _csn_pin,
          uint32_t _spi_periph);

    bool available(void);

    bool available(uint8_t* pipe_num);

    void begin(void);

    void disable_crc(void);

    void enable_ack_payload(void);

    void enable_dynamic_payloads(void);

    nrf24l01_crc_length get_crc_length(void);

    nrf24l01_data_rate get_data_rate(void);

    uint8_t get_dynamic_payload_size(void);

    nrf24l01_pa_dbm get_pa_level(void);

    uint8_t get_payload_size(void);

    bool is_ack_payload_available(void);

    bool is_listening(void);

    bool is_p_variant(void);

    void open_reading_pipe(uint8_t number, uint64_t address);

    void open_writing_pipe(uint64_t address);

    void power_down(void);

    void power_up(void);

    void print_details(void);

    bool read(void* buf, uint8_t len);

    void set_auto_ack(bool enable);

    void set_auto_ack(uint8_t pipe, bool enable);

    void set_channel(uint8_t channel);

    void set_crc_length(nrf24l01_crc_length length);

    bool set_data_rate(nrf24l01_data_rate speed);

    void set_pa_level(nrf24l01_pa_dbm level);

    void set_payload_size(uint8_t size);

    void set_retries(uint8_t delay, uint8_t count);

    void start_listening(void);

    void start_write(const void* buf, uint8_t len);

    void stop_listening(void);

    bool test_carrier(void);

    bool test_rpd(void);

    bool write(const void* buf, uint8_t len);

    void write_ack_payload(uint8_t pipe, const void* buf, uint8_t len);
};

#endif // __NRF24L01_H__
