# TM4C123GH6PM-nRF24L01

Driver library and example programs for using the nRF24L01+ 2.4 GHz transceiver with the TM4C123GXL development board or another device that uses the TM4C123GH6PM microcontroller.

## Requirements

  #### Hardware:
  - [TM4C123GXL](http://www.ti.com/tool/EK-TM4C123GXL) development board or another device using the [TM4C123GH6PM](http://www.ti.com/product/TM4C123GH6PM) microcontroller
  - nRF24L01+ 2.4GHz Tranceiver

  #### Software:
  - [TivaWare for C Series Software](http://www.ti.com/tool/SW-TM4C) from Texas Intruments
  
  ## Wiring
  
  | nRF24L01+  | TM4C123GXL |
  | ------- | ---- |
  | 1. GND  | GND  |
  | 2. 3V3  | 3V3  |
  | 3. CE   | PA6  |
  | 4. CSN  | PA7  |
  | 5. SCK  | PA2  |
  | 6. MOSI | PA5  |
  | 7. MISO | PA4  |
  | 8. IRQ  |  -   |

#### Resources:
- [nRF24L01+ datasheet](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf)
- [TM4C123GH6PM datasheet](http://www.ti.com/lit/ds/spms376e/spms376e.pdf)
