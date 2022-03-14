use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use log::info;
use stm32h7xx_hal::{device::I2C2, i2c::I2c};

use crate::delay_ms;

const W8731_ADDR: u8 = 0x1A;

const CODEC_REG_LEFT_LINE_IN: u8 = 0x00;
const CODEC_REG_RIGHT_LINE_IN: u8 = 0x01;
const CODEC_REG_LEFT_HEADPHONES_OUT: u8 = 0x02;
const CODEC_REG_RIGHT_HEADPHONES_OUT: u8 = 0x03;
const CODEC_REG_ANALOGUE_ROUTING: u8 = 0x04;
const CODEC_REG_DIGITAL_ROUTING: u8 = 0x05;
const CODEC_REG_POWER_MANAGEMENT: u8 = 0x06;
const CODEC_REG_DIGITAL_FORMAT: u8 = 0x07;
const CODEC_REG_SAMPLE_RATE: u8 = 0x08;
const CODEC_REG_ACTIVE: u8 = 0x09;
const CODEC_REG_RESET: u8 = 0x0f;

const CODEC_ANALOG_MICBOOST: u16 = 0x01;
const CODEC_ANALOG_MUTEMIC: u16 = 0x02;
const CODEC_ANALOG_INSEL: u16 = 0x04;
const CODEC_ANALOG_BYPASS: u16 = 0x08;
const CODEC_ANALOG_DACSEL: u16 = 0x10;
const CODEC_ANALOG_SIDETONE: u16 = 0x20;
const CODEC_ANALOG_SIDEATT_0: u16 = 0x40;
const CODEC_ANALOG_SIDEATT_1: u16 = 0x80;

const CODEC_INPUT_0_DB: u16 = 0x17;
const CODEC_INPUT_UPDATE_BOTH: u16 = 0x40;
const CODEC_HEADPHONES_MUTE: u16 = 0x00;
const CODEC_MIC_BOOST: u16 = 0x1;
const CODEC_MIC_MUTE: u16 = 0x2;
const CODEC_ADC_MIC: u16 = 0x4;
const CODEC_ADC_LINE: u16 = 0x0;
const CODEC_OUTPUT_DAC_ENABLE: u16 = 0x10;
const CODEC_OUTPUT_MONITOR: u16 = 0x20;
const CODEC_DEEMPHASIS_NONE: u16 = 0x00;
const CODEC_DEEMPHASIS_32K: u16 = 0x01;
const CODEC_DEEMPHASIS_44K: u16 = 0x02;
const CODEC_DEEMPHASIS_48K: u16 = 0x03;
const CODEC_SOFT_MUTE: u16 = 0x01;
const CODEC_ADC_HPF: u16 = 0x00;

const CODEC_POWER_DOWN_LINE_IN: u16 = 0x01;
const CODEC_POWER_DOWN_MIC: u16 = 0x02;
const CODEC_POWER_DOWN_ADC: u16 = 0x04;
const CODEC_POWER_DOWN_DAC: u16 = 0x08;
const CODEC_POWER_DOWN_LINE_OUT: u16 = 0x10;
const CODEC_POWER_DOWN_OSCILLATOR: u16 = 0x20;
const CODEC_POWER_DOWN_CLOCK_OUTPUT: u16 = 0x40;
const CODEC_POWER_DOWN_EVERYTHING: u16 = 0x80;

const CODEC_PROTOCOL_MASK_MSB_FIRST: u16 = 0x00;
const CODEC_PROTOCOL_MASK_LSB_FIRST: u16 = 0x01;
const CODEC_PROTOCOL_MASK_PHILIPS: u16 = 0x02;
const CODEC_PROTOCOL_MASK_DSP: u16 = 0x03;

const CODEC_FORMAT_MASK_16_BIT: u16 = 0x00 << 2;
const CODEC_FORMAT_MASK_20_BIT: u16 = 0x01 << 2;
const CODEC_FORMAT_MASK_24_BIT: u16 = 0x02 << 2;
const CODEC_FORMAT_MASK_32_BIT: u16 = 0x03 << 2;

const CODEC_FORMAT_LR_SWAP: u16 = 0x20;
const CODEC_FORMAT_MASTER: u16 = 0x40;
const CODEC_FORMAT_SLAVE: u16 = 0x00;
const CODEC_FORMAT_INVERT_CLOCK: u16 = 0x80;

const CODEC_RATE_48K_48K: u16 = 0x00 << 2;
const CODEC_RATE_8K_8K: u16 = 0x03 << 2;
const CODEC_RATE_96K_96K: u16 = 0x07 << 2;
const CODEC_RATE_32K_32K: u16 = 0x06 << 2;
const CODEC_RATE_44K_44K: u16 = 0x08 << 2;

const FORMAT_MSB_FIRST_RJ: u16 = 0x00;
const FORMAT_MSB_FIRST_LJ: u16 = 0x01;
const FORMAT_I2S: u16 = 0x02;
const FORMAT_DSP: u16 = 0x03;

const WORD_LENGTH_BITS_16: u16 = 0x00 << 2;
const WORD_LENGTH_BITS_20: u16 = 0x01 << 2;
const WORD_LENGTH_BITS_24: u16 = 0x02 << 2;
const WORD_LENGTH_BITS_32: u16 = 0x03 << 2;

pub struct I2CInterface {
    i2c: I2c<I2C2>,
    address: u8,
}

impl I2CInterface {
    pub fn new(i2c: I2c<I2C2>, address: u8) -> Self {
        Self { i2c, address }
    }
    pub fn release(self) -> I2c<I2C2> {
        self.i2c
    }
    pub fn write(&mut self, address: u8, data: u16) {
        let frame: [u8; 2] = [
            (address << 1) | ((data & 0x100) >> 8) as u8,
            (data & 0xff) as u8,
        ];

        let result = self.i2c.write(W8731_ADDR, &frame).unwrap();
        delay_ms(10);
        info!("{:?}", result);

        result
    }
}

pub fn init(i2c: I2c<I2C2>) {
    let mut interface = I2CInterface::new(i2c, W8731_ADDR);

    // Reset
    interface.write(CODEC_REG_RESET, 0);

    // Set Line Inputs to 0DB
    interface.write(CODEC_REG_LEFT_LINE_IN, CODEC_INPUT_0_DB);

    interface.write(CODEC_REG_RIGHT_LINE_IN, CODEC_INPUT_0_DB);

    // Set Headphone To Mute (and disable?)
    interface.write(CODEC_REG_LEFT_HEADPHONES_OUT, CODEC_HEADPHONES_MUTE);

    interface.write(CODEC_REG_RIGHT_HEADPHONES_OUT, CODEC_HEADPHONES_MUTE);

    // Analog and Digital Routing
    interface.write(
        CODEC_REG_ANALOGUE_ROUTING,
        CODEC_MIC_MUTE | CODEC_ADC_LINE | CODEC_OUTPUT_DAC_ENABLE,
    );

    interface.write(CODEC_REG_DIGITAL_ROUTING, CODEC_DEEMPHASIS_NONE);

    // Configure power management
    let power_down_reg: u16 =
        CODEC_POWER_DOWN_MIC | CODEC_POWER_DOWN_CLOCK_OUTPUT | CODEC_POWER_DOWN_OSCILLATOR;

    interface.write(CODEC_REG_POWER_MANAGEMENT, power_down_reg);

    // Digital Format
    let format_byte: u16 = FORMAT_MSB_FIRST_LJ | WORD_LENGTH_BITS_24 | CODEC_FORMAT_SLAVE;

    interface.write(CODEC_REG_DIGITAL_FORMAT, format_byte);

    // samplerate
    // TODO: add support for other samplerates
    interface.write(CODEC_REG_SAMPLE_RATE, CODEC_RATE_48K_48K);

    interface.write(CODEC_REG_ACTIVE, 0x00);

    // Enable
    interface.write(CODEC_REG_ACTIVE, 0x01);
}
