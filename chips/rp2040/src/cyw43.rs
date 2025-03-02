// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2024.
//
// Author: Jason Hu <jasonhu2026@u.northwestern.edu>
//         Anthony Alvarez <anthonyalvarez2026@u.northwestern.edu>
use crate::clocks::{self};
use crate::gpio::{RPGpio, RPGpioPin};
use crate::pio::{PIONumber, Pio, PioRxClient, PioTxClient, SMNumber, StateMachineConfiguration};
use core::cell::Cell;
use kernel::debug;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil;
use kernel::hil::gpio::{Configure, Output};
use kernel::hil::spi::cs::{ChipSelectPolar, Polarity};
use kernel::hil::spi::SpiMasterClient;
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;
/*
* following the zephyr one loosely
* https://github.com/beechwoods-software/zephyr-cyw43-driver/tree/main/drivers/wifi/zephyr_cyw43/src
* think this will have to be its own mini SPI driver
* if need to write custom instructions
https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pio/include/hardware/pio_instructions.h


 * Notes
 * according to page 53 of the cyw43 chip, we need to set pin L7 to low to enable SPI mode
*/

/*
; PicoWi PIO program for half-duplex SPI transfers
; See http://iosoft.blog/picowi for details

.program picowi_pio
.side_set 1
.wrap_target
.origin 0
public stall:                   ; Stall here when transfer complete

; Write data to SPI (42 MHz SPI clock, if divisor is set to 1)
public writer:
    pull                side 0  ; Get byte to transmit from FIFO
  wrloop:
    nop                 side 0  ; Delay (if deleted, SPI clock is 63 MHz)
    out pins, 1         side 0  ; Set next Tx bit
    jmp !osre wrloop    side 1  ; Loop if data in shift reg
.wrap

; Read data from SPI (25 MHz SPI clock, if divisor is set to 1)
public reader:
    pull                side 0  ; Get byte count from host FIFO
    out x, 32           side 0  ; Copy into x register
  byteloop:
    set y, 7            side 0  ; For each bit in byte..
  bitloop:
    nop                 side 1  ; Delay
    nop                 side 1
    nop                 side 1
    in pins, 1          side 0  ; Input SPI data bit
    jmp y--, bitloop    side 0  ; Loop until byte received
    push                side 0  ; Put byte in host FIFO
    jmp x--, byteloop   side 0  ; Loop until all bytes received
    jmp reader          side 0  ; Loop to start next transfer

; Read data from SPI, if clock is set slower (e.g. 10 MHz for write)
public slow_reader:
    pull                side 0
    out x, 32           side 0
  byteloop2:
    set y, 7            side 0
  bitloop2:
    nop                 side 0
    in pins, 1          side 0
    nop                 side 1
    jmp y--, bitloop2   side 1
    push                side 0
    jmp x--, byteloop2  side 0
    jmp slow_reader     side 0

; EOF

*/

const HALF_DUPLEX_SPI: [u16; 25] = [
    //     .wrap_target
    0x80a0, //  0: pull   block           side 0
    0xa042, //  1: nop                    side 0
    0x6001, //  2: out    pins, 1         side 0
    0x10e1, //  3: jmp    !osre, 1        side 1
    //     .wrap
    0x80a0, //  4: pull   block           side 0
    0x6020, //  5: out    x, 32           side 0
    0xe047, //  6: set    y, 7            side 0
    0xb042, //  7: nop                    side 1
    0xb042, //  8: nop                    side 1
    0xb042, //  9: nop                    side 1
    0x4001, // 10: in     pins, 1         side 0
    0x0087, // 11: jmp    y--, 7          side 0
    0x8020, // 12: push   block           side 0
    0x0046, // 13: jmp    x--, 6          side 0
    0x0004, // 14: jmp    4               side 0
    0x80a0, // 15: pull   block           side 0
    0x6020, // 16: out    x, 32           side 0
    0xe047, // 17: set    y, 7            side 0
    0xa042, // 18: nop                    side 0
    0x4001, // 19: in     pins, 1         side 0
    0xb042, // 20: nop                    side 1
    0x1092, // 21: jmp    y--, 18         side 1
    0x8020, // 22: push   block           side 0
    0x0051, // 23: jmp    x--, 17         side 0
    0x000f, // 24: jmp    15              side 0
];

const SD_ON_PIN: u32 = 23;
const SD_CMD_PIN: u32 = 24;
const SD_DIN_PIN: u32 = 24;
const SD_D0_PIN: u32 = 24;
const SD_CS_PIN: u32 = 25;
const SD_CLK_PIN: u32 = 29;
const SD_IRQ_PIN: u32 = 24;
const SD_LED_GPIO: u32 = 0;

const pio_wrap_target: u32 = 0;
const pio_wrap: u32 = 3;
const pio_offset_stall: u32 = 0;
const pio_offset_writer: u32 = 0;
const pio_offset_reader: u32 = 4;
const pio_offset_slow_reader: u32 = 15;

#[repr(u8)]
#[derive(PartialEq, Clone, Copy)]
pub enum RegisterType {
    Readable,
    Writeable,
    ReadWriteable,
}
pub struct CYW43Register {
    address: u32,
    register_type: RegisterType,
}

impl CYW43Register {
    pub fn new(address: u32, register_type: RegisterType) -> Self {
        Self {
            address,
            register_type,
        }
    }
}

const SETTINGS_REGISTER: CYW43Register = CYW43Register {
    address: 0x000,
    register_type: RegisterType::ReadWriteable,
};

const STATUS_SETTINGS_REGISTER: CYW43Register = CYW43Register {
    address: 0x002,
    register_type: RegisterType::ReadWriteable,
};

const INTERRUPT_REGISTER0: CYW43Register = CYW43Register {
    address: 0x004,
    register_type: RegisterType::ReadWriteable,
};

const INTERRUPT_REGISTER1: CYW43Register = CYW43Register {
    address: 0x005,
    register_type: RegisterType::Readable,
};

const INTERRUPT_ENABLE: CYW43Register = CYW43Register {
    address: 0x006,
    register_type: RegisterType::ReadWriteable,
};

const STATUS_REGISTER: CYW43Register = CYW43Register {
    address: 0x008,
    register_type: RegisterType::Readable,
};

const F1_INFO: CYW43Register = CYW43Register {
    address: 0x00C,
    register_type: RegisterType::ReadWriteable,
};

const F2_INFO: CYW43Register = CYW43Register {
    address: 0x00E,
    register_type: RegisterType::ReadWriteable,
};

const TEST_READ_ONLY: CYW43Register = CYW43Register {
    address: 0x014,
    register_type: RegisterType::Readable,
};

const TEST_READ_WRITE: CYW43Register = CYW43Register {
    address: 0x018,
    register_type: RegisterType::ReadWriteable,
};

const RESPONSE_DELAY_REG_F0: CYW43Register = CYW43Register {
    address: 0x01C,
    register_type: RegisterType::ReadWriteable,
};

const RESPONSE_DELAY_REG_F1: CYW43Register = CYW43Register {
    address: 0x01D,
    register_type: RegisterType::ReadWriteable,
};

const RESPONSE_DELAY_REG_F2: CYW43Register = CYW43Register {
    address: 0x01E,
    register_type: RegisterType::ReadWriteable,
};

const RESPONSE_DELAY_REG_F3: CYW43Register = CYW43Register {
    address: 0x01F,
    register_type: RegisterType::ReadWriteable,
};

register_structs! {
    MMIO_CYW43Registers {
        (0x0000 => settings: ReadWrite<u16, SETTINGS::Register>),
        (0x0002 => status_settings: ReadWrite<u16, STATUS_SETTINGS::Register>),
        (0x0004 => interrupt_reg0: ReadOnly<u8, INTERRUPT0::Register>),
        (0x0005 => interrupt_reg1: ReadOnly<u8, INTERRUPT1::Register>),
        (0x0006 => interrupt_enable: ReadWrite<u16, INTERRUPT_ENABLE::Register>),
        (0x0008 => status_reg: ReadOnly<u32, STATUS_REGISTER::Register>),
        (0x000C => f1_info: ReadWrite<u16, F1_INFO::Register>),
        (0x000E => f2_info: ReadWrite<u16, F2_INFO::Register>),
        (0x0010 => unused: ReadOnly<u32, UNUSED::Register>),
        (0x0014 => test_read_only: ReadOnly<u32, TEST_READ_ONLY::Register>),
        (0x0018 => test_read_write: ReadWrite<u32, TEST_READ_WRITE::Register>),
        (0x001C => response_delay_reg: [ReadWrite<u8, RESPONSE_DELAY::Register>; 4] ),
        (0x0020 => @END),

    }
}

// * this can still be used for bitmaks
register_bitfields![u32,
SETTINGS [
    WORD_LENGTH OFFSET(0) NUMBITS(1) [],
    ENDIANNESS OFFSET(1) NUMBITS(1) [],
    HIGH_SPEED_MODE OFFSET(4) NUMBITS(1) [],
    INTERRUPT_POLARITY OFFSET(5) NUMBITS(1) [],
    WAKE_UP OFFSET(7) NUMBITS(1) []

],
STATUS_SETTINGS [
    STATUS_ENABLE OFFSET(0) NUMBITS(1) [],
    INTERRUPT_WITH_STATUS OFFSET(1) NUMBITS(1) [],

],
INTERRUPT0 [
    INTERRUPT0_NOT_AVAILABLE OFFSET(0) NUMBITS(1) [],
    INTERRUPT0_F2_F3_UNDERFLOW OFFSET(1) NUMBITS(1) [],
    INTERRUPT0_F2_F3_OVERFLOW OFFSET(2) NUMBITS(1) [],
    INTERRUPT0_F2_PACKET_AVAILABLE OFFSET(5) NUMBITS(1) [],
    INTERRUPT0_F3_PACKET_AVAILABLE OFFSET(6) NUMBITS(1) [],
    INTERRUPT0_F1_OVERFLOW OFFSET(7) NUMBITS(1) [],
],
INTERRUPT1 [
    F1_INTERRUPT OFFSET(5) NUMBITS(1) [],
    F2_INTERRUPT OFFSET(6) NUMBITS(1) [],
    F3_INTERRUPT OFFSET(7) NUMBITS(1) [],
],
INTERRUPT_ENABLE [
    INTERRUPT_ENABLE OFFSET(0) NUMBITS(16) [],
],
F1_INFO [
    F1_ENABLED OFFSET(0) NUMBITS(1) [],
    F1_READY_FOR_TX OFFSET(1) NUMBITS(1) [],
    F1_MAX_PACKET_SIZE OFFSET(2) NUMBITS(12) [],
],
F2_INFO [
    F2_ENABLED OFFSET(0) NUMBITS(1) [],
    F2_READY_FOR_TX OFFSET(1) NUMBITS(1) [],
    F2_MAX_PACKET_SIZE OFFSET(2) NUMBITS(14) [],
],
TEST_READ_ONLY [
    TEST_READ_ONLY OFFSET(0) NUMBITS(32) [],
],
TEST_READ_WRITE [
    TEST_READ_WRITE OFFSET(0) NUMBITS(32) [],
],
RESPONSE_DELAY [
    RESPONSE_DELAY OFFSET(0) NUMBITS(8) [],
],
STATUS_REGISTER [
    STAT_REG OFFSET(0) NUMBITS(32) [],
],

UNUSED [
    UNUSED OFFSET(0) NUMBITS(32) [],
]

];

const TEST_READ_VALUE: u32 = 0xFEEDBEAD;

pub struct CYW43_DRIVER<'a> {
    pio: &'a Pio,
    sm_number: SMNumber,
}

impl<'a> CYW43_DRIVER<'a> {
    pub fn new(pio: &'a Pio, sm_number: SMNumber) -> Self {
        Self { pio, sm_number }
    }

    pub fn init(&self) -> Result<(), ErrorCode> {
        self.pio.init();

        match self.pio.add_program16(None, &HALF_DUPLEX_SPI) {
            Err(_error) => return Err(ErrorCode::FAIL),
            _ => {}
        };

        let mut custom_config = StateMachineConfiguration::default();

        custom_config.side_set_base = SD_CLK_PIN;
        custom_config.out_pins_base = SD_D0_PIN;
        custom_config.in_pins_base = SD_DIN_PIN;

        // TODO: maybe it wont work with the normal spi program init
        self.pio.spi_program_init(
            self.sm_number,
            SD_CLK_PIN,
            SD_DIN_PIN,
            SD_D0_PIN,
            &custom_config,
        );

        Ok(())
    }

    pub fn try_read(&self) -> Result<u32, ErrorCode> {
        let cmd_packet;
        cmd_packet = match make_wifi_cmd_packet(true, false, TEST_READ_ONLY.address, 4) {
            Ok(res) => res,
            Err(err) => {
                return Err(err);
            }
        };

        match self.pio.sm(self.sm_number).push_blocking(cmd_packet) {
            Err(err) => {
                return Err(err);
            }
            _ => {}
        }

        let data;
        data = match self.pio.sm(self.sm_number).pull_blocking() {
            Ok(res) => res,
            Err(err) => {
                return Err(err);
            }
        };

        Ok(data)
    }
}

fn make_wifi_cmd_packet(
    writing: bool,
    incremental: bool,
    address: u32,
    data_length: u16,
) -> Result<u32, ErrorCode> {
    // see page 20 of the data sheet
    // bits [0, 10] is length of the packet
    // bits [11, 27] is register address, is 17 bits
    // bits [28, 29] is function type, for now will leave at 00 because that is for SPI, and the others are for DMA and other things we're not using
    // bit 30 is incremental (1) vs fixed (0) address
    // bit 31 is read (0) or write (1)

    /*
        static inline uint32_t make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz) {
        return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
    }
        */

    const NUM_ADDRESS_BITS: u32 = 17;
    const NUM_DATA_LENGTH_BITS: u32 = 11;

    // do not let the address go out of bounds
    if address > (1 << NUM_ADDRESS_BITS) {
        return Err(ErrorCode::INVAL);
    }

    let mut packet: u32 = 0;

    if writing {
        packet = packet | 0x8000_0000u32;
    }

    if incremental {
        packet = packet | 0x4000_0000u32;
    }

    packet = packet | (address << NUM_ADDRESS_BITS);

    if data_length > (1 << NUM_DATA_LENGTH_BITS) {
        return Err(ErrorCode::INVAL);
    }

    packet = packet | data_length as u32;

    Ok(packet)
}

fn make_wifi_cmd_packet_bytes(
    writing: bool,
    incremental: bool,
    address: u32,
    data_length: u16,
) -> Result<[u8; 4], ErrorCode> {
    let temp = make_wifi_cmd_packet(writing, incremental, address, data_length);

    let packet = match temp {
        Ok(res) => res,
        Err(error) => return Err(error),
    };

    Ok([
        (packet & 0xFF000000 >> 24) as u8,
        (packet & 0xFF0000 >> 16) as u8,
        (packet & 0xFF00 >> 8) as u8,
        (packet & 0xFF) as u8,
    ])
}
