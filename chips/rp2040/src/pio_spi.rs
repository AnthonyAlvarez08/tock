// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2024.
//
// Author: Jason Hu <jasonhu2026@u.northwestern.edu>
//         Anthony Alvarez <anthonyalvarez2026@u.northwestern.edu>

//! Programmable Input Output (PIO) hardware test file.
use crate::clocks::{self};
use crate::gpio::{GpioFunction, RPGpio, RPGpioPin};
use crate::pio::{PIONumber, Pio, SMNumber, StateMachineConfiguration};
use kernel::hil::spi::cs::ChipSelectPolar;
use kernel::hil::spi::SpiMaster;
use kernel::hil::spi::SpiMasterClient;
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::cells::TakeCell;
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::{hil, ErrorCode};

pub struct PioSpi<'a> {
    clocks: &'a clocks::Clocks,
    pio: TakeCell<'a, Pio>,
}

impl<'a> PioSpi<'a> {
    pub fn new(pio: &'a mut Pio, clocks: &'a clocks::Clocks) -> Self {
        Self {
            clocks,
            pio: TakeCell::new(pio),
        }
    }
}

impl<'a> hil::spi::SpiMaster<'a> for PioSpi<'a> {
    // I just copied this from spi.rs in the same folder
    type ChipSelect = ChipSelectPolar<'a, crate::gpio::RPGpioPin<'a>>;

    fn init(&self) -> Result<(), ErrorCode> {
        /*

        implements this program for now
        https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/spi.pio

        assembly found here I think on line 63


        .pio_version 0 // only requires PIO version 0
        https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/spi/spi_rpi_pico_pio.c


        .program spi_cpha0
        .side_set 1

        ; Pin assignments:
        ; - SCK is side-set pin 0
        ; - MOSI is OUT pin 0
        ; - MISO is IN pin 0
        ;
        ; Autopush and autopull must be enabled, and the serial frame size is set by
        ; configuring the push/pull threshold. Shift left/right is fine, but you must
        ; justify the data yourself. This is done most conveniently for frame sizes of
        ; 8 or 16 bits by using the narrow store replication and narrow load byte
        ; picking behaviour of RP2040's IO fabric.

        ; Clock phase = 0: data is captured on the leading edge of each SCK pulse, and
        ; transitions on the trailing edge, or some time before the first leading edge.

            out pins, 1 side 0 [1] ; Stall here on empty (sideset proceeds even if
            in pins, 1  side 1 [1] ; instruction stalls, so we stall with SCK low)

        */

        let asm: [u8; 4] = [
            // these were 4 digit hex numbers in the zephyr one
            // maybe it will work fine if I split them?
            0x61, 0x01, /*  0: out    pins, 1         side 0 [1] */
            0x51, 0x01, /*  1: in     pins, 1         side 1 [1] */
        ];

        self.pio.map(|pio| {
            pio.init();
            pio.add_program(&asm);
            
            // TODO: add custom configurations if necessary
            let mut custom_config = StateMachineConfiguration::default();
            let sm_number = SMNumber::SM0;
            let pin = 1;

            pio.spi_program_init(PIONumber::PIO0, sm_number, pin, &custom_config);

            // use example blink program that is in 
            // chips/rp2040/src/pio.rs
            // put it in the other PIO and other State machine tho
            pio.blink_program_init(PIONumber::PIO1, SMNumber::SM1, 1, &custom_config);
        });

        Ok(())
    }

    fn set_client(&self, client: &'a dyn SpiMasterClient) {}

    fn is_busy(&self) -> bool {
        true
    }

    fn read_write_bytes(
        &self,
        write_buffer: SubSliceMut<'static, u8>,
        read_buffer: Option<SubSliceMut<'static, u8>>,
    ) -> Result<
        (),
        (
            ErrorCode,
            SubSliceMut<'static, u8>,
            Option<SubSliceMut<'static, u8>>,
        ),
    > {
        Ok(())
    }

    fn write_byte(&self, val: u8) -> Result<(), ErrorCode> {
        self.pio.map(|pio| {
            // Waits until the state machine TX FIFO is empty, then write the byte of data
            pio.sm_put_blocking(SMNumber::SM0, val as u32);
        });

        Ok(())
    }

    fn read_byte(&self) -> Result<u8, ErrorCode> {
        self.pio.map(|pio| {
            // Read data from the RX FIFO
            let data = pio.sm_get(SMNumber::SM0);
            return data;
        });

        Ok(1 as u8)
    }

    fn read_write_byte(&self, val: u8) -> Result<u8, ErrorCode> {
        Ok(1 as u8)
    }

    fn specify_chip_select(&self, cs: Self::ChipSelect) -> Result<(), ErrorCode> {
        Ok(())
    }

    fn set_rate(&self, rate: u32) -> Result<u32, ErrorCode> {
        Ok(rate)
    }

    fn get_rate(&self) -> u32 {
        4 as u32
    }

    fn set_polarity(&self, polarity: ClockPolarity) -> Result<(), ErrorCode> {
        Ok(())
    }

    fn get_polarity(&self) -> ClockPolarity {
        ClockPolarity::IdleHigh
    }

    fn set_phase(&self, phase: ClockPhase) -> Result<(), ErrorCode> {
        Ok(())
    }

    fn get_phase(&self) -> ClockPhase {
        ClockPhase::SampleLeading
    }

    fn hold_low(&self) {}

    fn release_low(&self) {}
}
