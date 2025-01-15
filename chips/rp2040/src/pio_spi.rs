// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2024.
//
// Author: Jason Hu <jasonhu2026@u.northwestern.edu>
//         Anthony Alvarez <anthonyalvarez2026@u.northwestern.edu>

//! Programmable Input Output (PIO) hardware test file.
use crate::clocks::{self};
use crate::gpio::RPGpio;
use crate::pio::{PIONumber, Pio, SMNumber, StateMachineConfiguration};

use kernel::utilities::cells::TakeCell;
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


impl hil::spi::SpiMaster<'a> for PioSpi<'a> {



    pub fn init(&self) -> Result<(), ErrorCode> {

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

        let asm: [u8, 2] = [
            0x6101, /*  0: out    pins, 1         side 0 [1] */
			0x5101 /*  1: in     pins, 1         side 1 [1] */
        ];

        self.pio.map(|pio |{
            pio.init();
            pio.add_program(&asm);

            // TODO: add configs and stuff
        });

        Ok(())
    }


    pub fn set_client(&self, client: &'a dyn SpiMasterClient) {

    }


    pub fn is_busy(&self) -> bool {
        true
    }


    pub fn read_write_bytes(
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
        Ok(())
    }

    
    fn read_byte(&self) -> Result<u8, ErrorCode> {
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

    
    fn hold_low(&self) {

    }

    
    fn release_low(&self) {

    }

}
