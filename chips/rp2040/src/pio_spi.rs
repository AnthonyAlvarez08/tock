// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2024.
//
// Author: Jason Hu <jasonhu2026@u.northwestern.edu>
//         Anthony Alvarez <anthonyalvarez2026@u.northwestern.edu>

//! Programmable Input Output (PIO) hardware test file.
use crate::clocks::{self};
use crate::gpio::{RPGpio, RPGpioPin};
use crate::pio::{PIONumber, Pio, PioRxClient, PioTxClient, SMNumber, StateMachineConfiguration};
use core::cell::Cell;
use kernel::debug;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::gpio::{Configure, Output};
use kernel::hil::spi::cs::ChipSelectPolar;
use kernel::hil::spi::SpiMaster;
use kernel::hil::spi::SpiMasterClient;
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::{hil, ErrorCode};

// since auto push / pull is set to 8 for the purposes of writing in bytes rather than words
// values read in have to be bitshifted by 24
const AUTOPULL_SHIFT: usize = 24;

pub struct PioSpi<'a> {
    clocks: &'a clocks::Clocks,
    pio: &'a Pio, //TakeCell<'a, Pio>,
    side_set_pin: u32,
    out_pin: u32,
    in_pin: u32,
    sm_number: SMNumber,
    pio_number: PIONumber,
    client: OptionalCell<&'a dyn SpiMasterClient>,
    tx_buffer: MapCell<SubSliceMut<'static, u8>>,
    tx_position: Cell<usize>,
    rx_buffer: MapCell<SubSliceMut<'static, u8>>,
    rx_position: Cell<usize>,
    len: Cell<usize>,
    state: Cell<PioSpiState>,
    deferred_call: DeferredCall,
}
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum PioSpiState {
    Free = 0b00,
    Writing = 0b01,
    Reading = 0b10,
    ReadingWriting = 0b11,
}

impl<'a> PioSpi<'a> {
    pub fn new(
        pio: &'a Pio,
        clocks: &'a clocks::Clocks,
        side_set_pin: u32,
        in_pin: u32,
        out_pin: u32,
        sm_number: SMNumber,
        pio_number: PIONumber,
    ) -> Self {
        Self {
            clocks,
            pio: pio,
            side_set_pin: side_set_pin,
            in_pin: in_pin,
            out_pin: out_pin,
            sm_number: sm_number,
            pio_number: pio_number,
            client: OptionalCell::empty(),
            tx_buffer: MapCell::empty(),
            tx_position: Cell::new(0),
            rx_buffer: MapCell::empty(),
            rx_position: Cell::new(0),
            len: Cell::new(0),
            state: Cell::new(PioSpiState::Free),
            deferred_call: DeferredCall::new(),
        }
    }

    pub fn block_until_empty(&self) {
        while !self.pio.sm(self.sm_number).rx_empty() || !self.pio.sm(self.sm_number).tx_empty() {
            self.clear_fifos();
        }
    }

    pub fn rx_empty(&self) -> bool {
        let mut empty: bool = true;
        if !self.pio.sm(self.sm_number).rx_empty() {
            empty = false;
        }

        empty
    }

    pub fn read_word(&self) -> Result<u32, ErrorCode> {
        let mut data: u32 = 0;

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they write 0 out before they're reading
        if !self.pio.sm(self.sm_number).tx_full() {
            self.pio.sm(self.sm_number).push_blocking(0);
        }

        data = match self.pio.sm(self.sm_number).pull() {
            Ok(res) => res,
            Err(err) => {
                debug!("Error code : STATE MACHINE BUSY");
                return Err(err);
            }
        };

        Ok(data)
    }

    pub fn write_word(&self, val: u32) -> Result<(), ErrorCode> {
        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they reading and dumping after they're writing
        self.pio.sm(self.sm_number).push_blocking(val);

        if !self.pio.sm(self.sm_number).rx_empty() {
            let _ = self.pio.sm(self.sm_number).pull_blocking().unwrap();
        }

        Ok(())
    }

    fn read_write_buffers(&self) {
        self.tx_buffer.map(|buf| {
            let txcursor = self.tx_position.get();
            let rxcursor = self.rx_position.get();

            // let mut keepgoing = true;
            let temp = self.len.get();
            // debug!("Trying to read and write more from buffers");
            // debug!("indices in buffers {txcursor} {rxcursor} length {temp}");

            for i in 0..4 {
                // only 4 words fit in a FIFO anyway
                let mut errors = false;

                // try to write one byte
                if self.tx_position.get() < temp {
                    let res = self
                        .pio
                        .sm(self.sm_number)
                        .push(buf[self.tx_position.get()] as u32);
                    match res {
                        Err(error) => errors = true,
                        _ => {
                            self.tx_position.set(self.tx_position.get() + 1);
                        }
                    }
                }

                // try to read one byte
                if self.rx_position.get() < temp {
                    let data = self.pio.sm(self.sm_number).pull();
                    match data {
                        Ok(val) => {
                            // this branch basically never gets called because there is likely nothing new in the buffer
                            // leave it here just in case

                            self.rx_buffer.map(|readbuf| {
                                // debug!("Read value {val}");
                                readbuf[self.rx_position.get()] = (val >> AUTOPULL_SHIFT) as u8;
                                self.rx_position.set(self.rx_position.get() + 1);
                            });
                        }
                        _ => errors = true,
                    }
                }

                // if done, then reset and call client
                if self.tx_position.get() >= self.len.get()
                    && self.rx_position.get() >= self.len.get()
                {
                    // printing tx contents
                    // let mut idx = 0;
                    // debug!("Printing TX Buffer");
                    // while idx < buf.len() {
                    //     let temp = buf[idx];
                    // debug!("[TX buf] {temp}");
                    //     idx += 1;
                    // }

                    // printing rx contents
                    debug!("Printing RX Buffer");
                    self.rx_buffer.map(|buf| {
                        let mut idx = 0;
                        while idx < buf.len() {
                            let temp = buf[idx];
                            debug!("[RX buf] {temp}");
                            idx += 1;
                        }
                    });

                    // should maybe not set it to free if it is writing
                    self.state.set(PioSpiState::Free);
                    self.len.set(0);
                    self.tx_position.set(0);
                    self.rx_position.set(0);

                    self.deferred_call.set();

                    // the following is now in the deferred call
                    // debug!("Transferring ownership");
                    // // transfer ownership to client
                    // if let Some(tx_buffer) = self.tx_buffer.take() {
                    //     self.client.map(|client| {
                    //         client.read_write_done(
                    //             tx_buffer,
                    //             self.rx_buffer.take(),
                    //             Ok(0 as usize),
                    //         );
                    //     });
                    // }
                    break;
                }

                if errors {
                    break;
                }
            }
        });
    }

    pub fn restart_clock(&self) {
        self.pio.sm(self.sm_number).clkdiv_restart();
    }

    pub fn clear_fifos(&self) {
        self.pio.sm(self.sm_number).clear_fifos();
    }
}

impl<'a> hil::spi::SpiMaster<'a> for PioSpi<'a> {
    type ChipSelect = ChipSelectPolar<'a, crate::gpio::RPGpioPin<'a>>;

    fn init(&self) -> Result<(), ErrorCode> {
        /*

                implements this program for now
                https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/spi.pio

                assembly found here I think on line 63


                .pio_version 0 // only requires PIO version 0
                https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/spi/spi_rpi_pico_pio.c

                #define SPI_MODE_0_0_WRAP_TARGET 0
        #define SPI_MODE_0_0_WRAP        1
        #define SPI_MODE_0_0_CYCLES      4



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

        let asm: [u16; 2] = [
            0x6101, /*  0: out    pins, 1         side 0 [1] */
            0x5101, /*  1: in     pins, 1         side 1 [1] */
        ];

        self.pio.init();
        self.pio.add_program16(None::<usize>, &asm);
        // TODO: add custom configurations if necessary
        let mut custom_config = StateMachineConfiguration::default();
        // the program requires auto push and pull to be on
        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/spi_loopback.c
        //
        //  float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
        // custom_config.div_int = 31;
        // custom_config.div_frac = 64; // is 64/256 = 1/4

        // should be able to work on bytes according to the PIO manual
        custom_config.in_push_threshold = 8;
        custom_config.out_pull_threshold = 8;

        custom_config.side_set_base = self.side_set_pin;
        custom_config.in_pins_base = self.in_pin;
        custom_config.out_pins_base = self.out_pin;
        custom_config.side_set_bit_count = 1;
        custom_config.wrap = 1;
        custom_config.in_autopush = true;
        custom_config.out_autopull = true;
        self.pio.spi_program_init(
            self.sm_number,
            self.side_set_pin,
            self.in_pin,
            self.out_pin,
            &custom_config,
        );

        Ok(())
    }

    fn set_client(&self, client: &'a dyn SpiMasterClient) {
        self.client.set(client);
    }

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
        // buffer should be at least one thing big
        if write_buffer.len() < 1 {
            return Err((ErrorCode::INVAL, write_buffer, read_buffer));
        }

        // TODO
        // keep track of the new buffers
        self.len.replace(write_buffer.len());
        self.tx_buffer.replace(write_buffer);
        self.tx_position.set(0);

        self.state.replace(PioSpiState::Writing);

        if let Some(readbuf) = read_buffer {
            self.rx_buffer.replace(readbuf);
            self.state.replace(PioSpiState::ReadingWriting);
            self.rx_position.set(0);
            debug!("will read");
        }

        self.read_write_buffers();

        Ok(())
    }

    fn write_byte(&self, val: u8) -> Result<(), ErrorCode> {
        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they reading and dumping after they're writing
        match self.pio.sm(self.sm_number).push(val as u32) {
            Err(error) => return Err(error),
            _ => {}
        }

        if !self.pio.sm(self.sm_number).rx_empty() {
            match self.pio.sm(self.sm_number).pull() {
                Ok(val) => {}
                Err(err) => {}
            }
        }

        Ok(())
    }

    fn read_byte(&self) -> Result<u8, ErrorCode> {
        let mut data: u32 = 0;

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they write 0 out before they're reading
        if !self.pio.sm(self.sm_number).tx_full() {
            self.pio.sm(self.sm_number).push(0);
        }

        data = match self.pio.sm(self.sm_number).pull() {
            Ok(val) => val,
            Err(error) => {
                return Err(error);
            }
        };

        data = data >> AUTOPULL_SHIFT;

        Ok(data as u8)
    }

    fn read_write_byte(&self, val: u8) -> Result<u8, ErrorCode> {
        let mut data: u32 = 0;
        // Read data from the RX FIFO

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they write 0 out before they're reading
        if !self.pio.sm(self.sm_number).tx_full() {
            match self.pio.sm(self.sm_number).push(val as u32) {
                Err(err) => {
                    return Err(err);
                }
                _ => {}
            }
        }

        data = match self.pio.sm(self.sm_number).pull() {
            Ok(val) => val,
            Err(error) => {
                return Err(error);
            }
        };

        data = data >> AUTOPULL_SHIFT;

        Ok(data as u8)
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

impl<'a> PioTxClient for PioSpi<'a> {
    fn on_buffer_space_available(&self) {
        // TODO: make this keep writing data
        // debug!("INSIDE INTERRUPT HANDLER buffer space available\n");

        // increment the tx index
        self.tx_position.set(self.tx_position.get() + 1);
        match self.state.get() {
            // if currently writing try to write the next byte
            PioSpiState::Writing | PioSpiState::ReadingWriting => {
                // debug!("trying to write more");
                self.read_write_buffers();
            }
            _ => {}
        }

        let pin = RPGpioPin::new(RPGpio::GPIO9);
        pin.make_output();
        for i in 0..16 {
            pin.toggle();
        }
    }
}

impl<'a> PioRxClient for PioSpi<'a> {
    fn on_data_received(&self, data: u32) {
        // TODO: make this actually record the data
        let pin = RPGpioPin::new(RPGpio::GPIO9);
        pin.make_output();
        for i in 0..16 {
            pin.toggle();
        }
        let data = data >> AUTOPULL_SHIFT;
        debug!("INSIDE INTERRUPT HANDLER Received data {data}");

        // add to rx buffer and update the rx index
        if self.len.get() > self.rx_position.get() {
            self.rx_buffer.map(|buf| {
                buf[self.rx_position.get()] = data as u8;
                self.rx_position.set(self.rx_position.get() + 1);
            });
        }

        match self.state.get() {
            // if currently writing try to write the next byte
            PioSpiState::Reading | PioSpiState::ReadingWriting => {
                // debug!("trying to read more more");
                self.read_write_buffers();
            }
            _ => {}
        }
    }
}

impl<'a> DeferredCallClient for PioSpi<'a> {
    fn handle_deferred_call(&self) {
        // Your action here

        debug!("Transferring ownership");
        debug!("Calling client in deferred call");
        // transfer ownership to client
        if let Some(tx_buffer) = self.tx_buffer.take() {
            self.client.map(|client| {
                client.read_write_done(tx_buffer, self.rx_buffer.take(), Ok(0 as usize));
            });
        }
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
