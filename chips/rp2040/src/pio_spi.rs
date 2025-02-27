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
use kernel::hil::spi::SpiMasterClient;
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::{hil, ErrorCode};

// Since auto push / pull is set to 8 for the purposes of writing in bytes
// rather than words, values read in have to be bitshifted by 24
const AUTOPULL_SHIFT: usize = 24;

// Frequency of system clock, for rate changess
const SYSCLOCK_FREQ: u32 = 125_000_000;

// // Leading edge clock phase
// const SPI_CPHA0: [u16; 2] = [
//     0x6101, /*  0: out    pins, 1         side 0 [1] */
//     0x5101, /*  1: in     pins, 1         side 1 [1] */
// ];

// // Trailing edge clock phase
// const SPI_CPHA1: [u16; 3] = [
//     0x6021, /* 0: out    x, 1            side 0 */
//     0xb101, /* 1: mov    pins, x         side 1 [1] */
//     0x4001, /* 2: in     pins, 1         side 0 */
// ];

pub struct PioSpi<'a> {
    clocks: OptionalCell<&'a clocks::Clocks>,
    pio: &'a Pio,
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
    clock_div_int: Cell<u32>,
    clock_div_frac: Cell<u32>,
    clock_phase: Cell<ClockPhase>,
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
        clock_phase: ClockPhase,
    ) -> Self {
        Self {
            clocks: OptionalCell::new(clocks),
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
            clock_div_int: Cell::new(125u32), // defaults to 1 MHz
            clock_div_frac: Cell::new(0u32),
            clock_phase: Cell::new(clock_phase),
        }
    }

    // Block until both the RX and TX FIFO's are empty
    pub fn block_until_empty(&self) {
        while !self.pio.sm(self.sm_number).rx_empty() || !self.pio.sm(self.sm_number).tx_empty() {
            self.clear_fifos();
        }
    }

    // Returns whether or not the RX FIFO is empty
    pub fn rx_empty(&self) -> bool {
        let mut empty: bool = true;
        if !self.pio.sm(self.sm_number).rx_empty() {
            empty = false;
        }

        empty
    }

    // Read a word from the RX FIFO
    pub fn read_word(&self) -> Result<u32, ErrorCode> {
        let data: u32;

        if !self.pio.sm(self.sm_number).tx_full() {
            let _ = self.pio.sm(self.sm_number).push(0);
        }

        data = match self.pio.sm(self.sm_number).pull() {
            Ok(res) => res,
            Err(err) => {
                return Err(err);
            }
        };

        Ok(data)
    }

    // Write a word to the TX FIFO
    pub fn write_word(&self, val: u32) -> Result<(), ErrorCode> {
        let res = self.pio.sm(self.sm_number).push(val);

        match res {
            Err(err) => {
                return Err(err);
            }
            _ => {}
        }

        if !self.pio.sm(self.sm_number).rx_empty() {
            let _ = self.pio.sm(self.sm_number).pull();
        }

        Ok(())
    }

    // Reads and writes to and from the buffers
    fn read_write_buffers(&self) {
        self.tx_buffer.map(|buf| {
            let temp = self.len.get();

            for _i in 0..4 {
                let mut errors = false;

                // Try to write one byte
                if self.tx_position.get() < temp {
                    let res = self
                        .pio
                        .sm(self.sm_number)
                        .push(buf[self.tx_position.get()] as u32);
                    match res {
                        Err(_error) => errors = true,
                        _ => {
                            self.tx_position.set(self.tx_position.get() + 1);
                        }
                    }
                }

                // Try to read one byte
                if self.rx_position.get() < temp {
                    let data = self.pio.sm(self.sm_number).pull();
                    match data {
                        Ok(val) => {
                            self.rx_buffer.map(|readbuf| {
                                readbuf[self.rx_position.get()] = (val >> AUTOPULL_SHIFT) as u8;
                                self.rx_position.set(self.rx_position.get() + 1);
                            });
                        }
                        _ => errors = true,
                    }
                }

                // If we are done reading and writing, then reset and call client
                if self.tx_position.get() >= self.len.get()
                    && self.rx_position.get() >= self.len.get()
                {
                    // printing tx contents
                    debug!("Printing TX Buffer");
                    let mut idx = 0;
                    while idx < buf.len() {
                        let temp = buf[idx];
                        debug!("[TX buf] {temp}");
                        idx += 1;
                    }

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

                    self.state.set(PioSpiState::Free);
                    self.len.set(0);
                    self.tx_position.set(0);
                    self.rx_position.set(0);
                    self.deferred_call.set();

                    break;
                }

                if errors {
                    break;
                }
            }
        });
    }

    // Restart the clock
    pub fn restart_clock(&self) {
        self.pio.sm(self.sm_number).clkdiv_restart();
    }

    // Clear both the TX and RX FIFO's
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


        ; Clock phase = 0: data is captured on the leading edge of each SCK pulse, and
        ; transitions on the trailing edge, or some time before the first leading edge.

        ; Clock phase = 1: data transitions on the leading edge of each SCK pulse, and
        ; is captured on the trailing edge.



        */

        // let asm: [u16; 2] = [
        //     0x6101, /*  0: out    pins, 1         side 0 [1] */
        //     0x5101, /*  1: in     pins, 1         side 1 [1] */
        // ];

        // Leading edge clock phase
        let SPI_CPHA0: [u16; 2] = [
            0x6101, /*  0: out    pins, 1         side 0 [1] */
            0x5101, /*  1: in     pins, 1         side 1 [1] */
        ];

        // Trailing edge clock phase
        let SPI_CPHA1: [u16; 3] = [
            0x6021, /* 0: out    x, 1            side 0 */
            0xb101, /* 1: mov    pins, x         side 1 [1] */
            0x4001, /* 2: in     pins, 1         side 0 */
        ];

        self.pio.init();
        // let _ = self.pio.add_program16(None::<usize>, &SPI_CPHA0);

        let mut wrap = 1;

        let mut program_load_res;

        match self.clock_phase.get() {
            ClockPhase::SampleLeading => {
                program_load_res = self.pio.add_program16(None::<usize>, &SPI_CPHA0);
            }
            ClockPhase::SampleTrailing => {
                program_load_res = self.pio.add_program16(None::<usize>, &SPI_CPHA1);
                wrap = 2;
                debug!("trailing");
            }
        }

        match program_load_res {
            Err(error) => return Err(ErrorCode::FAIL),
            _ => {}
        }

        let mut custom_config = StateMachineConfiguration::default();

        custom_config.div_int = self.clock_div_int.get();
        custom_config.div_frac = self.clock_div_frac.get();

        // should be able to work on bytes according to the PIO manual
        custom_config.in_push_threshold = 8;
        custom_config.out_pull_threshold = 8;

        custom_config.side_set_base = self.side_set_pin;
        custom_config.in_pins_base = self.in_pin;
        custom_config.out_pins_base = self.out_pin;
        custom_config.side_set_bit_count = 1;
        custom_config.wrap = wrap;
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
        // match self.state.get() {
        //     PioSpiState::Free => true,
        //     _ => false,
        // }
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
        if write_buffer.len() < 1 {
            return Err((ErrorCode::INVAL, write_buffer, read_buffer));
        }

        // Keep track of the new buffers
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
        match self.pio.sm(self.sm_number).push(val as u32) {
            Err(error) => return Err(error),
            _ => {}
        }

        if !self.pio.sm(self.sm_number).rx_empty() {
            let _ = self.pio.sm(self.sm_number).pull();
        }

        Ok(())
    }

    fn read_byte(&self) -> Result<u8, ErrorCode> {
        let mut data: u32;

        if !self.pio.sm(self.sm_number).tx_full() {
            let _ = self.pio.sm(self.sm_number).push(0);
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
        let mut data: u32;

        match self.pio.sm(self.sm_number).push(val as u32) {
            Err(err) => {
                return Err(err);
            }
            _ => {}
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
        if rate == 0 {
            return Err(ErrorCode::FAIL);
        }

        let sysclock_freq = self.clocks.map_or(SYSCLOCK_FREQ, |clocks| {
            clocks.get_frequency(clocks::Clock::System)
        });

        debug!("Demanded frequency is {rate}");
        debug!("System frequency {sysclock_freq}");

        // rate = SYSCLOCK_FREQ / div
        // div = SYCLCcOK / rate
        let divint = sysclock_freq / rate;

        /*
        fractional part of sysclock/rate is
        (sysclock % rate) / rate
        but want it to be a fraction of 256
        divfrac = (sysclock % rate) * 256 * rate
        */
        let divfrac = (sysclock_freq % rate) * 256u32 / rate;

        self.clock_div_int.replace(divint);
        self.clock_div_frac.replace(divfrac);

        // reinit the PIO so it updates the times
        self.pio.sm(self.sm_number).set_enabled(false);
        self.pio
            .sm(self.sm_number)
            .set_clkdiv_int_frac(divint, divfrac);
        self.pio.sm(self.sm_number).clkdiv_restart();
        self.pio.sm(self.sm_number).set_enabled(true);

        Ok(rate)
    }

    fn get_rate(&self) -> u32 {
        // rate = SYSCLOCK_FREQ / (clkdiv int + (clkdiv frac/256))
        let sysclock_freq = self.clocks.map_or(SYSCLOCK_FREQ, |clocks| {
            clocks.get_frequency(clocks::Clock::Peripheral)
        });

        let divisor = self.clock_div_int.get() as f32 + (self.clock_div_frac.get() as f32 / 256f32);

        if divisor == 0f32 {
            // page 375 of the rp2040 datasheet says it defaults to 65536 if given 0
            return sysclock_freq / 65536u32;
        }

        (sysclock_freq as f32 / divisor) as u32
    }

    fn set_polarity(&self, polarity: ClockPolarity) -> Result<(), ErrorCode> {
        // unimplemented!("Polarity is fixed");
        Ok(())
    }

    fn get_polarity(&self) -> ClockPolarity {
        ClockPolarity::IdleHigh
    }

    fn set_phase(&self, phase: ClockPhase) -> Result<(), ErrorCode> {
        // TODO: how to channge program in the thing
        // unimplemented!("Not implemented yet");
        Ok(())
    }

    fn get_phase(&self) -> ClockPhase {
        self.clock_phase.get()
    }

    fn hold_low(&self) {}

    fn release_low(&self) {}
}

impl<'a> PioTxClient for PioSpi<'a> {
    fn on_buffer_space_available(&self) {
        self.tx_position.set(self.tx_position.get() + 1);
        match self.state.get() {
            PioSpiState::Writing | PioSpiState::ReadingWriting => {
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
            PioSpiState::Reading | PioSpiState::ReadingWriting => {
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
        // Transfer ownership to client
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
