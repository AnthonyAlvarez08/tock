// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
//
// Author: Jason Hu <jasonhu2026@u.northwestern.edu>
//         Anthony Alvarez <anthonyalvarez2026@u.northwestern.edu>

//! Programmable Input Output (PIO) hardware test file.
use crate::clocks::{self};
use crate::pio::{PIONumber, Pio, PioRxClient, PioTxClient, SMNumber, StateMachineConfiguration};
use core::cell::Cell;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::spi::cs::{ChipSelectPolar, Polarity};
use kernel::hil::spi::SpiMasterClient;
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::{hil, ErrorCode};

// Since auto push / pull is set to 8 for the purposes of writing in bytes
// rather than words, values read in have to be bitshifted by 24
const AUTOPULL_SHIFT: usize = 24;

// Frequency of system clock, for rate changes
const SYSCLOCK_FREQ: u32 = 125_000_000;

// Leading edge clock phase + Idle low clock
const SPI_CPHA0: [u16; 2] = [
    0x6101, /*  0: out    pins, 1         side 0 [1] */
    0x5101, /*  1: in     pins, 1         side 1 [1] */
];

// Trailing edge clock phase + Idle low clock
const SPI_CPHA1: [u16; 3] = [
    0x6021, /* 0: out    x, 1            side 0 */
    0xb101, /* 1: mov    pins, x         side 1 [1] */
    0x4001, /* 2: in     pins, 1         side 0 */
];

// Leading edge clock phase + Idle high clock
const SPI_CPHA0_HIGH_CLOCK: [u16; 2] = [
    0x7101, /*  0: out    pins, 1         side 1 [1] */
    0x4101, /*  1: in     pins, 1         side 0 [1] */
];

// Trailing edge clock phase + Idle high clock
const SPI_CPHA1_HIGH_CLOCK: [u16; 3] = [
    0x7021, /*  0: out    x, 1            side 1 */
    0xa101, /*  1: mov    pins, x         side 0 [1] */
    0x5001, /*  2: in     pins, 1         side 1 */
];

pub struct PioSpi<'a> {
    clocks: OptionalCell<&'a clocks::Clocks>,
    pio: &'a Pio,
    clock_pin: u32,
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
    clock_polarity: Cell<ClockPolarity>,
    chip_select: OptionalCell<ChipSelectPolar<'a, crate::gpio::RPGpioPin<'a>>>,
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
        clock_pin: u32,
        in_pin: u32,
        out_pin: u32,
        sm_number: SMNumber,
        pio_number: PIONumber,
    ) -> Self {
        Self {
            clocks: OptionalCell::new(clocks),
            pio: pio,
            clock_pin: clock_pin,
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
            clock_div_int: Cell::new(31u32), // defaults to 1 MHz
            clock_div_frac: Cell::new(64u32),
            clock_phase: Cell::new(ClockPhase::SampleLeading), // defaults to mode 0 0
            clock_polarity: Cell::new(ClockPolarity::IdleLow),
            chip_select: OptionalCell::empty(),
        }
    }

    // Helper function to read and writes to and from the buffers
    fn read_write_buffers(&self) {
        self.tx_buffer.map(|buf| {
            let temp = self.len.get();

            // FIFOs are 4 units deep, so that is the max one can read/write before having to wait
            const FIFO_DEPTH: usize = 4;

            let left_to_do = self.len.get() - self.tx_position.get() + 1;
            let runto = if FIFO_DEPTH > left_to_do {
                left_to_do
            } else {
                FIFO_DEPTH
            };

            for _i in 0..runto {
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
                    self.state.set(PioSpiState::Free);
                    self.len.set(0);
                    self.tx_position.set(0);
                    self.rx_position.set(0);
                    self.deferred_call.set();
                    break;
                }

                // If any read/write errors
                if errors {
                    break;
                }
            }
        });
    }
}

impl<'a> hil::spi::SpiMaster<'a> for PioSpi<'a> {
    type ChipSelect = ChipSelectPolar<'a, crate::gpio::RPGpioPin<'a>>;

    fn init(&self) -> Result<(), ErrorCode> {
        self.pio.init();

        // the trailing phase programs have a different length
        let mut wrap = 1;
        let program: &[u16] = if self.clock_phase.get() == ClockPhase::SampleLeading {
            if self.clock_polarity.get() == ClockPolarity::IdleLow {
                &SPI_CPHA0
            } else {
                &SPI_CPHA0_HIGH_CLOCK
            }
        } else {
            // sample trailing branch
            wrap = 2;
            if self.clock_polarity.get() == ClockPolarity::IdleLow {
                &SPI_CPHA1
            } else {
                &SPI_CPHA1_HIGH_CLOCK
            }
        };

        match self.pio.add_program16(None::<usize>, program) {
            Ok(_res) => {
                self.pio.sm(self.sm_number).exec_program(_res, true);
            }
            Err(_error) => return Err(ErrorCode::FAIL),
        }

        let mut custom_config = StateMachineConfiguration::default();

        custom_config.div_int = self.clock_div_int.get();
        custom_config.div_frac = self.clock_div_frac.get();

        // 8 bit mode on pio
        custom_config.in_push_threshold = 8;
        custom_config.out_pull_threshold = 8;

        custom_config.side_set_base = self.clock_pin;
        custom_config.in_pins_base = self.in_pin;
        custom_config.out_pins_base = self.out_pin;
        custom_config.side_set_bit_count = 1;
        custom_config.wrap = wrap;

        // automatically push and pull from the fifos
        custom_config.in_autopush = true;
        custom_config.out_autopull = true;

        self.pio.spi_program_init(
            self.sm_number,
            self.clock_pin,
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
        match self.state.get() {
            PioSpiState::Free => false,
            _ => true,
        }
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

        // If there is a chip select, make sure it is on
        self.hold_low();

        // Keep track of the new buffers
        self.len.replace(write_buffer.len());
        self.tx_buffer.replace(write_buffer);
        self.tx_position.set(0);

        self.state.replace(PioSpiState::Writing);

        if let Some(readbuf) = read_buffer {
            self.rx_buffer.replace(readbuf);
            self.state.replace(PioSpiState::ReadingWriting);
            self.rx_position.set(0);
        }

        // Begin reading/writing to/from buffers
        self.read_write_buffers();

        Ok(())
    }

    fn write_byte(&self, val: u8) -> Result<(), ErrorCode> {
        // One byte operations can be synchronous
        match self.pio.sm(self.sm_number).push_blocking(val as u32) {
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

        // One byte operations can be synchronous
        data = match self.pio.sm(self.sm_number).pull_blocking() {
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

        // One byte operations can be synchronous
        match self.pio.sm(self.sm_number).push_blocking(val as u32) {
            Err(err) => {
                return Err(err);
            }
            _ => {}
        }

        data = match self.pio.sm(self.sm_number).pull_blocking() {
            Ok(val) => val,
            Err(error) => {
                return Err(error);
            }
        };

        data = data >> AUTOPULL_SHIFT;

        Ok(data as u8)
    }

    fn specify_chip_select(&self, cs: Self::ChipSelect) -> Result<(), ErrorCode> {
        if !self.is_busy() {
            self.chip_select.set(cs);
            Ok(())
        } else {
            Err(ErrorCode::BUSY)
        }
    }

    fn set_rate(&self, rate: u32) -> Result<u32, ErrorCode> {
        if rate == 0 {
            return Err(ErrorCode::FAIL);
        }

        let sysclock_freq = self.clocks.map_or(SYSCLOCK_FREQ, |clocks| {
            clocks.get_frequency(clocks::Clock::System)
        });

        // Program does two instructions per every SPI clock peak
        // Still runs at half of rate after that so multiply again by two
        let rate = rate * 4;

        // Max clock rate is the sys clock
        if rate > sysclock_freq {
            return Err(ErrorCode::INVAL);
        }

        let divint = sysclock_freq / rate;
        // Div frac is in units of 1/256
        let divfrac = (sysclock_freq % rate) * 256u32 / rate;

        self.clock_div_int.replace(divint);
        self.clock_div_frac.replace(divfrac);

        // Reinit the PIO so it updates the times
        self.pio.sm(self.sm_number).set_enabled(false);
        self.pio
            .sm(self.sm_number)
            .set_clkdiv_int_frac(divint, divfrac);
        self.pio.sm(self.sm_number).clkdiv_restart();
        self.pio.sm(self.sm_number).set_enabled(true);

        Ok(rate)
    }

    fn get_rate(&self) -> u32 {
        let sysclock_freq = self.clocks.map_or(SYSCLOCK_FREQ, |clocks| {
            clocks.get_frequency(clocks::Clock::Peripheral)
        });

        let divisor = self.clock_div_int.get() as f32 + (self.clock_div_frac.get() as f32 / 256f32);

        if divisor == 0f32 {
            return sysclock_freq / 65536u32;
        }

        (sysclock_freq as f32 / divisor) as u32 / 4u32
    }

    fn set_polarity(&self, polarity: ClockPolarity) -> Result<(), ErrorCode> {
        if !self.is_busy() {
            self.clock_polarity.replace(polarity);
            self.init()
        } else {
            Err(ErrorCode::BUSY)
        }
    }

    fn get_polarity(&self) -> ClockPolarity {
        self.clock_polarity.get()
    }

    fn set_phase(&self, phase: ClockPhase) -> Result<(), ErrorCode> {
        if !self.is_busy() {
            self.clock_phase.replace(phase);
            self.init()
        } else {
            Err(ErrorCode::BUSY)
        }
    }

    fn get_phase(&self) -> ClockPhase {
        self.clock_phase.get()
    }

    fn hold_low(&self) {
        self.chip_select.map(|p| {
            // Just treat it as active low regardless of what they passed in
            match p.polarity {
                Polarity::Low => {
                    p.activate();
                }
                _ => {
                    p.deactivate();
                }
            }
        });
    }

    fn release_low(&self) {
        self.chip_select.map(|p| {
            // Just treat it as active low regardless of what they passed in
            match p.polarity {
                Polarity::Low => {
                    p.deactivate();
                }
                _ => {
                    p.activate();
                }
            }
        });
    }
}

impl<'a> PioTxClient for PioSpi<'a> {
    // Buffer space availble, so send next byte
    fn on_buffer_space_available(&self) {
        self.tx_position.set(self.tx_position.get() + 1);
        match self.state.get() {
            PioSpiState::Writing | PioSpiState::ReadingWriting => {
                self.read_write_buffers();
            }
            _ => {}
        }
    }
}

impl<'a> PioRxClient for PioSpi<'a> {
    // Data received, so update buffer and continue reading/writing
    fn on_data_received(&self, data: u32) {
        let data = data >> AUTOPULL_SHIFT;

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
    // Transfer ownership to client
    fn handle_deferred_call(&self) {
        self.state.set(PioSpiState::Free);

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
