// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2024.
//
// Author: Jason Hu <jasonhu2026@u.northwestern.edu>
//         Anthony Alvarez <anthonyalvarez2026@u.northwestern.edu>

//! Programmable Input Output (PIO) hardware test file.
use core::borrow::Borrow;

use crate::clocks::{self};
use crate::gpio::{GpioFunction, RPGpio, RPGpioPin};
use crate::pio::{PIONumber, Pio, PioRxClient, PioTxClient, SMNumber, StateMachineConfiguration};
use kernel::debug;
use kernel::hil::gpio::Output;
use kernel::hil::spi::cs::ChipSelectPolar;
use kernel::hil::spi::SpiMaster;
use kernel::hil::spi::SpiMasterClient;
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::cells::TakeCell;
use kernel::utilities::leasable_buffer::{SubSlice, SubSliceMut};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::{hil, ErrorCode};

// TODO: refactor this so I can use just a PIO reference rather than a cell

pub struct PioSpi<'a> {
    clocks: &'a clocks::Clocks,
    pio: &'a Pio, //TakeCell<'a, Pio>,
    side_set_pin: u32,
    out_pin: u32,
    in_pin: u32,
    sm_number: SMNumber,
    pio_number: PIONumber,
    wiggle_pin: Option<&'a RPGpioPin<'a>>,
}

static QUEUE_CLIENT: QueueClient<'static> = QueueClient::<'_> { wahoo: &0 };

//* experimenting with having a singleton (or rather a doubleton) PioSpi Class
//* as there should only be two of them anyway

// static mut pio0 = Pio::new_pio0();
// static clocks = Clocks::new();

// static PioSpi0 : PioSpi<'static> = PioSpi::new(
//     &mut pio0,
//     &clocks,
//     10, // side set = clock
//     11, // in
//     12, // out
//     SMNumber::SM0,
//     PIONumber::PIO0,
// );

// static mut pio1 = Pio::new_pio1();

// static PioSpi1 : PioSpi<'static> = PioSpi::new(
//     &mut pio1,
//     &clocks,
//     19, // sideset = clock
//     20, // in
//     21, // out
//     SMNumber::SM1,
//     PIONumber::PIO1,
// );

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
            pio: pio, //TakeCell::new(pio),
            side_set_pin: side_set_pin,
            in_pin: in_pin,
            out_pin: out_pin,
            sm_number: sm_number,
            pio_number: pio_number,
            wiggle_pin: None::<&'a RPGpioPin<'a>>,
        }
    }

    pub fn set_wiggle_pin(&mut self, pin: &'a RPGpioPin<'a>) {
        self.wiggle_pin.insert(pin);
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
        // Read data from the RX FIFO
        self.pio.handle_interrupt();

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they write 0 out before they're reading
        if !self.pio.sm(self.sm_number).tx_full() {
            self.pio.sm(self.sm_number).push_blocking(0);
        }

        for _ in 0..10 {
            match self.wiggle_pin {
                Some(pin) => {
                    pin.toggle();
                }
                _ => {
                    debug!("no wiggle pin");
                }
            }
        }

        data = match self.pio.sm(self.sm_number).pull_blocking() {
            Ok(res) => res,
            Err(err) => {
                debug!("Error code : STATE MACHINE BUSY");
                return Err(err);
            }
        };
        // seeing the second pin wiggle means it returned something valid
        for _ in 0..20 {
            match self.wiggle_pin {
                Some(pin) => {
                    pin.toggle();
                }
                _ => {
                    debug!("no wiggle pin");
                }
            }
        }

        Ok(data)
    }

    pub fn write_word(&self, val: u32) -> Result<(), ErrorCode> {
        self.pio.handle_interrupt();

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they reading and dumping after they're writing
        self.pio.sm(self.sm_number).push_blocking(val);

        if !self.pio.sm(self.sm_number).rx_empty() {
            let _ = self.pio.sm(self.sm_number).pull_blocking().unwrap();
        }

        Ok(())
    }

    pub fn restart_clock(&self) {
        self.pio.sm(self.sm_number).clkdiv_restart();
    }

    pub fn clear_fifos(&self) {
        self.pio.sm(self.sm_number).clear_fifos();
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

        /*

        state machine configs

        pub out_pins_count: u32,
        pub out_pins_base: u32,
        pub set_pins_count: u32,
        pub set_pins_base: u32,
        pub in_pins_base: u32,
        pub side_set_base: u32,
        pub side_set_opt_enable: bool,
        pub side_set_bit_count: u32,
        pub side_set_pindirs: bool,
        pub wrap: u32,
        pub wrap_to: u32,
        pub in_shift_direction_right: bool,
        pub in_autopush: bool,
        pub in_push_threshold: u32,
        pub out_shift_direction_right: bool,
        pub out_autopull: bool,
        pub out_pull_threshold: u32,
        pub jmp_pin: u32,
        pub out_special_sticky: bool,
        pub out_special_has_enable_pin: bool,
        pub out_special_enable_pin_index: u32,
        pub mov_status_sel: PioMovStatusType,
        pub mov_status_n: u32,
        pub div_int: u32,
        pub div_frac: u32,*/

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
        // custom_config.in_push_threshold = 8;
        // custom_config.out_pull_threshold = 8;

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

        // subscribe to interrupts
        //*  currently doesnt work
        // debug!("Now enabling interrupts\n");

        // for i in self.pio.sm(self.sm_number).get_interrupt_sources() {
        //     self.pio.set_irq_source(0, i, true);
        // }
        // for i in self.pio.sm(self.sm_number).get_interrupt_sources() {
        //     self.pio.set_irq_source(1, i, true);
        // }

        // subscribe to the interrupts I guess?
        self.pio.sm(self.sm_number).set_tx_client(&QUEUE_CLIENT);
        self.pio.sm(self.sm_number).set_rx_client(&QUEUE_CLIENT);

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
        let mut reading: bool = false;

        // let writer = SubSlice::new(write_buffer.borrow().into());

        let mut reader = match read_buffer {
            Some(buf) => {
                reading = true;
                buf
            }
            _ => SubSliceMut::new(&mut []),
        };

        let mut readdex: usize = 0;
        let mut writedex: usize = 0;

        while writedex < write_buffer.len() {
            debug!(
                "[PIOSPI] idx {writedex} of writebuf: {}",
                write_buffer[writedex]
            );
            for _ in 0..10 {
                match self.wiggle_pin {
                    Some(pin) => {
                        pin.toggle();
                    }
                    _ => {
                        debug!("no wiggle pin");
                    }
                }
            }

            debug!("[PIOSPI] called read write byte");
            let res = self.read_write_byte(write_buffer[writedex]);
            debug!("[PIOSPI] passed read write byte");

            for _ in 0..10 {
                match self.wiggle_pin {
                    Some(pin) => {
                        pin.toggle();
                    }
                    _ => {
                        debug!("no wiggle pin");
                    }
                }
            }

            if reading {
                debug!("[PIOSPI] About to commit a read");
                match res {
                    Ok(val) => {
                        // do stuff
                        reader[readdex] = val;
                        readdex += 1;
                    }
                    Err(error) => {
                        return Err((error, write_buffer, Some(reader)));
                    }
                }
                debug!("[PIOSPI] Passed reading");
            }

            for _ in 0..10 {
                match self.wiggle_pin {
                    Some(pin) => {
                        pin.toggle();
                    }
                    _ => {
                        debug!("no wiggle pin");
                    }
                }
            }
            writedex += 1;
        }

        for _ in 0..10 {
            match self.wiggle_pin {
                Some(pin) => {
                    pin.toggle();
                }
                _ => {
                    debug!("no wiggle pin");
                }
            }
        }

        Ok(())
    }

    fn write_byte(&self, val: u8) -> Result<(), ErrorCode> {
        self.pio.handle_interrupt();

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they reading and dumping after they're writing
        self.pio.sm(self.sm_number).push_blocking(val as u32);

        if !self.pio.sm(self.sm_number).rx_empty() {
            match self.pio.sm(self.sm_number).pull_blocking() {
                Ok(val) => {}
                Err(err) => {}
            }
        }

        Ok(())
    }

    fn read_byte(&self) -> Result<u8, ErrorCode> {
        let mut data: u32 = 0;
        // Read data from the RX FIFO
        self.pio.handle_interrupt();

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they write 0 out before they're reading
        if !self.pio.sm(self.sm_number).tx_full() {
            self.pio.sm(self.sm_number).push_blocking(0);
        }

        data = match self.pio.sm(self.sm_number).pull_blocking() {
            Ok(val) => val,
            Err(error) => {
                return Err(error);
            }
        };

        Ok(data as u8)
    }

    fn read_write_byte(&self, val: u8) -> Result<u8, ErrorCode> {
        let mut data: u32 = 0;
        // Read data from the RX FIFO
        self.pio.handle_interrupt();

        // https://github.com/raspberrypi/pico-examples/blob/master/pio/spi/pio_spi.c
        // in this example they write 0 out before they're reading
        if !self.pio.sm(self.sm_number).tx_full() {
            let _ = self.pio.sm(self.sm_number).push_blocking(val as u32);
        }

        data = match self.pio.sm(self.sm_number).pull_blocking() {
            Ok(val) => val,
            Err(error) => {
                return Err(error);
            }
        };

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

// TODO: implement PioTxClient
// TODO: implement PioRxCLient

/*

pub trait PioTxClient {
    fn on_buffer_space_available(&self);
}

pub trait PioRxClient {
    fn on_data_received(&self, data: u32);
}

*/

impl<'a> PioTxClient for PioSpi<'a> {
    fn on_buffer_space_available(&self) {
        debug!("buffer space available");
    }
}

impl<'a> PioRxClient for PioSpi<'a> {
    fn on_data_received(&self, data: u32) {
        debug!("Received data {data}");
    }
}

struct QueueClient<'a> {
    wahoo: &'a i32,
}

impl<'a> QueueClient<'a> {
    pub fn new() -> Self {
        Self { wahoo: &0 }
    }
}

impl<'a> PioTxClient for QueueClient<'a> {
    fn on_buffer_space_available(&self) {
        debug!("INSIDE INTERRUPT HANDLER buffer space available\n");
    }
}

impl<'a> PioRxClient for QueueClient<'a> {
    fn on_data_received(&self, data: u32) {
        debug!("INSIDE INTERRUPT HANDLER Received data {data}\n");
    }
}
