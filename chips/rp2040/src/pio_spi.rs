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
