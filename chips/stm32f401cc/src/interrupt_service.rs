// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use crate::chip_specs::Stm32f401Specs;
use stm32f4xx::chip::Stm32f4xxDefaultPeripherals;

pub struct Stm32f401ccDefaultPeripherals<'a> {
    pub stm32f4: Stm32f4xxDefaultPeripherals<'a, Stm32f401Specs>,
    // Once implemented, place Stm32f401cc specific peripherals here
}

impl<'a> Stm32f401ccDefaultPeripherals<'a> {
    pub unsafe fn new(
        clocks: &'a crate::clocks::Clocks<'a, Stm32f401Specs>,
        exti: &'a crate::exti::Exti<'a>,
        dma1: &'a crate::dma::Dma1<'a>,
        dma2: &'a crate::dma::Dma2<'a>,
    ) -> Self {
        Self {
            stm32f4: Stm32f4xxDefaultPeripherals::new(clocks, exti, dma1, dma2),
        }
    }

    // Necessary for setting up circular dependencies & registering deferred calls
    pub fn init(&'static self) {
        self.stm32f4.setup_circular_deps();
    }
}
impl kernel::platform::chip::InterruptService for Stm32f401ccDefaultPeripherals<'_> {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        #[allow(clippy::match_single_binding)]
        match interrupt {
            // put Stm32f401cc specific interrupts here
            _ => self.stm32f4.service_interrupt(interrupt),
        }
    }
}
