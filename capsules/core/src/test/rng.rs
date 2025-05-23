// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Test entropy and random number generators.
//!
//! Usually, to test the full library, these generators should be
//! through two layers of translation for entropy then converted to
//! randomness. For example, if your platform provides an Entropy32,
//! then test Entropy32 -> Entropy32to8 -> Entropy8to32 ->
//! Entropy32ToRandom. Then simply ask for ELEMENTS random numbers and
//! print them in hex to console.

use core::cell::Cell;

use kernel::debug;
use kernel::hil::entropy;
use kernel::hil::rng;
use kernel::ErrorCode;

const ELEMENTS: usize = 8;

pub struct TestRandom<'a> {
    random: &'a dyn rng::Random<'a>,
}

impl<'a> TestRandom<'a> {
    pub fn new(random: &'a dyn rng::Random<'a>) -> TestRandom<'a> {
        TestRandom { random }
    }

    pub fn run(&self) {
        self.random.initialize();
        for _i in 0..ELEMENTS {
            let val = self.random.random();
            debug!("TestRandom: generated synchronous random number: {}", val);
        }
    }
}

// Use this test to test an Rng
pub struct TestRng<'a> {
    rng: &'a dyn rng::Rng<'a>,
    pool: Cell<[u32; ELEMENTS]>,
    count: Cell<usize>,
}

impl<'a> TestRng<'a> {
    pub fn new(rng: &'a dyn rng::Rng<'a>) -> TestRng<'a> {
        TestRng {
            rng,
            pool: Cell::new([0xeeeeeeee; ELEMENTS]),
            count: Cell::new(0),
        }
    }

    pub fn run(&self) {
        match self.rng.get() {
            Ok(()) => debug!("RNG test: first get Ok(())"),
            _ => panic!("RNG test: unable to get random numbers"),
        }
    }
}

impl rng::Client for TestRng<'_> {
    fn randomness_available(
        &self,
        randomness: &mut dyn Iterator<Item = u32>,
        error: Result<(), ErrorCode>,
    ) -> rng::Continue {
        let mut val = randomness.next();
        if error != Ok(()) {
            panic!(
                "RNG test: randomness_available called with error {:?}",
                error
            );
        }
        while val.is_some() {
            //debug!("RNG test: iterator returned Some.");
            let data = val.unwrap();

            let mut pool = self.pool.get();
            let mut count = self.count.get();
            pool[count] = data;
            count += 1;
            self.pool.set(pool);
            self.count.set(count);

            if count >= ELEMENTS {
                debug!("RNG test: obtained all {} values. They are:", count);
                for (i, c) in pool.iter().enumerate() {
                    debug!("[{:02x}]: {:08x}", i, c);
                }
                return rng::Continue::Done;
            } else {
                val = randomness.next();
            }
        }
        // val must be None: out of randomness, ask for more
        rng::Continue::More
    }
}

// Use this test to test a 32 bit Entropy source
pub struct TestEntropy32<'a> {
    egen: &'a dyn entropy::Entropy32<'a>,
    pool: Cell<[u32; ELEMENTS]>,
    count: Cell<usize>,
}

impl<'a> TestEntropy32<'a> {
    pub fn new(egen: &'a dyn entropy::Entropy32<'a>) -> TestEntropy32<'a> {
        TestEntropy32 {
            egen,
            pool: Cell::new([0xeeeeeeee; ELEMENTS]),
            count: Cell::new(0),
        }
    }

    pub fn run(&self) {
        match self.egen.get() {
            Ok(()) => debug!("Entropy32 test: first get Ok(())"),
            _ => panic!("Entropy32 test: unable to get entropy"),
        }
    }
}

impl entropy::Client32 for TestEntropy32<'_> {
    fn entropy_available(
        &self,
        entropy: &mut dyn Iterator<Item = u32>,
        error: Result<(), ErrorCode>,
    ) -> entropy::Continue {
        let mut val = entropy.next();
        if error != Ok(()) {
            panic!(
                "RNG test: randomness_available called with error {:?}",
                error
            );
        }
        while val.is_some() {
            //debug!("RNG test: iterator returned Some.");
            let data = val.unwrap();

            let mut pool = self.pool.get();
            let mut count = self.count.get();
            pool[count] = data;
            count += 1;
            self.pool.set(pool);
            self.count.set(count);

            if count >= ELEMENTS {
                debug!("Entropy test: obtained all {} values. They are:", count);
                for i in 0..pool.len() {
                    debug!("[{:02x}]: {:08x}", i, pool[i]);
                }
                return entropy::Continue::Done;
            } else {
                val = entropy.next();
            }
        }
        // val must be None: out of randomness, ask for more
        entropy::Continue::More
    }
}

// Use this test if the underlying Entropy source is an Entropy8
pub struct TestEntropy8<'a> {
    egen: &'a dyn entropy::Entropy8<'a>,
    pool: Cell<[u8; ELEMENTS]>,
    count: Cell<usize>,
}

impl<'a> TestEntropy8<'a> {
    pub fn new(egen: &'a dyn entropy::Entropy8<'a>) -> TestEntropy8<'a> {
        TestEntropy8 {
            egen,
            pool: Cell::new([0xee; ELEMENTS]),
            count: Cell::new(0),
        }
    }

    pub fn run(&self) {
        match self.egen.get() {
            Ok(()) => debug!("Entropy8 test: first get Ok(())"),
            _ => panic!("RNG test: unable to get random numbers"),
        }
    }
}

impl entropy::Client8 for TestEntropy8<'_> {
    fn entropy_available(
        &self,
        entropy: &mut dyn Iterator<Item = u8>,
        error: Result<(), ErrorCode>,
    ) -> entropy::Continue {
        let mut val = entropy.next();
        if error != Ok(()) {
            panic!(
                "Entropy8 test: entropy_available called with error {:?}",
                error
            );
        }
        while val.is_some() {
            debug!("Entropy8 test: entropy_available iterator returned Some, adding.");
            let data = val.unwrap();

            let mut pool = self.pool.get();
            let mut count = self.count.get();
            pool[count] = data;
            count += 1;
            self.pool.set(pool);
            self.count.set(count);

            if count >= ELEMENTS {
                debug!("RNG test: obtained {} values. They are:", count);
                for i in 0..pool.len() {
                    debug!("[{:02x}]: {:02x}", i, pool[i]);
                }
                return entropy::Continue::Done;
            } else {
                val = entropy.next();
            }
        }
        debug!("Entropy8 test: entropy_available iterator returned None, requesting more.");
        // val must be None: out of entropy, ask for more
        entropy::Continue::More
    }
}
