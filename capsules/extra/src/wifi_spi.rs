use core::cell::Cell;
use kernel::debug;
use kernel::platform::mpu::MPU;
use kernel::static_init;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

use capsules_core::virtualizers::virtual_spi::MuxSpiMaster;
use kernel::hil;
use kernel::hil::spi::{SpiMaster, SpiMasterClient, SpiMasterDevice};

pub struct WiFiSpi<'a, Spi: SpiMaster<'a>> {
    spi_master: &'a Spi,
    state: Cell<State>,
    in_buffer: TakeCell<'static, [u8]>,
    out_buffer: TakeCell<'static, [u8]>,
}

static mut ARR: [u8; 9] = [
    0xA7u8, 0xD4u8, 0x51u8, 0xB1u8, 0x72u8, 0xF6u8, 0xC5u8, 0x71u8, 0xE3u8,
];

impl<'a, Spi: SpiMaster<'a>> WiFiSpi<'a, Spi> {
    pub fn new(
        spi_master: &'a Spi,
        in_buffer: &'static mut [u8],
        out_buffer: &'static mut [u8],
    ) -> Self {
        Self {
            in_buffer: TakeCell::new(in_buffer),
            out_buffer: TakeCell::new(out_buffer),
            spi_master: spi_master,
            state: Cell::new(State::Suspend),
        }
    }

    pub fn start(&self) -> Result<(), ErrorCode> {
        debug!("writing word");

        // self.out_buffer.put(Some(unsafe { ARR.as_mut_slice() }));

        // self.out_buffer.take().map(|buffer| {});

        match self.out_buffer.take() {
            Some(buf) => {
                let out_buffer = SubSliceMut::new(buf);

                let in_buffer: SubSliceMut<'static, u8> = SubSliceMut::new(&mut []);

                // _pio_spi.block_until_ready_to_write();
                let res = self
                    .spi_master
                    .read_write_bytes(out_buffer, Some(in_buffer));
            }
            _ => {}
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
enum State {
    Suspend,
    Sleep,
    PowerOn,
    InitializeReading,
    ReadMeasurement,
    Read,
}

impl<'a, Spi: SpiMaster<'a>> SpiMasterClient for WiFiSpi<'a, Spi> {
    fn read_write_done(
        &self,
        write_buffer: SubSliceMut<'static, u8>,
        read_buffer: Option<SubSliceMut<'static, u8>>,
        status: Result<usize, ErrorCode>,
    ) {
        debug!("Spi Master client impl!!");
        debug!("Read Write Done");

        match read_buffer {
            Some(buf) => {
                debug!("Writing received bytes");
                for i in buf.take() {
                    debug!("{i}");
                }
            }
            _ => {}
        }

        debug!("Here is current write buffer");
        for i in write_buffer.take() {
            debug!("{i}");
        }
    }
}
