use core::cell::Cell;
use core::ops::Deref;
use kernel::debug;
use kernel::platform::mpu::MPU;
use kernel::static_init;
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

use capsules_core::virtualizers::virtual_spi::MuxSpiMaster;
use kernel::hil;
use kernel::hil::spi::{ClockPhase, ClockPolarity, SpiMaster, SpiMasterClient, SpiMasterDevice};

pub const DRIVER_NUM: usize = 20;
pub const WIFI_SPI_TX_SIZE: usize = 20;
pub const WIFI_SPI_RX_SIZE: usize = 20;

pub const TX_BUF_LEN: usize = WIFI_SPI_TX_SIZE;
pub const RX_BUF_LEN: usize = WIFI_SPI_RX_SIZE;
pub struct WiFiSpi<'a, Spi: SpiMasterDevice<'a>> {
    spi_master: &'a Spi,
    state: Cell<State>,
    rxbuffer: MapCell<SubSliceMut<'static, u8>>,
    txbuffer: MapCell<SubSliceMut<'static, u8>>,
}

impl<'a, Spi: SpiMasterDevice<'a>> WiFiSpi<'a, Spi> {
    pub fn new(
        spi_master: &'a Spi,
        in_buffer: &'static mut [u8],
        out_buffer: &'static mut [u8],
    ) -> Self {
        Self {
            rxbuffer: MapCell::new(SubSliceMut::new(in_buffer)),
            txbuffer: MapCell::new(SubSliceMut::new(out_buffer)),
            spi_master: spi_master,
            state: Cell::new(State::Suspend),
        }
    }

    pub fn print_read(&self) {
        debug!("[Capsule] try to print read buffer");

        self.rxbuffer.take().map(|buf| {
            let mut idx: usize = 0;
            while idx < buf.len() {
                let temp = buf[idx];
                debug!("[Capsule Read] {temp}");
                idx += 1;
            }
        });
    }

    pub fn start(&self) -> Result<(), ErrorCode> {
        debug!("[Capsule] starting");

        // self.out_buffer.put(Some(unsafe { ARR.as_mut_slice() }));

        // self.out_buffer.take().map(|buffer| {});

        self.txbuffer.take().map(|txbuf| {
            debug!("[Capsule] reading and writing to buffers");
            let val = self
                .spi_master
                .read_write_bytes(txbuf, self.rxbuffer.take());

            match val {
                Err((errcode, writebuf, readbuf)) => {
                    debug!("[Capsule] error while reading and writing bytes");
                }
                _ => {}
            }
        });

        Ok(())
    }

    pub fn configure(&self) -> Result<(), ErrorCode> {
        self.spi_master.configure(
            ClockPolarity::IdleHigh,
            ClockPhase::SampleTrailing,
            1_000_000,
        )
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

impl<'a, Spi: SpiMasterDevice<'a>> SpiMasterClient for WiFiSpi<'a, Spi> {
    fn read_write_done(
        &self,
        write_buffer: SubSliceMut<'static, u8>,
        read_buffer: Option<SubSliceMut<'static, u8>>,
        status: Result<usize, ErrorCode>,
    ) {
        debug!("Spi Master client impl!!");
        debug!("Read Write Done");

        // the l3gd20 example does this
        self.txbuffer.replace(write_buffer);
        if let Some(buf) = read_buffer {
            self.rxbuffer.replace(buf);
        }

        debug!("[Capsule client] current write buffer");
        self.txbuffer.map(|buf| {
            let mut idx: usize = 0;
            while idx < buf.len() {
                let temp = buf[idx];
                debug!("[Capsule Client] {temp}");
                idx += 1;
            }
        });

        debug!("[Capsule client] current read buffer");
        self.rxbuffer.map(|buf| {
            let mut idx: usize = 0;

            while idx < buf.len() {
                let temp = buf[idx];
                debug!("[Capsule Client] {temp}");
                idx += 1;
            }
        });
    }
}
