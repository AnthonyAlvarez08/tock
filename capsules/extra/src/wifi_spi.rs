use core::cell::Cell;
use core::ops::Deref;
use kernel::debug;
use kernel::platform::mpu::MPU;
use kernel::static_init;
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::{ErrorCode, ProcessId};

use capsules_core::driver::{self, NUM};
use capsules_core::virtualizers::virtual_spi::MuxSpiMaster;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
use kernel::hil;
use kernel::hil::spi::{ClockPhase, ClockPolarity, SpiMaster, SpiMasterClient, SpiMasterDevice};

pub const DRIVER_NUM: usize = driver::NUM::WiFiSpi as usize;
pub const WIFI_SPI_TX_SIZE: usize = 13;
pub const WIFI_SPI_RX_SIZE: usize = 13;

pub const TX_BUF_LEN: usize = WIFI_SPI_TX_SIZE;
pub const RX_BUF_LEN: usize = WIFI_SPI_RX_SIZE;
pub struct WiFiSpi<'a, Spi: SpiMasterDevice<'a>> {
    spi_master: &'a Spi,
    state: Cell<State>,
    rxbuffer: MapCell<SubSliceMut<'static, u8>>,
    txbuffer: MapCell<SubSliceMut<'static, u8>>,
    grants: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<0>>,
    deferred_call: DeferredCall,
}

#[derive(Default)]
pub struct App {}

impl<'a, Spi: SpiMasterDevice<'a>> WiFiSpi<'a, Spi> {
    pub fn new(
        spi_master: &'a Spi,
        txbuffer: &'static mut [u8; WIFI_SPI_TX_SIZE],
        rxbuffer: &'static mut [u8; WIFI_SPI_RX_SIZE],
        grants: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<0>>,
    ) -> Self {
        Self {
            rxbuffer: MapCell::new((&mut rxbuffer[..]).into()),
            txbuffer: MapCell::new((&mut txbuffer[..]).into()),
            spi_master: spi_master,
            state: Cell::new(State::Suspend),
            grants: grants,
            deferred_call: DeferredCall::new(),
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

        debug!("[Capsule] printing read buffer over");
    }

    pub fn start(
        &self,
        writebuf: &'static mut [u8; WIFI_SPI_TX_SIZE],
        idx: i32,
    ) -> Result<(), ErrorCode> {
        debug!("[Capsule] starting {idx}");

        // self.out_buffer.put(Some(unsafe { ARR.as_mut_slice() }));

        // self.out_buffer.take().map(|buffer| {});

        self.txbuffer.put(SubSliceMut::new(writebuf));

        self.txbuffer.take().map(|txbuf| {
            debug!("[Capsule] reading and writing to buffers");

            // self.spi_master.hold_low();
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
        // set to 5MHz in the future
        // kinda hard to sync up to spi masters on the same chip so clock rate is at 5khz
        debug!("Trying to config SPI");
        self.spi_master
            .configure(ClockPolarity::IdleLow, ClockPhase::SampleLeading, 1_000)
        // Ok(())
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

        // self.spi_master.release_low();

        // the l3gd20 example does this
        self.txbuffer.replace(write_buffer);
        if let Some(buf) = read_buffer {
            self.rxbuffer.replace(buf);
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

        debug!("[Capsule client] current write buffer");
        self.txbuffer.map(|buf| {
            let mut idx: usize = 0;
            while idx < buf.len() {
                let temp = buf[idx];
                debug!("[Capsule Client] {temp}");
                idx += 1;
            }
        });
    }
}

impl<'a, Spi: SpiMasterDevice<'a>> SyscallDriver for WiFiSpi<'a, Spi> {
    fn command(
        &self,
        command_num: usize,
        data1: usize,
        data2: usize,
        process_id: ProcessId,
    ) -> CommandReturn {
        if command_num == 0 {
            return CommandReturn::success();
        }

        let id = process_id.id();
        debug!("WiFiSpi driver called with {data1} {data2}, for processid {id}");

        match command_num {
            // Check is sensor is correctly connected
            1 => {
                // do nothing for now
                return CommandReturn::success();
            }
            // default
            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, processid: ProcessId) -> Result<(), kernel::process::Error> {
        self.grants.enter(processid, |_, _| {})
        // Ok(())
    }
}

impl<'a, Spi: SpiMasterDevice<'a>> DeferredCallClient for WiFiSpi<'a, Spi> {
    fn handle_deferred_call(&self) {
        // Your action here
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
