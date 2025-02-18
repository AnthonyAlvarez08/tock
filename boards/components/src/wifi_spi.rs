use capsules_core::virtualizers::virtual_spi::{MuxSpiMaster, VirtualSpiMasterDevice};
use capsules_extra::wifi_spi::WiFiSpi;
use core::mem::MaybeUninit;
use kernel::capabilities;
use kernel::component::Component;
use kernel::create_capability;
use kernel::hil::spi;
use kernel::hil::spi::SpiMasterDevice;

// Setup static space for the objects.
#[macro_export]
macro_rules! wifi_spi_component_static {
    ($S:ty $(,)?) => {{
        let txbuffer = kernel::static_buf!([u8; capsules_extra::wifi_spi::TX_BUF_LEN]);
        let rxbuffer = kernel::static_buf!([u8; capsules_extra::wifi_spi::RX_BUF_LEN]);

        let spi = kernel::static_buf!(
            capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>
        );
        let wifi_spi = kernel::static_buf!(
            capsules_extra::wifi_spi::WiFiSpi<
                'static,
                capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>,
            >
        );

        (spi, wifi_spi, txbuffer, rxbuffer)
    };};
}

pub type WiFiSpiComponentType<S> = capsules_extra::wifi_spi::WiFiSpi<'static, S>;

pub struct WiFiSpiComponent<
    S: 'static + spi::SpiMaster<'static>,
    CS: spi::cs::IntoChipSelect<S::ChipSelect, spi::cs::ActiveLow>,
> {
    spi_mux: &'static MuxSpiMaster<'static, S>,
    chip_select: CS,
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
}

impl<
        S: 'static + spi::SpiMaster<'static>,
        CS: spi::cs::IntoChipSelect<S::ChipSelect, spi::cs::ActiveLow>,
    > WiFiSpiComponent<S, CS>
{
    pub fn new(
        spi_mux: &'static MuxSpiMaster<'static, S>,
        chip_select: CS,
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
    ) -> Self {
        Self {
            spi_mux,
            chip_select,
            board_kernel,
            driver_num,
        }
    }
}

impl<
        S: 'static + spi::SpiMaster<'static>,
        CS: spi::cs::IntoChipSelect<S::ChipSelect, spi::cs::ActiveLow>,
    > Component for WiFiSpiComponent<S, CS>
{
    type StaticInput = (
        &'static mut MaybeUninit<VirtualSpiMasterDevice<'static, S>>,
        &'static mut MaybeUninit<WiFiSpi<'static, VirtualSpiMasterDevice<'static, S>>>,
        &'static mut MaybeUninit<[u8; capsules_extra::wifi_spi::TX_BUF_LEN]>,
        &'static mut MaybeUninit<[u8; capsules_extra::wifi_spi::RX_BUF_LEN]>,
    );
    type Output = &'static WiFiSpi<'static, VirtualSpiMasterDevice<'static, S>>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);
        let grant = self.board_kernel.create_grant(self.driver_num, &grant_cap);

        let spi_device = static_buffer.0.write(VirtualSpiMasterDevice::new(
            self.spi_mux,
            self.chip_select.into_cs(),
        ));
        spi_device.setup();

        let txbuffer = static_buffer
            .2
            .write([0; capsules_extra::wifi_spi::TX_BUF_LEN]);
        let rxbuffer = static_buffer
            .3
            .write([0; capsules_extra::wifi_spi::RX_BUF_LEN]);

        let wifi_spi = static_buffer
            .1
            .write(WiFiSpi::new(spi_device, txbuffer, rxbuffer, grant));
        spi_device.set_client(wifi_spi);

        // TODO verify SPI return value
        let _ = wifi_spi.configure();

        wifi_spi
    }
}
