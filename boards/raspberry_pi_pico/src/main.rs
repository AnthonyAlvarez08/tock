// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Tock kernel for the Raspberry Pi Pico.
//!
//! It is based on RP2040SoC SoC (Cortex M0+).

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use core::borrow::BorrowMut;
use core::ptr::{addr_of, addr_of_mut};

use capsules_core::gpio::GPIO;
use capsules_core::i2c_master::I2CMasterDriver;
use capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm;
use capsules_extra::wifi_spi::WiFiSpi;
use components::date_time_component_static;
use components::gpio::GpioComponent;
use components::led::LedsComponent;
use components::wifi_spi::WiFiSpiComponent;
use enum_primitive::cast::FromPrimitive;
use kernel::component::Component;
use kernel::debug;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::gpio::Output;
use kernel::hil::gpio::{Configure, FloatingState};
use kernel::hil::i2c::I2CMaster;
use kernel::hil::led::LedHigh;
use kernel::hil::pwm::Pwm;
use kernel::hil::spi::{ClockPhase, SpiMaster};
use kernel::hil::usb::Client;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::syscall::SyscallDriver;
use kernel::utilities::cells::TakeCell;
use kernel::{capabilities, create_capability, static_init, Kernel};
use rp2040::adc::{Adc, Channel};
use rp2040::chip::{Rp2040, Rp2040DefaultPeripherals};
use rp2040::clocks::Clocks;
use rp2040::clocks::{
    AdcAuxiliaryClockSource, PeripheralAuxiliaryClockSource, PllClock,
    ReferenceAuxiliaryClockSource, ReferenceClockSource, RtcAuxiliaryClockSource,
    SystemAuxiliaryClockSource, SystemClockSource, UsbAuxiliaryClockSource,
};
use rp2040::gpio::{GpioFunction, RPGpio, RPGpioPin};
use rp2040::i2c::I2c;
use rp2040::pio::Pio;
use rp2040::pio::{PIONumber, SMNumber, StateMachineConfiguration};
use rp2040::pio_pwm::PioPwm;
// use rp2040::pio_spi::PioInterruptClient;
use rp2040::cyw43::CYW43_DRIVER;
use rp2040::pio_spi::PioSpi;
use rp2040::resets::Peripheral;
use rp2040::sysinfo;
use rp2040::timer::RPTimer;

mod io;

mod flash_bootloader;

/// Allocate memory for the stack
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1500] = [0; 0x1500];

// Manually setting the boot header section that contains the FCB header
#[used]
#[link_section = ".flash_bootloader"]
static FLASH_BOOTLOADER: [u8; 256] = flash_bootloader::FLASH_BOOTLOADER;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: capsules_system::process_policies::PanicFaultPolicy =
    capsules_system::process_policies::PanicFaultPolicy {};

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static Rp2040<Rp2040DefaultPeripherals>> = None;
static mut PROCESS_PRINTER: Option<&'static capsules_system::process_printer::ProcessPrinterText> =
    None;

type TemperatureRp2040Sensor = components::temperature_rp2040::TemperatureRp2040ComponentType<
    capsules_core::virtualizers::virtual_adc::AdcDevice<'static, rp2040::adc::Adc<'static>>,
>;
type TemperatureDriver = components::temperature::TemperatureComponentType<TemperatureRp2040Sensor>;

/// Supported drivers by the platform
pub struct RaspberryPiPico {
    ipc: kernel::ipc::IPC<{ NUM_PROCS as u8 }>,
    console: &'static capsules_core::console::Console<'static>,
    alarm: &'static capsules_core::alarm::AlarmDriver<
        'static,
        VirtualMuxAlarm<'static, rp2040::timer::RPTimer<'static>>,
    >,
    gpio: &'static capsules_core::gpio::GPIO<'static, RPGpioPin<'static>>,
    led: &'static capsules_core::led::LedDriver<'static, LedHigh<'static, RPGpioPin<'static>>, 1>,
    adc: &'static capsules_core::adc::AdcVirtualized<'static>,
    temperature: &'static TemperatureDriver,
    i2c: &'static capsules_core::i2c_master::I2CMasterDriver<'static, I2c<'static, 'static>>,

    date_time:
        &'static capsules_extra::date_time::DateTimeCapsule<'static, rp2040::rtc::Rtc<'static>>,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm0p::systick::SysTick,

    pio0: &'static Pio,
    pio1: &'static Pio,
}

impl SyscallDriverLookup for RaspberryPiPico {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules_core::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules_core::led::DRIVER_NUM => f(Some(self.led)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            capsules_core::adc::DRIVER_NUM => f(Some(self.adc)),
            capsules_extra::temperature::DRIVER_NUM => f(Some(self.temperature)),
            capsules_core::i2c_master::DRIVER_NUM => f(Some(self.i2c)),
            capsules_extra::date_time::DRIVER_NUM => f(Some(self.date_time)),
            _ => f(None),
        }
    }
}

impl KernelResources<Rp2040<'static, Rp2040DefaultPeripherals<'static>>> for RaspberryPiPico {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm0p::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

#[allow(dead_code)]
extern "C" {
    /// Entry point used for debugger
    ///
    /// When loaded using gdb, the Raspberry Pi Pico is not reset
    /// by default. Without this function, gdb sets the PC to the
    /// beginning of the flash. This is not correct, as the RP2040
    /// has a more complex boot process.
    ///
    /// This function is set to be the entry point for gdb and is used
    /// to send the RP2040 back in the bootloader so that all the boot
    /// sequence is performed.
    fn jump_to_bootloader();
}

#[cfg(any(doc, all(target_arch = "arm", target_os = "none")))]
core::arch::global_asm!(
    "
    .section .jump_to_bootloader, \"ax\"
    .global jump_to_bootloader
    .thumb_func
  jump_to_bootloader:
    movs r0, #0
    ldr r1, =(0xe0000000 + 0x0000ed08)
    str r0, [r1]
    ldmia r0!, {{r1, r2}}
    msr msp, r1
    bx r2
    "
);

fn init_clocks(peripherals: &Rp2040DefaultPeripherals) {
    // Start tick in watchdog
    peripherals.watchdog.start_tick(12);

    // Disable the Resus clock
    peripherals.clocks.disable_resus();

    // Setup the external Oscillator
    peripherals.xosc.init();

    // disable ref and sys clock aux sources
    peripherals.clocks.disable_sys_aux();
    peripherals.clocks.disable_ref_aux();

    peripherals
        .resets
        .reset(&[Peripheral::PllSys, Peripheral::PllUsb]);
    peripherals
        .resets
        .unreset(&[Peripheral::PllSys, Peripheral::PllUsb], true);

    // Configure PLLs (from Pico SDK)
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz

    // It seems that the external oscillator is clocked at 12 MHz

    peripherals
        .clocks
        .pll_init(PllClock::Sys, 12, 1, 1500 * 1000000, 6, 2);
    peripherals
        .clocks
        .pll_init(PllClock::Usb, 12, 1, 480 * 1000000, 5, 2);

    // pico-sdk: // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    peripherals.clocks.configure_reference(
        ReferenceClockSource::Xosc,
        ReferenceAuxiliaryClockSource::PllUsb,
        12000000,
        12000000,
    );
    // pico-sdk: CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    peripherals.clocks.configure_system(
        SystemClockSource::Auxiliary,
        SystemAuxiliaryClockSource::PllSys,
        125000000,
        125000000,
    );
    // pico-sdk: CLK USB = PLL USB (48MHz) / 1 = 48MHz
    peripherals
        .clocks
        .configure_usb(UsbAuxiliaryClockSource::PllSys, 48000000, 48000000);
    // pico-sdk: CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    peripherals
        .clocks
        .configure_adc(AdcAuxiliaryClockSource::PllUsb, 48000000, 48000000);
    // pico-sdk: CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    peripherals
        .clocks
        .configure_rtc(RtcAuxiliaryClockSource::PllSys, 48000000, 46875);
    // pico-sdk:
    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    peripherals
        .clocks
        .configure_peripheral(PeripheralAuxiliaryClockSource::System, 125000000);
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
pub unsafe fn start() -> (
    &'static kernel::Kernel,
    RaspberryPiPico,
    &'static rp2040::chip::Rp2040<'static, Rp2040DefaultPeripherals<'static>>,
) {
    // Loads relocations and clears BSS
    rp2040::init();

    let peripherals = static_init!(Rp2040DefaultPeripherals, Rp2040DefaultPeripherals::new());
    peripherals.resolve_dependencies();

    // Reset all peripherals except QSPI (we might be booting from Flash), PLL USB and PLL SYS
    peripherals.resets.reset_all_except(&[
        Peripheral::IOQSpi,
        Peripheral::PadsQSpi,
        Peripheral::PllUsb,
        Peripheral::PllSys,
    ]);

    // Unreset all the peripherals that do not require clock setup as they run using the sys_clk or ref_clk
    // Wait for the peripherals to reset
    peripherals.resets.unreset_all_except(
        &[
            Peripheral::Adc,
            Peripheral::Rtc,
            Peripheral::Spi0,
            Peripheral::Spi1,
            Peripheral::Uart0,
            Peripheral::Uart1,
            Peripheral::UsbCtrl,
        ],
        true,
    );

    init_clocks(peripherals);

    // Unreset all peripherals
    peripherals.resets.unreset_all_except(&[], true);

    // Set the UART used for panic
    (*addr_of_mut!(io::WRITER)).set_uart(&peripherals.uart0);

    //set RX and TX pins in UART mode
    let gpio_tx = peripherals.pins.get_pin(RPGpio::GPIO0);
    let gpio_rx = peripherals.pins.get_pin(RPGpio::GPIO1);
    gpio_rx.set_function(GpioFunction::UART);
    gpio_tx.set_function(GpioFunction::UART);

    // Disable IE for pads 26-29 (the Pico SDK runtime does this, not sure why)
    for pin in 26..30 {
        peripherals
            .pins
            .get_pin(RPGpio::from_usize(pin).unwrap())
            .deactivate_pads();
    }

    let chip = static_init!(
        Rp2040<Rp2040DefaultPeripherals>,
        Rp2040::new(peripherals, &peripherals.sio)
    );

    CHIP = Some(chip);

    let board_kernel = static_init!(Kernel, Kernel::new(&*addr_of!(PROCESSES)));

    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);

    let mux_alarm = components::alarm::AlarmMuxComponent::new(&peripherals.timer)
        .finalize(components::alarm_mux_component_static!(RPTimer));

    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules_core::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_static!(RPTimer));

    // CDC
    let strings = static_init!(
        [&str; 3],
        [
            "Raspberry Pi",      // Manufacturer
            "Pico - TockOS",     // Product
            "00000000000000000", // Serial number
        ]
    );

    let cdc = components::cdc::CdcAcmComponent::new(
        &peripherals.usb,
        //capsules_extra::usb::cdc::MAX_CTRL_PACKET_SIZE_RP2040,
        64,
        peripherals.sysinfo.get_manufacturer_rp2040() as u16,
        peripherals.sysinfo.get_part() as u16,
        strings,
        mux_alarm,
        None,
    )
    .finalize(components::cdc_acm_component_static!(
        rp2040::usb::UsbCtrl,
        rp2040::timer::RPTimer
    ));

    // UART
    // Create a shared UART channel for kernel debug.
    let uart_mux = components::console::UartMuxComponent::new(cdc, 115200)
        .finalize(components::uart_mux_component_static!());

    // Uncomment this to use UART as an output
    // let uart_mux2 = components::console::UartMuxComponent::new(
    //     &peripherals.uart0,
    //     115200,
    // )
    // .finalize(components::uart_mux_component_static!());

    // Setup the console.
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux)
        .finalize(components::debug_writer_component_static!());

    cdc.enable();
    cdc.attach();

    let gpio = GpioComponent::new(
        board_kernel,
        capsules_core::gpio::DRIVER_NUM,
        components::gpio_component_helper!(
            RPGpioPin,
            // Used for serial communication. Comment them in if you don't use serial.
            // 0 => peripherals.pins.get_pin(RPGpio::GPIO0),
            // 1 => peripherals.pins.get_pin(RPGpio::GPIO1),
            2 => peripherals.pins.get_pin(RPGpio::GPIO2),
            3 => peripherals.pins.get_pin(RPGpio::GPIO3),
            // Used for i2c. Comment them in if you don't use i2c.
            // 4 => peripherals.pins.get_pin(RPGpio::GPIO4),
            // 5 => peripherals.pins.get_pin(RPGpio::GPIO5),
            6 => peripherals.pins.get_pin(RPGpio::GPIO6),
            7 => peripherals.pins.get_pin(RPGpio::GPIO7),
            8 => peripherals.pins.get_pin(RPGpio::GPIO8),
            9 => peripherals.pins.get_pin(RPGpio::GPIO9),
            10 => peripherals.pins.get_pin(RPGpio::GPIO10),
            11 => peripherals.pins.get_pin(RPGpio::GPIO11),
            12 => peripherals.pins.get_pin(RPGpio::GPIO12),
            13 => peripherals.pins.get_pin(RPGpio::GPIO13),
            14 => peripherals.pins.get_pin(RPGpio::GPIO14),
            15 => peripherals.pins.get_pin(RPGpio::GPIO15),
            16 => peripherals.pins.get_pin(RPGpio::GPIO16),
            17 => peripherals.pins.get_pin(RPGpio::GPIO17),
            18 => peripherals.pins.get_pin(RPGpio::GPIO18),
            19 => peripherals.pins.get_pin(RPGpio::GPIO19),
            20 => peripherals.pins.get_pin(RPGpio::GPIO20),
            21 => peripherals.pins.get_pin(RPGpio::GPIO21),
            22 => peripherals.pins.get_pin(RPGpio::GPIO22),
            23 => peripherals.pins.get_pin(RPGpio::GPIO23),
            24 => peripherals.pins.get_pin(RPGpio::GPIO24),
            // LED pin
            // 25 => peripherals.pins.get_pin(RPGpio::GPIO25),

            // Uncomment to use these as GPIO pins instead of ADC pins
            // 26 => peripherals.pins.get_pin(RPGpio::GPIO26),
            // 27 => peripherals.pins.get_pin(RPGpio::GPIO27),
            // 28 => peripherals.pins.get_pin(RPGpio::GPIO28),
            // 29 => peripherals.pins.get_pin(RPGpio::GPIO29)
        ),
    )
    .finalize(components::gpio_component_static!(RPGpioPin<'static>));

    let led = LedsComponent::new().finalize(components::led_component_static!(
        LedHigh<'static, RPGpioPin<'static>>,
        LedHigh::new(peripherals.pins.get_pin(RPGpio::GPIO25))
    ));

    peripherals.adc.init();

    let adc_mux = components::adc::AdcMuxComponent::new(&peripherals.adc)
        .finalize(components::adc_mux_component_static!(Adc));

    let temp_sensor = components::temperature_rp2040::TemperatureRp2040Component::new(
        adc_mux,
        Channel::Channel4,
        1.721,
        0.706,
    )
    .finalize(components::temperature_rp2040_adc_component_static!(
        rp2040::adc::Adc
    ));

    // RTC DATE TIME

    match peripherals.rtc.rtc_init() {
        Ok(()) => {}
        Err(e) => {
            debug!("error starting rtc {:?}", e);
        }
    };

    let date_time = components::date_time::DateTimeComponent::new(
        board_kernel,
        capsules_extra::date_time::DRIVER_NUM,
        &peripherals.rtc,
    )
    .finalize(date_time_component_static!(rp2040::rtc::Rtc<'static>));

    let temp = components::temperature::TemperatureComponent::new(
        board_kernel,
        capsules_extra::temperature::DRIVER_NUM,
        temp_sensor,
    )
    .finalize(components::temperature_component_static!(
        TemperatureRp2040Sensor
    ));

    let adc_channel_0 = components::adc::AdcComponent::new(adc_mux, Channel::Channel0)
        .finalize(components::adc_component_static!(Adc));

    let adc_channel_1 = components::adc::AdcComponent::new(adc_mux, Channel::Channel1)
        .finalize(components::adc_component_static!(Adc));

    let adc_channel_2 = components::adc::AdcComponent::new(adc_mux, Channel::Channel2)
        .finalize(components::adc_component_static!(Adc));

    let adc_channel_3 = components::adc::AdcComponent::new(adc_mux, Channel::Channel3)
        .finalize(components::adc_component_static!(Adc));

    let adc_syscall =
        components::adc::AdcVirtualComponent::new(board_kernel, capsules_core::adc::DRIVER_NUM)
            .finalize(components::adc_syscall_component_helper!(
                adc_channel_0,
                adc_channel_1,
                adc_channel_2,
                adc_channel_3,
            ));
    // PROCESS CONSOLE
    let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
        .finalize(components::process_printer_text_component_static!());
    PROCESS_PRINTER = Some(process_printer);

    let process_console = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
        Some(cortexm0p::support::reset),
    )
    .finalize(components::process_console_component_static!(RPTimer));
    let _ = process_console.start();

    let sda_pin = peripherals.pins.get_pin(RPGpio::GPIO4);
    let scl_pin = peripherals.pins.get_pin(RPGpio::GPIO5);

    sda_pin.set_function(GpioFunction::I2C);
    scl_pin.set_function(GpioFunction::I2C);

    sda_pin.set_floating_state(FloatingState::PullUp);
    scl_pin.set_floating_state(FloatingState::PullUp);

    let i2c_master_buffer = static_init!(
        [u8; capsules_core::i2c_master::BUFFER_LENGTH],
        [0; capsules_core::i2c_master::BUFFER_LENGTH]
    );
    let i2c0 = &peripherals.i2c0;
    let i2c = static_init!(
        I2CMasterDriver<I2c<'static, 'static>>,
        I2CMasterDriver::new(
            i2c0,
            i2c_master_buffer,
            board_kernel.create_grant(
                capsules_core::i2c_master::DRIVER_NUM,
                &memory_allocation_capability
            ),
        )
    );
    i2c0.init(10 * 1000);
    i2c0.set_master_client(i2c);

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&*addr_of!(PROCESSES))
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    // TODO: does not compile due to lifetime shenanigans

    let pin6 = peripherals.pins.get_pin(RPGpio::GPIO6);
    pin6.make_output();
    let pin7 = peripherals.pins.get_pin(RPGpio::GPIO7);
    pin7.make_output();
    let pin8 = peripherals.pins.get_pin(RPGpio::GPIO8);
    pin8.make_output();
    let pin9 = peripherals.pins.get_pin(RPGpio::GPIO9);
    pin9.make_output();

    for _ in 0..10 {
        pin6.toggle(); // gray line
        pin7.toggle(); // brown line
        pin8.toggle(); // red line, inside writer spi
        pin9.toggle(); // orange line, inside receiver spi
    }

    // try to turn on the wifi chip
    let pin23 = peripherals.pins.get_pin(RPGpio::GPIO23);
    pin23.make_output();
    pin23.set_function(GpioFunction::PIO0);
    pin23.set();
    let pin25 = peripherals.pins.get_pin(RPGpio::GPIO25);
    pin25.make_output();
    pin25.set_function(GpioFunction::PIO0);
    pin25.set();

    // let pio = static_init!(Pio, Pio::new_pio1());
    let _pio_spi: &'static mut PioSpi<'static> = static_init!(
        PioSpi,
        PioSpi::<'static>::new(
            &peripherals.pio0,
            &peripherals.clocks,
            10, // side set = clock
            11, // in
            12, // out
            SMNumber::SM0,
            PIONumber::PIO0,
            ClockPhase::SampleLeading,
        )
    );

    // make the pio subscribe to interrupts
    peripherals.pio0.sm(SMNumber::SM0).set_rx_client(_pio_spi);
    peripherals.pio0.sm(SMNumber::SM0).set_tx_client(_pio_spi);

    // let pio2 = static_init!(Pio, Pio::new_pio1());
    let _receive_spi: &'static mut PioSpi<'static> = static_init!(
        PioSpi,
        PioSpi::<'static>::new(
            &peripherals.pio1,
            &peripherals.clocks,
            19, // sideset = clock
            20, // in
            21, // out
            SMNumber::SM1,
            PIONumber::PIO1,
            ClockPhase::SampleLeading,
        )
    );

    // make the pio subscribe to interrupts
    peripherals
        .pio1
        .sm(SMNumber::SM1)
        .set_rx_client(_receive_spi);
    peripherals
        .pio1
        .sm(SMNumber::SM1)
        .set_tx_client(_receive_spi);

    // // debug!("Attempting to initialize PIO");
    let _ = _pio_spi.init();

    let _ = _receive_spi.init();

    _pio_spi.register();
    _receive_spi.register();

    let _wifi_spi: &'static mut CYW43_DRIVER<'static> = static_init!(
        CYW43_DRIVER,
        CYW43_DRIVER::<'static>::new(&peripherals.pio0, SMNumber::SM1)
    );

    match _wifi_spi.init() {
        Err(_err) => {
            debug!("WiFi SPI Init Error")
        }
        _ => {}
    }
    match _wifi_spi.try_read() {
        Err(_err) => {
            debug!("Try Read Error")
        }
        Ok(res) => {
            debug!("This is what we read: {res}")
        }
    }

    // _receive_spi.clear_fifos();
    // _pio_spi.clear_fifos();
    // _receive_spi.block_until_empty();
    // _pio_spi.block_until_empty();
    // debug!("empty rx on receive spi");

    // put like 4 bytes in a queue, and read
    // seems to have space for 5 items only
    // should be read as [167, 212, 81, 177, 114, 246, 197, 113, 227]
    // for i in [
    //     0xA7u8, 0xB4u8, 0xC3u8,
    //     0xD5u8, /*0xD4u8, 0x51u8, 0xB1u8, 0x72u8, 0xF6u8, 0xC5u8, 0x71u8, 0xE3u8,*/
    // ] {
    //     // debug!("writing word");
    //     for _ in 0..6 {
    //         // shouw show 3 peaks
    //         pin6.toggle();
    //     }
    //     // _pio_spi.block_until_ready_to_write();
    //     _pio_spi.write_word(i as u32);
    //     // debug!("finished writing word");

    //     for _ in 0..6 {
    //         pin7.toggle();
    //     }

    //     let val = match _receive_spi.read_word() {
    //         Ok(data) => data,
    //         Err(err) => {
    //             // debug!("receive spi error");
    //             for _ in 0..30 {
    //                 // should show 15 peaks
    //                 pin7.toggle();
    //             }
    //             0
    //         }
    //     };
    //     // debug!("recv this value: {val}");

    //     for _ in 0..10 {
    //         // shoud show 5 peaks
    //         pin6.toggle();
    //     }

    //     let _ = _receive_spi.write_word(val);

    //     for _ in 0..10 {
    //         // should show 5 peaks
    //         pin7.toggle();
    //     }
    // }

    // WIFI chip actual pins
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/boards/include/boards/pico2_w.h#L124
    // 29 = clock
    // 25 = chip select
    // 24 = both input and output
    // 23 = power on

    // let spi_mux = components::spi::SpiMuxComponent::new(_receive_spi)
    //     .finalize(components::spi_mux_component_static!(PioSpi));

    // let wifi_spi = components::wifi_spi::WiFiSpiComponent::new(
    //     spi_mux,
    //     peripherals.pins.get_pin(RPGpio::GPIO25),
    //     board_kernel,
    //     capsules_extra::wifi_spi::DRIVER_NUM,
    // )
    // .finalize(components::wifi_spi_component_static!(
    //     // spi type
    //     PioSpi
    // ));
    // wifi_spi.register();
    // _receive_spi.set_client(wifi_spi);

    // let spi_mux2 = components::spi::SpiMuxComponent::new(_pio_spi)
    //     .finalize(components::spi_mux_component_static!(PioSpi));

    // let wifi_spi2 = components::wifi_spi::WiFiSpiComponent::new(
    //     spi_mux2,
    //     peripherals.pins.get_pin(RPGpio::GPIO25),
    //     board_kernel,
    //     capsules_extra::wifi_spi::DRIVER_NUM,
    // )
    // .finalize(components::wifi_spi_component_static!(
    //     // spi type
    //     PioSpi
    // ));
    // wifi_spi2.register();
    // _pio_spi.set_client(wifi_spi2);

    // // I temporarily made the buffer sizes 8 for the spi capsule
    // static mut outbuf: [u8; 8] = [
    //     0x6Au8, 0xB1u8, 0x43u8, 0xF1u8, 0x42u8, 0xE2u8, 0x79u8, 0x2Cu8,
    // ];
    // // _pio_spi.write_word(0xA7);
    // let _ = wifi_spi.start(&mut outbuf, 0);
    // for _ in 0..20 {
    //     pin8.toggle();
    // }

    // let _ = wifi_spi2.start(&mut outbuf, 0);
    // for _ in 0..20 {
    //     pin9.toggle();
    // }

    // wifi_spi.print_read();

    for _ in 0..40 {
        // should show 20 peaks
        pin7.toggle();
    }

    let raspberry_pi_pico = RaspberryPiPico {
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        alarm,
        gpio,
        led,
        console,
        adc: adc_syscall,
        temperature: temp,
        i2c,
        date_time,

        scheduler,
        systick: cortexm0p::systick::SysTick::new_with_calibration(125_000_000),
        pio0: &peripherals.pio0,
        pio1: &peripherals.pio1,
    };

    let platform_type = match peripherals.sysinfo.get_platform() {
        sysinfo::Platform::Asic => "ASIC",
        sysinfo::Platform::Fpga => "FPGA",
    };

    for _ in 0..10 {
        // should show 5 peaks
        pin6.toggle();
    }

    debug!(
        "RP2040 Revision {} {}",
        peripherals.sysinfo.get_revision(),
        platform_type
    );

    debug!("Initialization complete. Enter main loop");

    // These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            core::ptr::addr_of!(_sapps),
            core::ptr::addr_of!(_eapps) as usize - core::ptr::addr_of!(_sapps) as usize,
        ),
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(_sappmem),
            core::ptr::addr_of!(_eappmem) as usize - core::ptr::addr_of!(_sappmem) as usize,
        ),
        &mut *addr_of_mut!(PROCESSES),
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    for _ in 0..10 {
        // should show 5 peaks
        pin7.toggle();
        pin8.toggle();
    }

    // let mut pio: Pio = Pio::new_pio0();

    // let pio_pwm = PioPwm::new(&mut pio, &peripherals.clocks);
    // // This will start a PWM with PIO with the set frequency and duty cycle on the specified pin.
    // pio_pwm
    //     .start(
    //         &RPGpio::GPIO7,
    //         pio_pwm.get_maximum_frequency_hz() / 125000, /*1_000*/
    //         pio_pwm.get_maximum_duty_cycle() / 2,
    //     )
    //     .unwrap();

    (board_kernel, raspberry_pi_pico, chip)
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    let (board_kernel, platform, chip) = start();
    board_kernel.kernel_loop(&platform, chip, Some(&platform.ipc), &main_loop_capability);
}
