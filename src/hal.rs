//! Shared hardware abstraction helpers for TOML-driven BareFOC runners.

#![allow(dead_code)]

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{Operation, SpiBus, SpiDevice};

#[cfg(mcu_stm32g431cb)]
use panic_halt as _;

#[cfg(mcu_stm32h503cb)]
#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    barefoc_stm32::bootloader::barefoc_failsafe_recovery();
}

/// Any hard fault (including a debugger-induced DEBUGEVT) trips the failsafe
/// recovery trampoline: PWM disabled, DRV_SLEEP low, mailbox token written,
/// and system resets into the bootloader.
#[cfg(mcu_stm32h503cb)]
#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    barefoc_stm32::bootloader::barefoc_failsafe_recovery();
}

/// Independent watchdog base address on STM32H5.
#[cfg(mcu_stm32h503cb)]
const IWDG_BASE: usize = 0x4000_3000;

/// Initialize the independent watchdog (~8 s timeout at 32 kHz LSI).
#[cfg(mcu_stm32h503cb)]
pub fn iwdg_init() {
    unsafe {
        let kr = IWDG_BASE as *mut u32;
        let pr = (IWDG_BASE + 0x04) as *mut u32;
        let rlr = (IWDG_BASE + 0x08) as *mut u32;
        core::ptr::write_volatile(kr, 0x5555); // unlock
        core::ptr::write_volatile(pr, 0x04); // /64
        core::ptr::write_volatile(rlr, 0x0FFF); // ~8 s at 32 kHz LSI
        core::ptr::write_volatile(kr, 0xAAAA); // initial reload
        core::ptr::write_volatile(kr, 0xCCCC); // start
    }
}

/// Feed the independent watchdog.
#[cfg(mcu_stm32h503cb)]
pub fn iwdg_feed() {
    unsafe {
        core::ptr::write_volatile(IWDG_BASE as *mut u32, 0xAAAA);
    }
}

/// Configure STM32G431 clocks (HSI 16 MHz / 4 * 85 / 2 = 170 MHz SYSCLK).
#[cfg(mcu_stm32g431cb)]
pub fn configure_g4_clocks() -> embassy_stm32::Config {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource,
        Sysclk,
    };

    let mut cfg = embassy_stm32::Config::default();
    cfg.rcc.hsi = true;
    cfg.rcc.pll = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div4,
        mul: PllMul::Mul85,
        divp: Some(PllPDiv::Div2),
        divq: Some(PllQDiv::Div2),
        divr: Some(PllRDiv::Div2),
    });
    cfg.rcc.sys = Sysclk::Pll1R;
    cfg.rcc.ahb_pre = AHBPrescaler::Div1;
    cfg.rcc.apb1_pre = APBPrescaler::Div1;
    cfg.rcc.apb2_pre = APBPrescaler::Div1;
    cfg
}

/// Exclusive SPI device wrapper asserting chip-select around each transaction.
pub struct ExclusiveSpiDevice<BUS, CS> {
    bus: BUS,
    cs: CS,
}

impl<BUS: SpiBus, CS: OutputPin> ExclusiveSpiDevice<BUS, CS> {
    pub fn new(bus: BUS, mut cs: CS) -> Self {
        let _ = cs.set_high();
        Self { bus, cs }
    }
}

impl<BUS: SpiBus, CS: OutputPin> embedded_hal::spi::ErrorType for ExclusiveSpiDevice<BUS, CS> {
    type Error = BUS::Error;
}

impl<BUS: SpiBus, CS: OutputPin> SpiDevice for ExclusiveSpiDevice<BUS, CS> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let sys_clk = crate::bsp::SYSTEM_CLOCK_HZ;
        let _ = self.cs.set_low();
        cortex_m::asm::delay(sys_clk / 1_000_000); // ~1 us CS setup

        let res = (|| {
            for op in operations {
                match op {
                    Operation::Read(buf) => self.bus.read(buf)?,
                    Operation::Write(buf) => self.bus.write(buf)?,
                    Operation::Transfer(r, w) => self.bus.transfer(r, w)?,
                    Operation::TransferInPlace(buf) => self.bus.transfer_in_place(buf)?,
                    Operation::DelayNs(ns) => {
                        let cycles = (*ns as u64 * sys_clk as u64 / 1_000_000_000) as u32;
                        cortex_m::asm::delay(cycles.max(1));
                    }
                }
            }
            Ok(())
        })();

        cortex_m::asm::delay(sys_clk / 1_000_000); // ~1 us CS hold
        let _ = self.cs.set_high();
        cortex_m::asm::delay(sys_clk / 1_000_000); // ~1 us bus release
        res
    }
}

/// Shared I2C device wrapper borrowing a `RefCell<BUS>` for multi-sensor I2C buses.
pub struct SharedI2cDevice<'a, BUS> {
    bus: &'a core::cell::RefCell<BUS>,
}

impl<'a, BUS> SharedI2cDevice<'a, BUS> {
    pub fn new(bus: &'a core::cell::RefCell<BUS>) -> Self {
        Self { bus }
    }
}

impl<'a, BUS: embedded_hal::i2c::ErrorType> embedded_hal::i2c::ErrorType for SharedI2cDevice<'a, BUS> {
    type Error = BUS::Error;
}

impl<'a, BUS: embedded_hal::i2c::I2c> embedded_hal::i2c::I2c for SharedI2cDevice<'a, BUS> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.bus.borrow_mut().transaction(address, operations)
    }
}
