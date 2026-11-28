#![no_std]
#![no_main]

//! Unified TOML-driven BareFOC firmware entry point.
//!
//! Hardware constants, memory layout, and capability `--cfg` flags are
//! generated at build time from `BAREFOC_BOARD` (`barefoc.toml`).

pub mod bsp {
    #![allow(dead_code, unused_macros, unused_imports)]
    include!(concat!(env!("OUT_DIR"), "/bsp_generated.rs"));
}

#[cfg(drv_drv8311)]
mod control_table;
mod hal;
#[cfg(drv_drv8311)]
mod nvm_config;
mod runners;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let _ = bsp::BOARD_NAME;
    runners::run(spawner).await;
}
