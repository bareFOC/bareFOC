#![no_std]
#![no_main]

//! Unified TOML-driven BareFOC firmware entry point.
//!
//! Hardware constants, memory layout, and board selection `--cfg` flags are
//! generated at build time from `BAREFOC_BOARD` (`barefoc.toml`).

pub mod bsp {
    #![allow(dead_code, unused_macros, unused_imports)]
    include!(concat!(env!("OUT_DIR"), "/bsp_generated.rs"));
}

mod boards;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let _ = bsp::BOARD_NAME;
    boards::run(spawner).await;
}
