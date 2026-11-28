//! Capability-driven runner dispatch.
//!
//! Runner selection is driven purely by hardware capability `--cfg` flags
//! emitted from `barefoc.toml` in `build.rs`, with zero per-board modules.

#[cfg(drv_drv8411a)]
mod dc_quad;

#[cfg(drv_drv8311)]
mod bldc_foc_3pwm;

#[cfg(drv_drv8316)]
mod bldc_foc_6pwm;

pub async fn run(spawner: embassy_executor::Spawner) {
    #[cfg(drv_drv8411a)]
    dc_quad::run(spawner).await;

    #[cfg(all(drv_drv8311, not(drv_drv8411a)))]
    bldc_foc_3pwm::run(spawner).await;

    #[cfg(all(drv_drv8316, not(drv_drv8411a)))]
    bldc_foc_6pwm::run(spawner).await;
}
