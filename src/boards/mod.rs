#[cfg(all(mcu_stm32h503cb, drv_drv8411a, sensor_tmag5273))]
mod fishyservo;

#[cfg(all(mcu_stm32g431cb, drv_drv8316, sensor_mt6701))]
mod microspora;

#[cfg(all(mcu_stm32h503cb, drv_drv8311, sensor_mt6701))]
mod miniata;

pub async fn run(spawner: embassy_executor::Spawner) {
    #[cfg(all(mcu_stm32h503cb, drv_drv8411a, sensor_tmag5273))]
    fishyservo::run(spawner).await;

    #[cfg(all(mcu_stm32g431cb, drv_drv8316, sensor_mt6701))]
    microspora::run(spawner).await;

    #[cfg(all(mcu_stm32h503cb, drv_drv8311, sensor_mt6701))]
    miniata::run(spawner).await;
}
