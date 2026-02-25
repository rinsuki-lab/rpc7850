#![no_std]
#![no_main]

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let rp = embassy_rp::init(embassy_rp::config::Config::new(embassy_rp::clocks::ClockConfig::crystal(12_000_000)));
    let mut led = embassy_rp::gpio::Output::new(rp.PIN_25, embassy_rp::gpio::Level::Low);
    loop {
        led.toggle();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}