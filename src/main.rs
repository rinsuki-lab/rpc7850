#![no_std]
#![no_main]

mod config; // please make your one by copy config.example.rs and modify it

use embassy_rp::gpio;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn ethernet_task(
    runner: embassy_net_wiznet::Runner<
        'static,
        embassy_net_wiznet::chip::W5500,
        embedded_hal_bus::spi::ExclusiveDevice<
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
            gpio::Output<'static>,
            embassy_time::Delay,
        >,
        gpio::Input<'static>,
        gpio::Output<'static>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<'static, embassy_net_wiznet::Device<'static>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let rp = embassy_rp::init(embassy_rp::config::Config::new(
        embassy_rp::clocks::ClockConfig::system_freq(150_000_000).unwrap(),
    ));
    let mut led = gpio::Output::new(rp.PIN_25, gpio::Level::Low);
    led.set_high();
    let mut rng = embassy_rp::clocks::RoscRng;

    let mut spi_cfg = embassy_rp::spi::Config::default();
    spi_cfg.frequency = 72_000_000;
    let (miso, mosi, clk) = (rp.PIN_16, rp.PIN_19, rp.PIN_18);
    let spi = embassy_rp::spi::Spi::new(rp.SPI0, clk, mosi, miso, rp.DMA_CH0, rp.DMA_CH1, spi_cfg);
    let cs = gpio::Output::new(rp.PIN_17, gpio::Level::High);
    let w5500_int = gpio::Input::new(rp.PIN_21, gpio::Pull::Up);
    let w5500_reset = gpio::Output::new(rp.PIN_20, gpio::Level::High);

    static STATE: StaticCell<embassy_net_wiznet::State<8, 8>> = StaticCell::new();
    let state = STATE.init(embassy_net_wiznet::State::new());

    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01]; // TODO
    let (device, runner) = embassy_net_wiznet::new(
        mac_addr,
        state,
        embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, embassy_time::Delay),
        w5500_int,
        w5500_reset,
    )
    .await
    .unwrap();
    spawner.spawn(ethernet_task(runner)).unwrap();

    static RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(
                embassy_net::Ipv4Address::new(
                    config::LOCAL_IP_ADDRESS[0],
                    config::LOCAL_IP_ADDRESS[1],
                    config::LOCAL_IP_ADDRESS[2],
                    config::LOCAL_IP_ADDRESS[3],
                ),
                config::LOCAL_IP_PREFIXLEN,
            ),
            gateway: None,
            dns_servers: heapless::Vec::new(),
        }),
        RESOURCES.init(embassy_net::StackResources::new()),
        rng.next_u64(),
    );
    spawner.spawn(net_task(runner)).unwrap();

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        led.set_low();
        if let Err(e) = socket.accept(5151).await {
            continue;
        }
        led.set_high();

        read_loop(socket).await;
    }
}

async fn read_loop(mut socket: embassy_net::tcp::TcpSocket<'_>) {
    let mut buf: [u8; 16] = [0; 16];
    loop {
        let mut i = 0;
        while i < buf.len() {
            match socket.read(&mut buf[i..]).await {
                Ok(0) => return, // EOF
                Ok(n) => i += n,
                Err(_) => return, // error
            }
        }
        if buf == *b"reset_to_usbboot" {
            embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            break;
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    loop {}
}
