#![no_std]
#![no_main]

mod config; // please make your one by copy config.example.rs and modify it
mod hid;

use core::sync::atomic::AtomicU8;

use embassy_rp::gpio;
use embassy_usb::control;
use static_cell::StaticCell;
use usbd_hid::descriptor::SerializedDescriptor as _;

embassy_rp::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

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

static HID_PROTOCOL_MODE: AtomicU8 = AtomicU8::new(usbd_hid::hid_class::HidProtocolMode::Boot as u8);

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

    let driver = embassy_rp::usb::Driver::new(rp.USB, Irqs);
    // https://github.com/obdev/v-usb/blob/7a28fdc685952412dad2b8842429127bc1cf9fa7/usbdrv/USB-IDs-for-free.txt
    let mut config = embassy_usb::Config::new(0x16c0, 0x05df);
    // https://www.usb.org/defined-class-codes#anchor_BaseClass00h
    config.composite_with_iads = false;
    config.device_class = 0;
    config.device_sub_class = 0;
    config.device_protocol = 0;

    let mut hid_state_kb = embassy_usb::class::hid::State::new();
    let mut hid_state_gc = embassy_usb::class::hid::State::new();
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut keyboard_handler = KeyboardRequestHandler {};
    let mut builder = embassy_usb::Builder::new(driver, config, &mut config_descriptor, &mut bos_descriptor, &mut msos_descriptor, &mut control_buf);
    let config = embassy_usb::class::hid::Config {
        report_descriptor: hid::controller::CONTROLLER_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 17,
        max_packet_size: 64,
        hid_boot_protocol: embassy_usb::class::hid::HidBootProtocol::None,
        hid_subclass: embassy_usb::class::hid::HidSubclass::No,
    };
    let hid_gc = embassy_usb::class::hid::HidReaderWriter::<_, 2, 8>::new(&mut builder, &mut hid_state_gc, config);
    let config = embassy_usb::class::hid::Config {
        report_descriptor: usbd_hid::descriptor::KeyboardReport::desc(),
        request_handler: Some(&mut keyboard_handler),
        poll_ms: 17,
        max_packet_size: 8,
        hid_boot_protocol: embassy_usb::class::hid::HidBootProtocol::Keyboard,
        hid_subclass: embassy_usb::class::hid::HidSubclass::Boot,
    };
    let hid_kb = embassy_usb::class::hid::HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut hid_state_kb, config);
    let mut usb = builder.build();
    let usb_fut = usb.run();

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let (mut hid_kb_reader, mut hid_kb_writer) = hid_kb.split();
    let (_hid_gc_reader, mut hid_gc_writer) = hid_gc.split();

    let tcp_fut = async {
        loop {
            let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
            socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
            led.set_low();
            if let Err(e) = socket.accept(5151).await {
                continue;
            }
            led.set_high();

            read_loop(socket, &mut hid_kb_writer, &mut hid_gc_writer).await;
        }
    };

    let mut out_report_handler = OutReportHandler {};
    let out_fut = async {
        hid_kb_reader.run(false, &mut out_report_handler).await;
    };

    embassy_futures::join::join3(usb_fut, tcp_fut, out_fut).await;
}

async fn read_loop<'d, D: embassy_usb::driver::Driver<'d>>(
    mut socket: embassy_net::tcp::TcpSocket<'_>,
    hid_kb: &mut embassy_usb::class::hid::HidWriter<'d, D, 8>,
    hid_gc: &mut embassy_usb::class::hid::HidWriter<'d, D, 8>
) {
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
        if buf == *b"write_some_texts" {
            let mut buf: [u8; 1] = [0; 1];
            loop {
                match socket.read(&mut buf).await {
                    Ok(0) => break, // EOF
                    Ok(_) => {
                        let charcode = match buf[0] {
                            b'a'..=b'z' => buf[0] - b'a' + 0x04,
                            b'\n' => 0x28,
                            _ => 0,
                        };
                        hid_kb.write(&[0, 0, charcode, 0, 0, 0, 0, 0]).await.unwrap();
                        hid_kb.write(&[0; 8]).await.unwrap();
                    },
                    Err(_) => break, // error
                }
            }
        } else if buf[0..8] == *b"rawhidkb" {
            hid_kb.write(&buf[8..16]).await.unwrap();
            socket.write(b"ack\x00").await.unwrap();
        } else if buf[0..8] == *b"rawhidgc" {
            hid_gc.write(&buf[8..16]).await.unwrap();
            socket.write(b"ack\x00").await.unwrap();
        } else if buf == *b"reset_to_usbboot" {
            embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            break;
        }
    }
    socket.abort();
    socket.flush().await.unwrap();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    loop {}
}

struct OutReportHandler {}
impl embassy_usb::class::hid::RequestHandler for OutReportHandler {}

struct KeyboardRequestHandler {}
impl embassy_usb::class::hid::RequestHandler for KeyboardRequestHandler {
    fn get_protocol(&self) -> embassy_usb::class::hid::HidProtocolMode {
        match HID_PROTOCOL_MODE.load(core::sync::atomic::Ordering::SeqCst) {
            x if x == usbd_hid::hid_class::HidProtocolMode::Report as u8 => embassy_usb::class::hid::HidProtocolMode::Report,
            _ => embassy_usb::class::hid::HidProtocolMode::Boot,
        }
    }

    fn set_protocol(&mut self, protocol: embassy_usb::class::hid::HidProtocolMode) -> control::OutResponse {
        HID_PROTOCOL_MODE.store(protocol as u8, core::sync::atomic::Ordering::SeqCst);
        control::OutResponse::Accepted
    }
}