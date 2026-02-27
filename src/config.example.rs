pub const LOCAL_IP_ADDRESS: [u8; 4] = [10, 0, 2, 15];
pub const LOCAL_IP_PREFIXLEN: u8 = 24;

// https://github.com/obdev/v-usb/blob/7a28fdc685952412dad2b8842429127bc1cf9fa7/usbdrv/USB-IDs-for-free.txt
pub const USB_VID: u16 = 0x16c0;
pub const USB_PID: u16 = 0x05df;
pub const USB_PRODUCT: Option<&str> = Some("rpc7850");
pub const USB_MANUFACTURER: Option<&str> = Some("rinsuki-lab");
