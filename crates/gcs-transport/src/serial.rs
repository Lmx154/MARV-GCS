use serialport::{DataBits, FlowControl, Parity, SerialPortType, StopBits};

use crate::{Endpoint, Link, OpenError, READ_TIMEOUT, Reader};

/// A serial port found on this machine.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    pub path: String,
    pub usb: Option<UsbInfo>,
}

/// USB identity of a serial port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbInfo {
    pub vid: u16,
    pub pid: u16,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    pub serial_number: Option<String>,
}

pub(crate) fn open(endpoint: &Endpoint, path: &str, baud: u32) -> Result<Link, OpenError> {
    let fail = |e: serialport::Error| OpenError::new(endpoint, e);
    let mut port = serialport::new(path, baud)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(fail)?;
    // Some USB devices only talk once DTR is up; a pty or a port without the line cannot do it.
    let _ = port.write_data_terminal_ready(true);
    let writer = port.try_clone().map_err(fail)?;
    Ok(Link {
        reader: Box::new(Reader(port)),
        writer,
    })
}

/// Every serial port the system reports; USB ports carry their identity. An enumeration failure
/// gives an empty list.
pub fn list_serial_ports() -> Vec<PortInfo> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    ports
        .into_iter()
        .map(|p| PortInfo {
            path: p.port_name,
            usb: match p.port_type {
                SerialPortType::UsbPort(u) => Some(UsbInfo {
                    vid: u.vid,
                    pid: u.pid,
                    manufacturer: u.manufacturer,
                    product: u.product,
                    serial_number: u.serial_number,
                }),
                _ => None,
            },
        })
        .collect()
}
