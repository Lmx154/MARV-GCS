//! Byte transports for the ground station.
//!
//! Layer rule: this crate moves bytes and nothing else. It must never depend on `mavlink` or on
//! `gcs-mavlink`. The MAVLink layer sits on top of it and takes any `Read` / `Write` pair.
//!
//! Contract of every [`Link`] reader:
//! - `read` returns as soon as at least one byte is available;
//! - with no data for [`READ_TIMEOUT`] it returns an error of kind `TimedOut` (never `WouldBlock`)
//!   and loses no bytes;
//! - `Ok(0)` or any other error means the link is down for good;
//! - the two halves may be used from two threads at once; dropping both closes the link.

mod endpoint;
mod serial;
mod tcp;
mod udp;

use std::io;
use std::time::Duration;

pub use endpoint::{DEFAULT_BAUD, Endpoint, ParseEndpointError};
pub use serial::{PortInfo, UsbInfo, list_serial_ports};

/// How long a `read` waits for data before it returns a `TimedOut` error.
pub const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// An open byte link: the two halves can be moved to different threads.
pub struct Link {
    pub reader: Box<dyn io::Read + Send>,
    pub writer: Box<dyn io::Write + Send>,
}

/// Why an endpoint could not be opened.
#[derive(Debug)]
pub struct OpenError {
    endpoint: String,
    source: Box<dyn std::error::Error + Send + Sync>,
}

impl OpenError {
    pub(crate) fn new(
        endpoint: &Endpoint,
        source: impl Into<Box<dyn std::error::Error + Send + Sync>>,
    ) -> Self {
        OpenError {
            endpoint: endpoint.to_string(),
            source: source.into(),
        }
    }

    /// The endpoint that failed, in its `Display` form.
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }
}

impl std::fmt::Display for OpenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "cannot open {}: {}", self.endpoint, self.source)
    }
}

impl std::error::Error for OpenError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.source.as_ref())
    }
}

/// Open the endpoint and return its reader and writer halves.
pub fn open(endpoint: &Endpoint) -> Result<Link, OpenError> {
    match endpoint {
        Endpoint::Serial { path, baud } => serial::open(endpoint, path, *baud),
        Endpoint::Tcp { addr } => tcp::open(endpoint, addr),
        Endpoint::Udp { bind } => udp::open(endpoint, *bind),
    }
}

/// Wraps a reader so that it follows the contract: `WouldBlock` becomes `TimedOut` and an
/// interrupted system call is retried.
pub(crate) struct Reader<R>(pub(crate) R);

impl<R: io::Read> io::Read for Reader<R> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        loop {
            match self.0.read(buf) {
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "read timed out"));
                }
                other => return other,
            }
        }
    }
}
