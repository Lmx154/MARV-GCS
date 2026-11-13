//! The UDP link: a socket bound to a local address, a peer learned from what arrives.
//!
//! Endpoint text: `udp:ADDR:PORT`, the LOCAL address to listen on (`udp:0.0.0.0:14550`). The radio
//! (for instance the WiFi backpack of an ExpressLRS transmitter) sends to that port.
//!
//! - **Peer.** It is the source address of the most recent datagram received, empty ones included.
//!   Before any datagram has arrived there is no peer: a `write` succeeds and the bytes are
//!   discarded. After that every `write` call is sent to the peer as one datagram. There is no
//!   authentication: whoever sent last is the peer.
//! - **Reader.** It presents a byte stream. A datagram larger than the caller's buffer is kept
//!   and returned over the next reads, in order, nothing lost (the receive buffer takes the largest
//!   possible datagram, 65 535 bytes). An empty datagram is skipped, never `Ok(0)`.
//! - **Errors.** Reads: `Interrupted` is retried; `WouldBlock` and `TimedOut` (the socket timeout
//!   expired) give `TimedOut`; `ConnectionRefused`, `ConnectionReset`, `NetworkUnreachable`,
//!   `HostUnreachable` and `NetworkDown` (a peer that went away, or a WiFi network that dropped)
//!   are "nothing yet": they are retried until [`READ_TIMEOUT`] has passed, then `TimedOut`; every
//!   other error is returned and means the link is down. Writes: `Interrupted` is retried; those
//!   same five kinds count as success (the bytes are discarded, as UDP allows); every other error
//!   is returned and means the link is down. A radio that is switched off and on again leaves the link usable with no
//!   reconnect.
//! - **Halves.** The socket is cloned for the writer; the peer is shared through a mutex.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use crate::{Endpoint, Link, OpenError, READ_TIMEOUT};

/// Room for the largest datagram UDP can carry.
const MAX_DATAGRAM: usize = 65_535;

type Peer = Arc<Mutex<Option<SocketAddr>>>;

fn lock(peer: &Peer) -> MutexGuard<'_, Option<SocketAddr>> {
    peer.lock().unwrap_or_else(|e| e.into_inner())
}

/// A peer that went away or a network that is down: not a reason to drop the link.
fn is_gone(kind: io::ErrorKind) -> bool {
    matches!(
        kind,
        io::ErrorKind::ConnectionRefused
            | io::ErrorKind::ConnectionReset
            | io::ErrorKind::NetworkUnreachable
            | io::ErrorKind::HostUnreachable
            | io::ErrorKind::NetworkDown
    )
}

pub(crate) fn open(endpoint: &Endpoint, bind: SocketAddr) -> Result<Link, OpenError> {
    let fail = |e: io::Error| OpenError::new(endpoint, e);
    let socket = UdpSocket::bind(bind).map_err(fail)?;
    let writer = socket.try_clone().map_err(fail)?;
    let peer: Peer = Arc::new(Mutex::new(None));
    Ok(Link {
        reader: Box::new(UdpReader {
            socket,
            peer: Arc::clone(&peer),
            buf: vec![0; MAX_DATAGRAM],
            start: 0,
            end: 0,
        }),
        writer: Box::new(UdpWriter {
            socket: writer,
            peer,
        }),
    })
}

struct UdpReader {
    socket: UdpSocket,
    peer: Peer,
    buf: Vec<u8>,
    /// The bytes of the last datagram that were not returned yet are `buf[start..end]`.
    start: usize,
    end: usize,
}

impl Read for UdpReader {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }
        let deadline = Instant::now() + READ_TIMEOUT;
        while self.start == self.end {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "read timed out"));
            }
            self.socket
                .set_read_timeout(Some(remaining.max(Duration::from_millis(1))))?;
            match self.socket.recv_from(&mut self.buf) {
                Ok((n, from)) => {
                    *lock(&self.peer) = Some(from);
                    self.start = 0;
                    self.end = n;
                }
                Err(e) => match e.kind() {
                    io::ErrorKind::Interrupted => {}
                    kind if is_gone(kind) => {}
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut => {
                        return Err(io::Error::new(io::ErrorKind::TimedOut, "read timed out"));
                    }
                    _ => return Err(e),
                },
            }
        }
        let n = out.len().min(self.end - self.start);
        out[..n].copy_from_slice(&self.buf[self.start..self.start + n]);
        self.start += n;
        Ok(n)
    }
}

struct UdpWriter {
    socket: UdpSocket,
    peer: Peer,
}

impl Write for UdpWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        if data.is_empty() {
            return Ok(0);
        }
        let Some(peer) = *lock(&self.peer) else {
            return Ok(data.len());
        };
        loop {
            match self.socket.send_to(data, peer) {
                Ok(_) => return Ok(data.len()),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) if is_gone(e.kind()) => return Ok(data.len()),
                Err(e) => return Err(e),
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gone_kinds_keep_the_link_and_other_kinds_do_not() {
        for kind in [
            io::ErrorKind::ConnectionRefused,
            io::ErrorKind::ConnectionReset,
            io::ErrorKind::NetworkUnreachable,
            io::ErrorKind::HostUnreachable,
            io::ErrorKind::NetworkDown,
        ] {
            assert!(is_gone(kind), "{kind:?}");
        }
        for kind in [
            io::ErrorKind::PermissionDenied,
            io::ErrorKind::InvalidInput,
            io::ErrorKind::BrokenPipe,
            io::ErrorKind::UnexpectedEof,
        ] {
            assert!(!is_gone(kind), "{kind:?}");
        }
    }
}
