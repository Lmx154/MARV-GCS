//! TCP server around [`SimVehicle`]: real time, one client at a time.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use crate::SimVehicle;

/// Time between steps of the vehicle, and the longest wait for a client's bytes.
const TICK: Duration = Duration::from_millis(5);
const WRITE_TIMEOUT: Duration = Duration::from_millis(100);

/// Runs `vehicle` on `listener` until `stop` is set (checked every few milliseconds).
///
/// `now_ms` is the time since this call started. The vehicle keeps stepping, and keeps its state,
/// while no client is connected; what it sends then is dropped. A client that disconnects or
/// stops reading is dropped and the next one accepted.
pub fn serve_tcp(listener: TcpListener, vehicle: Arc<Mutex<SimVehicle>>, stop: Arc<AtomicBool>) {
    if listener.set_nonblocking(true).is_err() {
        return;
    }
    let start = Instant::now();
    let mut client: Option<TcpStream> = None;
    let mut buf = [0u8; 1024];
    while !stop.load(Ordering::Relaxed) {
        let mut received: &[u8] = &[];
        let mut closed = false;
        if let Some(stream) = client.as_mut() {
            match stream.read(&mut buf) {
                Ok(0) => closed = true,
                Ok(n) => received = &buf[..n],
                Err(e) if is_timeout(&e) => {}
                Err(_) => closed = true,
            }
        } else {
            match listener.accept() {
                Ok((stream, _)) => client = configure(stream),
                Err(_) => std::thread::sleep(TICK),
            }
        }

        let now_ms = start.elapsed().as_millis() as u64;
        let out = lock(&vehicle).step(now_ms, received);

        if closed {
            client = None;
        } else if let Some(stream) = client.as_mut()
            && !out.is_empty()
            && stream.write_all(&out).is_err()
        {
            client = None;
        }
    }
}

fn configure(stream: TcpStream) -> Option<TcpStream> {
    stream.set_nonblocking(false).ok()?;
    stream.set_nodelay(true).ok()?;
    stream.set_read_timeout(Some(TICK)).ok()?;
    stream.set_write_timeout(Some(WRITE_TIMEOUT)).ok()?;
    Some(stream)
}

fn is_timeout(e: &std::io::Error) -> bool {
    use std::io::ErrorKind::{Interrupted, TimedOut, WouldBlock};
    matches!(e.kind(), WouldBlock | TimedOut | Interrupted)
}

fn lock(vehicle: &Mutex<SimVehicle>) -> MutexGuard<'_, SimVehicle> {
    vehicle.lock().unwrap_or_else(PoisonError::into_inner)
}
