//! In-memory byte pipe for tests. The reader follows the transport contract: `read` returns as soon
//! as there is data, `TimedOut` after 100 ms without data, `Ok(0)` once the writer is gone.

use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::time::Duration;

pub const READ_TIMEOUT: Duration = Duration::from_millis(100);

pub struct PipeWriter {
    tx: Sender<Vec<u8>>,
}

pub struct PipeReader {
    rx: Receiver<Vec<u8>>,
    pending: Vec<u8>,
}

pub fn pipe() -> (PipeWriter, PipeReader) {
    let (tx, rx) = mpsc::channel();
    (
        PipeWriter { tx },
        PipeReader {
            rx,
            pending: Vec::new(),
        },
    )
}

impl Write for PipeWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.tx
            .send(buf.to_vec())
            .map_err(|_| io::Error::new(io::ErrorKind::BrokenPipe, "pipe reader dropped"))?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Read for PipeReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.pending.is_empty() {
            match self.rx.recv_timeout(READ_TIMEOUT) {
                Ok(chunk) => self.pending = chunk,
                Err(RecvTimeoutError::Timeout) => {
                    return Err(io::ErrorKind::TimedOut.into());
                }
                Err(RecvTimeoutError::Disconnected) => return Ok(0),
            }
        }
        let n = buf.len().min(self.pending.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }
}

/// Wraps a reader or writer and sets a flag when it is dropped.
pub struct DropFlag<T> {
    inner: T,
    flag: Arc<AtomicBool>,
}

impl<T> DropFlag<T> {
    pub fn new(inner: T) -> (Self, Arc<AtomicBool>) {
        let flag = Arc::new(AtomicBool::new(false));
        (
            Self {
                inner,
                flag: Arc::clone(&flag),
            },
            flag,
        )
    }
}

impl<T> Drop for DropFlag<T> {
    fn drop(&mut self) {
        self.flag.store(true, Ordering::SeqCst);
    }
}

impl<T: Read> Read for DropFlag<T> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.inner.read(buf)
    }
}

impl<T: Write> Write for DropFlag<T> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.inner.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}
