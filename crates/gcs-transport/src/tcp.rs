use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::{Endpoint, Link, OpenError, READ_TIMEOUT, Reader};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);

pub(crate) fn open(endpoint: &Endpoint, addr: &str) -> Result<Link, OpenError> {
    let fail = |e: std::io::Error| OpenError::new(endpoint, e);
    let mut last = None;
    let mut stream = None;
    for sa in addr.to_socket_addrs().map_err(fail)? {
        match TcpStream::connect_timeout(&sa, CONNECT_TIMEOUT) {
            Ok(s) => {
                stream = Some(s);
                break;
            }
            Err(e) => last = Some(e),
        }
    }
    let stream = match stream {
        Some(s) => s,
        None => {
            return Err(fail(last.unwrap_or_else(|| {
                std::io::Error::other("address resolved to nothing")
            })));
        }
    };
    stream.set_read_timeout(Some(READ_TIMEOUT)).map_err(fail)?;
    stream.set_nodelay(true).map_err(fail)?;
    let writer = stream.try_clone().map_err(fail)?;
    Ok(Link {
        reader: Box::new(Reader(stream)),
        writer: Box::new(writer),
    })
}
