use std::fmt;
use std::str::FromStr;

/// Baud rate used when a serial endpoint string does not give one.
pub const DEFAULT_BAUD: u32 = 115_200;

/// Where to connect. Text form: `tcp:127.0.0.1:5760`, `serial:/dev/ttyACM0:115200`,
/// `serial:COM3` (baud defaults to 115200).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Endpoint {
    Serial { path: String, baud: u32 },
    Tcp { addr: String },
}

/// A string that is not a valid [`Endpoint`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseEndpointError(String);

impl fmt::Display for ParseEndpointError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ParseEndpointError {}

impl fmt::Display for Endpoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Endpoint::Serial { path, baud } => write!(f, "serial:{path}:{baud}"),
            Endpoint::Tcp { addr } => write!(f, "tcp:{addr}"),
        }
    }
}

impl FromStr for Endpoint {
    type Err = ParseEndpointError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let err = |m: &str| ParseEndpointError(format!("invalid endpoint {s:?}: {m}"));
        if let Some(addr) = s.strip_prefix("tcp:") {
            if addr.is_empty() {
                return Err(err("missing address, expected tcp:HOST:PORT"));
            }
            return Ok(Endpoint::Tcp {
                addr: addr.to_string(),
            });
        }
        if let Some(rest) = s.strip_prefix("serial:") {
            let (path, baud) = match rest.rsplit_once(':') {
                Some((path, last))
                    if !last.is_empty() && last.bytes().all(|b| b.is_ascii_digit()) =>
                {
                    let baud: u32 = last.parse().map_err(|_| err("baud rate out of range"))?;
                    if baud == 0 {
                        return Err(err("baud rate must not be 0"));
                    }
                    (path, baud)
                }
                Some((_, "")) => return Err(err("empty baud rate after the last colon")),
                _ => (rest, DEFAULT_BAUD),
            };
            if path.is_empty() {
                return Err(err("missing port path, expected serial:PATH[:BAUD]"));
            }
            return Ok(Endpoint::Serial {
                path: path.to_string(),
                baud,
            });
        }
        Err(err("expected tcp:HOST:PORT or serial:PATH[:BAUD]"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn serial(path: &str, baud: u32) -> Endpoint {
        Endpoint::Serial {
            path: path.into(),
            baud,
        }
    }

    #[test]
    fn parses_the_documented_forms() {
        assert_eq!(
            "tcp:127.0.0.1:5760".parse(),
            Ok(Endpoint::Tcp {
                addr: "127.0.0.1:5760".into()
            })
        );
        assert_eq!(
            "serial:/dev/ttyACM0:115200".parse(),
            Ok(serial("/dev/ttyACM0", 115200))
        );
        assert_eq!(
            "serial:/dev/ttyACM0".parse(),
            Ok(serial("/dev/ttyACM0", 115200))
        );
        assert_eq!("serial:COM3:57600".parse(), Ok(serial("COM3", 57600)));
        assert_eq!("serial:COM3".parse(), Ok(serial("COM3", 115200)));
    }

    #[test]
    fn colons_inside_the_path() {
        let p = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0";
        assert_eq!(format!("serial:{p}").parse(), Ok(serial(p, 115200)));
        assert_eq!(format!("serial:{p}:57600").parse(), Ok(serial(p, 57600)));
    }

    #[test]
    fn round_trip_is_exact() {
        for e in [
            serial("/dev/ttyACM0", 115200),
            serial("COM3", 57600),
            serial("/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0", 921600),
            serial("odd:57600", 115200),
            serial("trailing:", 9600),
            Endpoint::Tcp {
                addr: "127.0.0.1:5760".into(),
            },
            Endpoint::Tcp {
                addr: "[::1]:5760".into(),
            },
        ] {
            assert_eq!(e.to_string().parse::<Endpoint>(), Ok(e.clone()), "{e}");
        }
    }

    #[test]
    fn malformed_strings_are_errors() {
        for s in [
            "",
            "tcp",
            "tcp:",
            "serial:",
            "serial::115200",
            "serial:/dev/x:",
            "serial:/dev/x:0",
            "serial:/dev/x:99999999999",
            "udp:127.0.0.1:1",
            "TCP:127.0.0.1:1",
            "/dev/ttyACM0",
            "\u{0}",
        ] {
            let r = s.parse::<Endpoint>();
            assert!(r.is_err(), "{s:?} parsed as {r:?}");
        }
    }
}
