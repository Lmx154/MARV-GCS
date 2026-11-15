//! Joins a transport to the MAVLink layer, for every application (the window and the command line).
//!
//! Layer rule: this crate and the applications are the only ones that depend on both `gcs-transport`
//! and `gcs-mavlink`. It must never depend on a GUI crate. It holds the one mapping from an endpoint
//! to a role (`access_of`) and the one way to start a session on an endpoint (`start_session`); the
//! transport knows only bytes and `gcs-mavlink` knows no transport.

use gcs_mavlink::{Access, Session, SessionConfig};
use gcs_transport::{Endpoint, OpenError};

/// The role a link gives the station (docs/link-contract.md, "Access"). The one place that maps a
/// link to an `Access`, exhaustive over the endpoints: a new link kind does not compile until it has a
/// role.
pub fn access_of(endpoint: &Endpoint) -> Access {
    match endpoint {
        Endpoint::Serial { .. } => Access::Configure,
        Endpoint::Tcp { .. } => Access::Configure,
        Endpoint::Udp { .. } => Access::Operate,
    }
}

/// Open the endpoint and start a session on it with `access`; `wake` is called when the state changed.
pub fn start_session(
    endpoint: &Endpoint,
    access: Access,
    wake: Box<dyn Fn() + Send + Sync>,
) -> Result<Session, OpenError> {
    let link = gcs_transport::open(endpoint)?;
    Ok(Session::start(
        link.reader,
        link.writer,
        SessionConfig {
            access,
            ..SessionConfig::default()
        },
        wake,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_endpoint_has_its_role() {
        let cases = [
            ("serial:/dev/ttyACM0:115200", Access::Configure),
            ("tcp:127.0.0.1:5760", Access::Configure),
            ("udp:0.0.0.0:14550", Access::Operate),
        ];
        for (text, access) in cases {
            let endpoint: Endpoint = text.parse().expect("endpoint");
            assert_eq!(access_of(&endpoint), access, "{text}");
        }
    }
}
