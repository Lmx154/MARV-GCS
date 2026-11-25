//! Joins a transport to the MAVLink layer, for every application (the window and the command line).
//!
//! Layer rule: this crate and the applications are the only ones that depend on both `gcs-transport`
//! and `gcs-mavlink`. It must never depend on a GUI crate. It holds the one mapping from an endpoint
//! to a role (`access_of`) and the one way to start a session on an endpoint (`start_session`); the
//! transport knows only bytes and `gcs-mavlink` knows no transport.

pub mod session_folder;

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

    #[test]
    fn the_mission_commands_are_allowed_on_every_link_and_a_configure_command_is_not_on_udp() {
        use gcs_mavlink::Command;
        let udp: Endpoint = "udp:0.0.0.0:14550".parse().expect("endpoint");
        let operate = access_of(&udp);
        assert_eq!(operate, Access::Operate);
        for command in [
            Command::UploadMission {
                items: Vec::new(),
                return_height: 20.0,
            },
            Command::DownloadMission,
            Command::ClearMission,
        ] {
            assert!(operate.allows(command.access()), "{}", command.name());
        }
        assert!(!operate.allows(Command::CancelLogDownload.access()));
    }
}
