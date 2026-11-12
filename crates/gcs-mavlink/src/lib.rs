//! MAVLink layer of the ground station.
//!
//! Layer rule: this crate speaks MAVLink over any `std::io::Read` / `std::io::Write` pair. It must
//! never depend on `gcs-transport`, `serialport` or a GUI crate. The application joins a transport
//! to this layer.

mod frame;
pub mod param_file;
pub mod services;
mod session;
mod state;
#[cfg(test)]
mod test_pipe;
pub mod tlog;

pub use frame::{Body, Deframer, Encoder, Frame, RxStats};
pub use mavlink::dialects::common as dialect;
pub use services::{Ctx, Service, default_services};
pub use session::{Session, SessionConfig};
pub use state::{Command, LinkState, LinkStatus, VehicleId, VehicleState};

pub type Message = dialect::MavMessage;
