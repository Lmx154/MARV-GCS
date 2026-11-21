//! Simulated MAVLink vehicle.
//!
//! Independence rule: this crate is written from the MAVLink specification and uses the `mavlink`
//! crate directly. It must never depend on `gcs-mavlink`, so that the ground station and the
//! simulator check each other instead of sharing one reading of the protocol.
//!
//! [`SimVehicle`] is sans-IO: virtual time and bytes go in, bytes come out. [`serve_tcp`] puts it
//! on a TCP socket in real time. Sensor values are pure functions of time in [`signals`].
//!
//! Behaviour follows section 2 of `docs/link-contract.md`. MAV_CMD_SET_MESSAGE_INTERVAL (511) is
//! not implemented in the simulator yet: it gets COMMAND_ACK UNSUPPORTED like every other command
//! except MAV_CMD_REQUEST_MESSAGE (512).

pub mod actions;
pub mod devices;
pub mod mag;
pub mod presets;
pub mod rcmap;
pub mod script;
pub mod signals;

mod fwparams;
mod logs;
mod params;
mod server;
mod vehicle;

pub use logs::{LogDataDrop, SimLog, default_logs, log_bytes};
pub use params::{ParamEncoding, ParamValue, SimParam, default_params};
pub use server::serve_tcp;
pub use vehicle::{SimConfig, SimVehicle, Stream};
