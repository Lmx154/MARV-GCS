//! MARV ground station on the command line: one command per invocation on one link.
//!
//! Layer rule: like `marv-gcs` this is an application, so it may use both `gcs-transport` and
//! `gcs-mavlink` (through `gcs-link`); it never depends on a GUI crate. The gate is the session's
//! (`Session::send`); nothing here decides what a link may do.

use std::io::Write;

pub mod args;
pub mod commands;
pub mod log_report;
pub mod log_tune;
pub mod session;
pub mod waypoints;

/// The exit statuses.
pub mod exit {
    pub const OK: i32 = 0;
    /// The command failed, or the vehicle did not accept it.
    pub const FAILED: i32 = 1;
    pub const USAGE: i32 = 2;
    /// No link, no heartbeat, or the link went down.
    pub const NO_VEHICLE: i32 = 3;
    /// The link is operate-only and the command writes.
    pub const REFUSED: i32 = 4;
}

/// Runs the command line `args` (after the program name); returns the exit status.
pub fn run(args: &[String], out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let invocation = match args::parse(args) {
        Ok(args::Parsed::Help) => {
            let _ = out.write_all(args::USAGE.as_bytes());
            return exit::OK;
        }
        Ok(args::Parsed::Run(invocation)) => invocation,
        Ok(args::Parsed::Session(command)) => return session::run(&command, out, err),
        Ok(args::Parsed::Log(command)) => return log_report::run(&command, out, err),
        Ok(args::Parsed::LogTune(command)) => return log_tune::run(&command, out, err),
        Err(message) => {
            let _ = writeln!(err, "marv-cli: {message}\nTry: marv-cli --help");
            return exit::USAGE;
        }
    };
    let access = gcs_link::access_of(&invocation.endpoint);
    let session = match gcs_link::start_session(&invocation.endpoint, access, Box::new(|| {})) {
        Ok(session) => session,
        Err(error) => {
            let _ = writeln!(err, "marv-cli: {error}");
            return exit::NO_VEHICLE;
        }
    };
    commands::execute(&invocation, &session, access, out, err)
}
