//! `marv-sim`: the simulated vehicle on a TCP port.

use std::net::TcpListener;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use gcs_sim::{SimConfig, SimVehicle, serve_tcp};

const DEFAULT_ADDR: &str = "127.0.0.1:5760";

const USAGE: &str = "\
marv-sim: a simulated MAVLink vehicle

Usage: marv-sim [--tcp <addr>] [--drop-mission-rx <n>] [--drop-mission-tx <n>]

Options:
  --tcp <addr>            address to listen on (default 127.0.0.1:5760)
  --drop-mission-rx <n>   lose every n-th mission frame addressed to the vehicle (default 0: none)
  --drop-mission-tx <n>   lose every n-th mission frame the vehicle sends (default 0: none)
  --help                  show this help
";

fn main() -> ExitCode {
    let mut addr = DEFAULT_ADDR.to_string();
    let mut config = SimConfig::default().with_device_status(1.0);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            "--tcp" => match args.next() {
                Some(value) => addr = value,
                None => return usage_error("--tcp needs an address"),
            },
            "--drop-mission-rx" | "--drop-mission-tx" => {
                let Some(n) = args.next().and_then(|v| v.parse::<u32>().ok()) else {
                    return usage_error(&format!("{arg} needs a number"));
                };
                if arg == "--drop-mission-rx" {
                    config.drop_mission_rx_every = n;
                } else {
                    config.drop_mission_tx_every = n;
                }
            }
            other => return usage_error(&format!("unknown argument: {other}")),
        }
    }

    let listener = match TcpListener::bind(&addr) {
        Ok(listener) => listener,
        Err(e) => {
            eprintln!("marv-sim: cannot listen on {addr}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let bound = listener.local_addr().map_or(addr, |a| a.to_string());
    println!("marv-sim listening on {bound}");

    let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
    serve_tcp(listener, vehicle, Arc::new(AtomicBool::new(false)));
    ExitCode::SUCCESS
}

fn usage_error(message: &str) -> ExitCode {
    eprintln!("marv-sim: {message}\n\n{USAGE}");
    ExitCode::from(2)
}
