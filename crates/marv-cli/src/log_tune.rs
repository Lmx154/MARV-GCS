//! `marv-cli log tune`: the post-flight tuning analysis of a DataFlash log of the MARV firmware.
//!
//! Its reader is a person or an AI agent who will propose one controller gain change per flight, so every number
//! carries its count, every threshold is a named constant printed in the header (`settings`), and what the log
//! cannot support is said so (never approximated from other records). It proposes no gain.
//!
//! The log is read once through `gcs_dflog::Reader` (`data`), analysed per armed span that was airborne
//! (`analysis`, `events`, `dsp`), and printed as text or as a JSON document (`render`).

use std::io::Write;

use crate::args::TuneCommand;
use crate::exit;

mod analysis;
mod data;
pub mod dsp;
pub mod events;
mod observe;
mod render;
pub mod settings;

pub use analysis::{TuneReport, analyze};
pub use settings::{DEFS, Settings};

pub fn run(command: &TuneCommand, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let data = match data::read(&command.file) {
        Ok(d) => d,
        Err(e) => {
            let _ = writeln!(
                err,
                "marv-cli: log tune of {} failed: {e}",
                command.file.display()
            );
            return exit::FAILED;
        }
    };
    let report = analyze(&data, &command.settings, command.from_s, command.to_s);
    let text = if command.json {
        let doc = render::to_json(&report, &command.settings);
        format!(
            "{}\n",
            serde_json::to_string_pretty(&doc).expect("a JSON value")
        )
    } else {
        render::to_text(&report, &command.settings)
    };
    if out.write_all(text.as_bytes()).is_err() {
        return exit::FAILED;
    }
    exit::OK
}
