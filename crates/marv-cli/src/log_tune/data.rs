//! The log as `log tune` uses it: one pass through `gcs_dflog::Reader`, keeping only the records the analysis needs.
//! No position record (GPS, GPV, NKF1, ...) is read.

use std::collections::BTreeMap;
use std::path::Path;

use gcs_dflog::{Reader, Value};

/// The 14 gain parameters of decision-gain-params-20261123, in the order they are printed.
pub const GAIN_NAMES: [&str; 14] = [
    "ATC_ANG_RP_P",
    "ATC_ANG_YAW_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_RAT_RLL_IMAX",
    "ATC_RAT_PIT_P",
    "ATC_RAT_PIT_I",
    "ATC_RAT_PIT_D",
    "ATC_RAT_PIT_IMAX",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_FLTD",
];
/// The notch switch parameter.
pub const NOTCH_PARAM: &str = "RPM_NOTCH_EN";

/// One armed span: from the MOT record that first has Arm set to the first later record that has not.
#[derive(Clone, Debug)]
pub struct Armed {
    pub start_us: u64,
    /// Exclusive end: the first disarmed MOT record, or one past the last MOT record when the log ends armed.
    pub end_us: u64,
    pub open_at_end: bool,
    /// (time, mode code) at the start and at every change.
    pub modes: Vec<(u64, u8)>,
}

/// One DMD record.
#[derive(Clone, Copy, Debug)]
pub struct Dmd {
    pub t_us: u64,
    pub ok: bool,
    pub thrust_n: f32,
    pub torque: [f32; 3],
    pub sat: u8,
}

/// The rate-loop steps of PIDT joined with NTCH (and DMD) by `TimeUS`; a channel a step lacks is NaN.
#[derive(Default)]
pub struct Steps {
    pub t_us: Vec<u64>,
    pub tar: [Vec<f32>; 3],
    pub p: [Vec<f32>; 3],
    pub i: [Vec<f32>; 3],
    pub d: [Vec<f32>; 3],
    /// NTCH G: the rate the loop used.
    pub g: [Vec<f32>; 3],
    /// NTCH F0..F3: rotor fundamental by DShot line, Hz.
    pub f: [Vec<f32>; 4],
    pub notch_on: Vec<u8>,
    /// DMD.Sat of the same step; 0xFF when there is no DMD record for it.
    pub sat: Vec<u8>,
    /// DMD torque and accepted flag of the same step.
    pub dmd_torque: [Vec<f32>; 3],
    pub dmd_ok: Vec<u8>,
}

impl Steps {
    pub fn len(&self) -> usize {
        self.t_us.len()
    }
}

/// A parameter value from a PARM record.
#[derive(Clone, Debug)]
pub struct Parm {
    pub t_us: u64,
    pub name: String,
    pub value: f64,
}

pub struct LogData {
    pub file_name: String,
    pub bytes: u64,
    pub bad_bytes: u64,
    pub tail_bytes: u64,
    /// Records read, by type name.
    pub counts: BTreeMap<String, u64>,
    pub armed: Vec<Armed>,
    pub dmd: Vec<Dmd>,
    pub steps: Steps,
    pub has_pidt: bool,
    pub has_ntch: bool,
    pub parms: Vec<Parm>,
    /// (time, records dropped since the log opened) from FDS and DSF.
    pub fds_drops: Vec<(u64, u64)>,
    pub dsf_drops: Vec<(u64, u64)>,
    /// Records of PIDT or NTCH whose time did not exceed the previous one (dropped from the series).
    pub time_reversals: u64,
}

impl LogData {
    pub fn count(&self, name: &str) -> u64 {
        self.counts.get(name).copied().unwrap_or(0)
    }
}

struct Pidt {
    t: u64,
    v: [f32; 12],
}

struct Ntch {
    t: u64,
    g: [f32; 3],
    f: [f32; 4],
    on: u8,
}

fn num(r: &gcs_dflog::Record<'_>, name: &str) -> f64 {
    r.number_of(name).unwrap_or(f64::NAN)
}

pub fn read(path: &Path) -> Result<LogData, String> {
    let mut reader = Reader::open(path).map_err(|e| e.to_string())?;
    let mut data = LogData {
        file_name: path.file_name().map_or_else(
            || "(stream)".to_string(),
            |n| n.to_string_lossy().into_owned(),
        ),
        bytes: 0,
        bad_bytes: 0,
        tail_bytes: 0,
        counts: BTreeMap::new(),
        armed: Vec::new(),
        dmd: Vec::new(),
        steps: Steps::default(),
        has_pidt: false,
        has_ntch: false,
        parms: Vec::new(),
        fds_drops: Vec::new(),
        dsf_drops: Vec::new(),
        time_reversals: 0,
    };
    let (mut pidt, mut ntch): (Vec<Pidt>, Vec<Ntch>) = (Vec::new(), Vec::new());
    let mut armed_now = false;
    loop {
        let rec = match reader.next_record() {
            Ok(Some(rec)) => rec,
            Ok(None) => break,
            Err(e) => return Err(e.to_string()),
        };
        let name = rec.type_name();
        if let Some(c) = data.counts.get_mut(name) {
            *c += 1;
        } else {
            data.counts.insert(name.to_string(), 1);
        }
        let Some(t) = rec.time_us() else { continue };
        match name {
            "PIDT" => {
                let mut v = [0f32; 12];
                for (slot, col) in v.iter_mut().zip([
                    "TarX", "PX", "IX", "DX", "TarY", "PY", "IY", "DY", "TarZ", "PZ", "IZ", "DZ",
                ]) {
                    *slot = num(&rec, col) as f32;
                }
                pidt.push(Pidt { t, v });
            }
            "NTCH" => {
                let g = ["GX", "GY", "GZ"].map(|c| num(&rec, c) as f32);
                let f = ["F0", "F1", "F2", "F3"].map(|c| num(&rec, c) as f32);
                ntch.push(Ntch {
                    t,
                    g,
                    f,
                    on: num(&rec, "On") as u8,
                });
            }
            "DMD" => data.dmd.push(Dmd {
                t_us: t,
                ok: num(&rec, "Ok") != 0.0,
                thrust_n: num(&rec, "Thr") as f32,
                torque: ["TX", "TY", "TZ"].map(|c| num(&rec, c) as f32),
                sat: num(&rec, "Sat") as u8,
            }),
            "MOT" => {
                let arm = num(&rec, "Arm") != 0.0;
                let mode = num(&rec, "Mode") as u8;
                if arm && !armed_now {
                    data.armed.push(Armed {
                        start_us: t,
                        end_us: t + 1,
                        open_at_end: true,
                        modes: vec![(t, mode)],
                    });
                } else if let Some(span) = data.armed.last_mut().filter(|_| armed_now) {
                    if arm {
                        if span.modes.last().is_none_or(|m| m.1 != mode) {
                            span.modes.push((t, mode));
                        }
                        span.end_us = t + 1;
                    } else {
                        span.end_us = t;
                        span.open_at_end = false;
                    }
                }
                armed_now = arm;
            }
            "PARM" => {
                if let Some(Value::Text(n)) = rec.get("Name")
                    && (n.starts_with("ATC_") || n == NOTCH_PARAM)
                {
                    data.parms.push(Parm {
                        t_us: t,
                        name: n.into_owned(),
                        value: num(&rec, "Value"),
                    });
                }
            }
            "FDS" => data.fds_drops.push((t, num(&rec, "Dp") as u64)),
            "DSF" => data.dsf_drops.push((t, num(&rec, "Dp") as u64)),
            _ => {}
        }
    }
    data.bytes = reader.bytes_consumed();
    data.bad_bytes = reader.bad_bytes();
    data.tail_bytes = reader.tail_bytes();
    data.has_pidt = !pidt.is_empty();
    data.has_ntch = !ntch.is_empty();
    join(&mut data, pidt, ntch);
    Ok(data)
}

/// Keeps strictly increasing times only, counting the rest.
fn increasing<T>(items: Vec<T>, time: impl Fn(&T) -> u64, reversals: &mut u64) -> Vec<T> {
    let mut last: Option<u64> = None;
    items
        .into_iter()
        .filter(|x| {
            let t = time(x);
            let ok = last.is_none_or(|l| t > l);
            if ok {
                last = Some(t);
            } else {
                *reversals += 1;
            }
            ok
        })
        .collect()
}

fn join(data: &mut LogData, pidt: Vec<Pidt>, ntch: Vec<Ntch>) {
    let mut rev = 0;
    let pidt = increasing(pidt, |p| p.t, &mut rev);
    let ntch = increasing(ntch, |n| n.t, &mut rev);
    data.time_reversals = rev;
    let dmd_at: std::collections::HashMap<u64, usize> = data
        .dmd
        .iter()
        .enumerate()
        .map(|(i, d)| (d.t_us, i))
        .collect();
    let s = &mut data.steps;
    let push = |s: &mut Steps, t: u64, p: Option<&Pidt>, n: Option<&Ntch>, dmd: Option<&Dmd>| {
        s.t_us.push(t);
        for a in 0..3 {
            let get = |k: usize| p.map_or(f32::NAN, |p| p.v[4 * a + k]);
            s.tar[a].push(get(0));
            s.p[a].push(get(1));
            s.i[a].push(get(2));
            s.d[a].push(get(3));
            s.g[a].push(n.map_or(f32::NAN, |n| n.g[a]));
            s.dmd_torque[a].push(dmd.map_or(f32::NAN, |d| d.torque[a]));
        }
        for l in 0..4 {
            s.f[l].push(n.map_or(f32::NAN, |n| n.f[l]));
        }
        s.notch_on.push(n.map_or(0xFF, |n| n.on));
        s.sat.push(dmd.map_or(0xFF, |d| d.sat));
        s.dmd_ok.push(dmd.map_or(0xFF, |d| u8::from(d.ok)));
    };
    match (pidt.is_empty(), ntch.is_empty()) {
        (true, true) => {}
        (false, true) => {
            for p in &pidt {
                push(
                    s,
                    p.t,
                    Some(p),
                    None,
                    dmd_at.get(&p.t).map(|&i| &data.dmd[i]),
                );
            }
        }
        (true, false) => {
            for n in &ntch {
                push(
                    s,
                    n.t,
                    None,
                    Some(n),
                    dmd_at.get(&n.t).map(|&i| &data.dmd[i]),
                );
            }
        }
        (false, false) => {
            // Both: the steps both have (they are written together, one step each).
            let by_time: std::collections::HashMap<u64, usize> =
                ntch.iter().enumerate().map(|(i, n)| (n.t, i)).collect();
            let mut matched = 0usize;
            for p in &pidt {
                if let Some(&j) = by_time.get(&p.t) {
                    matched += 1;
                    push(
                        s,
                        p.t,
                        Some(p),
                        Some(&ntch[j]),
                        dmd_at.get(&p.t).map(|&i| &data.dmd[i]),
                    );
                }
            }
            if matched == 0 {
                // No common step: use none of it (the two cannot be aligned).
                *s = Steps::default();
                data.has_pidt = false;
                data.has_ntch = false;
            }
        }
    }
}
