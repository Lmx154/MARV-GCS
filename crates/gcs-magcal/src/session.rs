//! The guided calibration as a pure state machine (decision sections 2, 3 and 4; link contract section 16).
//!
//! No clock, no I/O, nothing written: the caller feeds it the start conditions, HIGHRES_IMU samples, parameter
//! changes, the armed state and the time, and reads back a [`Status`] for the view and an [`Outcome`]. The outcome
//! is either an ending with a plain message or [`Outcome::Accepted`] with the ordered [`write_plan`] the caller
//! sends; the session itself writes nothing.
//!
//! Time. `now` is seconds on any monotonic clock of the caller (it is only differenced); [`Session::tick`] must be
//! called regularly (the view's frame rate is enough). A sample carries the vehicle's own `time_usec`, used for the
//! gyro integration only. The 90 s of a pose restart at the `now` of the latest `start` or `tick` when a pose
//! completes.

use crate::Vec3;
use crate::accept::{self, Accepted, Refusal};
use crate::collect::Collector;
use crate::fit;
use crate::pose::Pose;
use crate::turn::{Ring, SECTORS, TurnTracker};
use crate::undo::{Calibration, INVALID_MATRIX_MESSAGE};
use crate::wmm::{self, Field, WmmError};

/// Seconds to complete a pose, from the start and again from each pose completed. Decision section 3 ("Time and
/// control"); PX4 `timeout = 90_s`, calibration_routines.cpp:78. Expired when the elapsed time reaches it.
pub const POSE_TIMEOUT_S: f64 = 90.0;
/// A GPS fix must be this type or better: PX4 `gnss.receiver.fix_type >= 2`, mag_calibration.cpp:181; decision
/// section 3 ("Position").
pub const GPS_MIN_FIX_TYPE: u8 = 2;
/// ... and younger than this, seconds: PX4 `hrt_elapsed_time(&gnss.timestamp) < 100_s`, mag_calibration.cpp:181.
/// (PX4 also requires `eph < 1000`; GPS_RAW_INT's `eph` is not an input here, the decision names type and age.)
pub const GPS_MAX_AGE_S: f64 = 100.0;

/// The refusal without a usable fix (decision section 3, "Position", verbatim).
pub const NO_POSITION_MESSAGE: &str =
    "Needs a GPS position to know the Earth's field here; take the vehicle outside";

/// The GPS fix read at the start.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gps {
    /// GPS_RAW_INT `fix_type` (2 = 2D, 3 = 3D, ...).
    pub fix_type: u8,
    /// Age of the fix, seconds.
    pub age_s: f64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height for the model, metres (the model's effect is a few tenths of a percent per kilometre).
    pub height_m: f64,
}

/// Why the run does not start; [`StartRefusal::message`] is the decision's text.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StartRefusal {
    /// The stored matrix is not positive definite.
    InvalidMatrix,
    /// No fix of type 2 or better younger than [`GPS_MAX_AGE_S`].
    NoPosition,
    /// The model has no value for this position or date (outside 2025.0 to 2030.0, or a position that is not valid).
    Model(WmmError),
}

impl StartRefusal {
    pub fn message(&self) -> String {
        match self {
            StartRefusal::InvalidMatrix => INVALID_MATRIX_MESSAGE.to_string(),
            StartRefusal::NoPosition => NO_POSITION_MESSAGE.to_string(),
            StartRefusal::Model(e) => format!("The Earth's field model cannot be used: {e}"),
        }
    }
}

impl std::fmt::Display for StartRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message())
    }
}

impl std::error::Error for StartRefusal {}

/// Why a run ended without a result; nothing was written in any case.
#[derive(Clone, Debug, PartialEq)]
pub enum Ending {
    /// The fit or an acceptance check refused it.
    Refused(Refusal),
    /// 90 s passed without completing a pose; the pose named.
    TimedOut(Pose),
    /// A parameter of the calibration in force changed during the run (the name as given).
    ParamChanged(String),
    Armed,
    Cancelled,
}

impl Ending {
    pub fn message(&self) -> String {
        match self {
            Ending::Refused(r) => r.message(),
            Ending::TimedOut(p) => format!("Timed out waiting for {p}: start again"),
            Ending::ParamChanged(name) => {
                format!("{name} changed during the calibration: stopped, start again")
            }
            Ending::Armed => "The vehicle was armed: calibration stopped".to_string(),
            Ending::Cancelled => "Calibration cancelled".to_string(),
        }
    }
}

/// A parameter value of the write plan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamValue {
    F32(f32),
    I32(i32),
}

impl ParamValue {
    /// The value as a PARAM_VALUE carries it, a float.
    pub fn as_f32(self) -> f32 {
        match self {
            ParamValue::F32(v) => v,
            ParamValue::I32(v) => v as f32,
        }
    }
}

/// `MAG_CAL_TYPE` values (link contract section 16).
pub const CAL_TYPE_NONE: i32 = 0;
pub const CAL_TYPE_GUIDED: i32 = 2;

/// The parameters whose change during the run abandons it: `MAG_OFFX/Y/Z`, `MAG_DIAGX/Y/Z`,
/// `MAG_ODIAGXY/XZ/YZ` (decision section 2; link contract section 16, "The station's undo").
pub const WATCHED_PARAMS: [&str; 9] = [
    "MAG_OFFX",
    "MAG_OFFY",
    "MAG_OFFZ",
    "MAG_DIAGX",
    "MAG_DIAGY",
    "MAG_DIAGZ",
    "MAG_ODIAGXY",
    "MAG_ODIAGXZ",
    "MAG_ODIAGYZ",
];

/// One write of the plan.
pub type Write = (&'static str, ParamValue);

/// The ordered writes of the commit (link contract section 16, "Write order", steps 1-3): `MAG_CAL_TYPE` 0;
/// `MAG_OFFX/Y/Z`, `MAG_DIAGX/Y/Z`, `MAG_ODIAGXY/XZ/YZ`, `MAG_FLD_UT`, `MAG_FLD_INC`; `MAG_CAL_TYPE` 2. Thirteen
/// writes (the decision's section 2 counts 13 PARAM_SET). F32 parameters carry the value as f32.
pub fn write_plan(accepted: &Accepted) -> Vec<Write> {
    let c = &accepted.calibration;
    let f = |v: f64| ParamValue::F32(v as f32);
    vec![
        ("MAG_CAL_TYPE", ParamValue::I32(CAL_TYPE_NONE)),
        ("MAG_OFFX", f(c.offsets[0])),
        ("MAG_OFFY", f(c.offsets[1])),
        ("MAG_OFFZ", f(c.offsets[2])),
        ("MAG_DIAGX", f(c.diag[0])),
        ("MAG_DIAGY", f(c.diag[1])),
        ("MAG_DIAGZ", f(c.diag[2])),
        ("MAG_ODIAGXY", f(c.offdiag[0])),
        ("MAG_ODIAGXZ", f(c.offdiag[1])),
        ("MAG_ODIAGYZ", f(c.offdiag[2])),
        ("MAG_FLD_UT", f(accepted.field_ut)),
        ("MAG_FLD_INC", f(accepted.inclination_deg)),
        ("MAG_CAL_TYPE", ParamValue::I32(CAL_TYPE_GUIDED)),
    ]
}

/// Step 4 of the write order: compares what was read back with the plan, every value equal as f32.
///
/// `read_back` holds (name, value as PARAM_VALUE carries it) for the plan's parameters. `MAG_CAL_TYPE` is in the
/// plan twice; the last write is what must be read back. `Err` names the first parameter that is missing or
/// differs. The station offers Save only after `Ok`.
pub fn verify_read_back(plan: &[Write], read_back: &[(&str, f32)]) -> Result<(), String> {
    for (i, &(name, value)) in plan.iter().enumerate() {
        if plan[i + 1..].iter().any(|&(n, _)| n == name) {
            continue;
        }
        let want = value.as_f32();
        match read_back.iter().find(|&&(n, _)| n == name) {
            None => return Err(format!("{name} was not read back")),
            Some(&(_, got)) if got != want => {
                return Err(format!("{name} reads back {got}, written {want}"));
            }
            Some(_) => {}
        }
    }
    Ok(())
}

/// A finished, accepted run.
#[derive(Clone, Debug, PartialEq)]
pub struct Success {
    pub accepted: Accepted,
    /// What to write, in order ([`write_plan`]).
    pub plan: Vec<Write>,
    /// Samples the fit used.
    pub samples: usize,
}

/// Where the run is.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Running,
    Accepted(Box<Success>),
    Ended(Ending),
}

impl Outcome {
    pub fn message(&self) -> Option<String> {
        match self {
            Outcome::Running => None,
            Outcome::Accepted(_) => Some(accept::ACCEPTED_MESSAGE.to_string()),
            Outcome::Ended(e) => Some(e.message()),
        }
    }
}

/// One pose in the [`Status`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PoseStatus {
    pub pose: Pose,
    /// Visited 30-deg sectors; index 0 is where the turn started.
    pub visited: [bool; SECTORS],
    pub visited_count: usize,
    pub complete: bool,
}

/// What the view shows.
#[derive(Clone, Debug, PartialEq)]
pub struct Status {
    /// The pose the last sample was in.
    pub recognised: Option<Pose>,
    /// The pose to show as the target: the one being held when it is incomplete, else the first incomplete.
    pub target: Option<Pose>,
    /// The six poses in [`Pose::ALL`] order.
    pub poses: [PoseStatus; 6],
    /// Seconds left for the pose in progress; 0 when the run is over.
    pub seconds_left: f64,
    /// Samples kept so far.
    pub samples_kept: usize,
    /// One line for the user.
    pub instruction: String,
}

/// The run.
#[derive(Clone, Debug)]
pub struct Session {
    model: Field,
    tracker: TurnTracker,
    collector: Collector,
    /// `now` of the last `start`, `tick`, or the restart of the 90 s.
    clock: f64,
    deadline_from: f64,
    recognised: Option<Pose>,
    outcome: Outcome,
}

impl Session {
    /// Starts a run with the calibration in force as read from the vehicle, the GPS fix, the date as a decimal
    /// year (for the model) and the time.
    ///
    /// Refuses, in this order, when the stored matrix is not positive definite, when `gps` is `None` or its fix
    /// is of a type below [`GPS_MIN_FIX_TYPE`] or older than [`GPS_MAX_AGE_S`], or when the model has no value.
    /// (The armed state is the caller's: it refuses to start while armed, then calls [`Session::armed`] if the
    /// vehicle arms later.)
    pub fn start(
        calibration: Calibration,
        gps: Option<Gps>,
        decimal_year: f64,
        now: f64,
    ) -> Result<Session, StartRefusal> {
        let undo = calibration
            .undo()
            .map_err(|_| StartRefusal::InvalidMatrix)?;
        let gps = gps
            .filter(|g| {
                g.fix_type >= GPS_MIN_FIX_TYPE && g.age_s.is_finite() && g.age_s < GPS_MAX_AGE_S
            })
            .ok_or(StartRefusal::NoPosition)?;
        let model = wmm::field(gps.lat_deg, gps.lon_deg, gps.height_m, decimal_year)
            .map_err(StartRefusal::Model)?;
        Ok(Session {
            collector: Collector::new(undo, model.intensity_ut),
            model,
            tracker: TurnTracker::new(),
            clock: now,
            deadline_from: now,
            recognised: None,
            outcome: Outcome::Running,
        })
    }

    pub fn outcome(&self) -> &Outcome {
        &self.outcome
    }

    pub fn is_running(&self) -> bool {
        self.outcome == Outcome::Running
    }

    /// The model's field at the GPS position, the one the run is judged against.
    pub fn model(&self) -> &Field {
        &self.model
    }

    /// One HIGHRES_IMU sample (`id` 0): vehicle time, accelerometer m/s^2, gyro rad/s, magnetometer gauss (the
    /// calibration in force applied), all body frame FRD. Ignored once the run is over.
    pub fn sample(
        &mut self,
        time_usec: u64,
        accel: [f32; 3],
        gyro: [f32; 3],
        mag_gauss: [f32; 3],
    ) -> &Outcome {
        if !self.is_running() {
            return &self.outcome;
        }
        let a = accel.map(f64::from);
        let g = gyro.map(f64::from);
        let update = self.tracker.update(time_usec, a, g);
        self.recognised = update.pose;
        if update.pose.is_some() {
            let norm = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
            let down: Vec3 = [-a[0] / norm, -a[1] / norm, -a[2] / norm];
            self.collector.offer(mag_gauss, down);
        }
        if update.completed.is_some() {
            self.deadline_from = self.clock;
            if self.tracker.all_complete() {
                self.finish();
            }
        }
        &self.outcome
    }

    /// A parameter changed on the vehicle (a PARAM_VALUE with a different value). Abandons the run when it is one
    /// of [`WATCHED_PARAMS`]; any other name is ignored.
    pub fn param_changed(&mut self, name: &str) -> &Outcome {
        if self.is_running() && WATCHED_PARAMS.contains(&name) {
            self.outcome = Outcome::Ended(Ending::ParamChanged(name.to_string()));
        }
        &self.outcome
    }

    /// HEARTBEAT shows armed: stops the run.
    pub fn armed(&mut self) -> &Outcome {
        self.end(Ending::Armed)
    }

    /// The user cancelled.
    pub fn cancel(&mut self) -> &Outcome {
        self.end(Ending::Cancelled)
    }

    /// The time passes; ends the run when [`POSE_TIMEOUT_S`] have elapsed since the start or the last pose
    /// completed. Names the pose being held when it is incomplete, else the first incomplete one.
    pub fn tick(&mut self, now: f64) -> &Outcome {
        if !self.is_running() {
            return &self.outcome;
        }
        self.clock = now;
        if now - self.deadline_from >= POSE_TIMEOUT_S
            && let Some(pose) = self.target()
        {
            self.outcome = Outcome::Ended(Ending::TimedOut(pose));
        }
        &self.outcome
    }

    /// A snapshot for the view at time `now`.
    pub fn status(&self, now: f64) -> Status {
        let poses = Pose::ALL.map(|pose| {
            let ring: &Ring = self.tracker.ring(pose);
            PoseStatus {
                pose,
                visited: std::array::from_fn(|i| ring.is_visited(i)),
                visited_count: ring.visited_count(),
                complete: ring.is_complete(),
            }
        });
        let running = self.is_running();
        let target = if running { self.target() } else { None };
        let instruction = match (&self.outcome, self.recognised, target) {
            (Outcome::Running, None, _) => "hold it as shown".to_string(),
            (Outcome::Running, Some(p), _) if !self.tracker.is_complete(p) => {
                "turn it slowly once round about the vertical".to_string()
            }
            (Outcome::Running, Some(_), Some(next)) => {
                format!("this one is done: put it {next}")
            }
            (outcome, _, _) => outcome.message().unwrap_or_default(),
        };
        Status {
            recognised: self.recognised,
            target,
            poses,
            seconds_left: if running {
                (POSE_TIMEOUT_S - (now - self.deadline_from)).clamp(0.0, POSE_TIMEOUT_S)
            } else {
                0.0
            },
            samples_kept: self.collector.len(),
            instruction,
        }
    }

    /// The pose being held when incomplete, else the first incomplete one; `None` when all are complete.
    fn target(&self) -> Option<Pose> {
        match self.recognised {
            Some(p) if !self.tracker.is_complete(p) => Some(p),
            _ => self.tracker.incomplete().first().copied(),
        }
    }

    fn end(&mut self, ending: Ending) -> &Outcome {
        if self.is_running() {
            self.outcome = Outcome::Ended(ending);
        }
        &self.outcome
    }

    /// All six poses are complete: the fit, then the acceptance checks (coverage input empty, since it is met).
    fn finish(&mut self) {
        let (raw, down) = self.collector.raw_and_down();
        let fit = fit::fit(&raw, self.model.intensity_ut);
        let result = accept::accept(&accept::Inputs {
            incomplete_poses: &[],
            fit: &fit,
            samples: &raw,
            down: &down,
            model: &self.model,
            inclination_tolerance_deg: accept::INCLINATION_TOLERANCE_DEG,
        });
        self.outcome = match result {
            Ok(accepted) => Outcome::Accepted(Box::new(Success {
                plan: write_plan(&accepted),
                accepted,
                samples: raw.len(),
            })),
            Err(refusal) => Outcome::Ended(Ending::Refused(refusal)),
        };
    }
}
