//! The actions of `docs/link-contract.md` section 10 as the simulated vehicle runs them: what each
//! command number and parameter set means, and the progress of the one action that runs. Written
//! from the specification; it names no type of the ground station.

/// MAV_CMD_PREFLIGHT_CALIBRATION.
pub const PREFLIGHT_CALIBRATION: u16 = 241;
/// MAV_CMD_PREFLIGHT_STORAGE.
pub const PREFLIGHT_STORAGE: u16 = 245;
/// MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN.
pub const PREFLIGHT_REBOOT_SHUTDOWN: u16 = 246;
/// The project's MAV_CMD_MARV_DEVICE.
pub const MARV_DEVICE: u16 = 42_430;

/// MAV_CMD_DO_MOTOR_TEST (`docs/link-contract.md` section 15).
pub const DO_MOTOR_TEST: u16 = 209;
/// The highest throttle of a motor test, in percent, and the longest spin, in seconds.
pub const TEST_MAX_PERCENT: f32 = 5.0;
pub const TEST_MAX_SECONDS: f32 = 3.0;
/// rpm shown on the tested motor per level (the 2026-11-13 bench median: about 1 860 rpm at level 100).
pub const RPM_PER_LEVEL: f64 = 18.6;

/// A progress ACK is sent this often.
pub const PROGRESS_EVERY_MS: u64 = 500;
/// How long an action runs: a calibration, and every other action.
pub const CALIBRATION_MS: u64 = 3_000;
/// The compass calibration, shortened for the tests (the vehicle gives it 30 s of rotating).
pub const COMPASS_CALIBRATION_MS: u64 = 4_000;
pub const ACTION_MS: u64 = 2_000;

/// Devices that can be restarted and devices that have a self-test (`MARV_DEVICE` ids).
const RESTART: [u8; 7] = [1, 2, 3, 4, 5, 6, 9];
const SELF_TEST: [u8; 3] = [1, 2, 4];

/// What the vehicle makes of a command and its parameters before it looks whether it is busy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Admission {
    Unsupported,
    /// Refused by the parameters; the text is the reason for the STATUSTEXT.
    Denied(&'static str),
    /// All seven parameters of a calibration are zero.
    Cancel,
    /// A motor test spin: motor 0 to 3, level of 2000 (at least 1), how long.
    Spin {
        motor: usize,
        level: u32,
        duration_ms: u64,
    },
    /// A motor test stop (always accepted).
    Stop,
    Run {
        duration_ms: u64,
    },
}

fn only(params: &[f32; 7], index: usize, value: f32) -> bool {
    params
        .iter()
        .enumerate()
        .all(|(i, p)| if i == index { *p == value } else { *p == 0.0 })
}

/// 209 by the table of section 15; a NaN or an infinity fails every range.
fn admit_motor_test(params: &[f32; 7]) -> Admission {
    let whole = |v: f32, lo: f32, hi: f32| v.fract() == 0.0 && (lo..=hi).contains(&v);
    if !whole(params[0], 1.0, 4.0) {
        return Admission::Denied("motor test: motor must be 1 to 4");
    }
    if params[1] != 0.0 {
        return Admission::Denied("motor test: throttle type must be 0 (percent)");
    }
    if !(0.0..=TEST_MAX_PERCENT).contains(&params[2]) {
        return Admission::Denied("motor test: throttle must be 0 to 5 percent");
    }
    if !whole(params[4], 0.0, 1.0) {
        return Admission::Denied("motor test: motor count must be 0 or 1");
    }
    if params[5] != 0.0 && params[5] != 2.0 {
        return Admission::Denied("motor test: order must be 0 or 2");
    }
    if params[6] != 0.0 {
        return Admission::Denied("motor test: param7 must be 0");
    }
    let motor = params[0] as usize - 1;
    if params[2] == 0.0 {
        return Admission::Stop;
    }
    if !(params[3] > 0.0 && params[3] <= TEST_MAX_SECONDS) {
        return Admission::Denied("motor test: duration must be above 0 and at most 3 s");
    }
    Admission::Spin {
        motor,
        level: ((f64::from(params[2]) * 20.0).round() as u32).max(1),
        duration_ms: (f64::from(params[3]) * 1000.0).round() as u64,
    }
}

/// The admission of `command` with `params`.
pub fn admit(command: u16, params: &[f32; 7]) -> Admission {
    let run = |duration_ms| Admission::Run { duration_ms };
    match command {
        PREFLIGHT_CALIBRATION => {
            if params.iter().all(|p| *p == 0.0) {
                Admission::Cancel
            } else if only(params, 0, 1.0) || only(params, 4, 4.0) {
                run(CALIBRATION_MS)
            } else if only(params, 1, 1.0) {
                run(COMPASS_CALIBRATION_MS)
            } else {
                Admission::Denied("calibration not supported yet")
            }
        }
        PREFLIGHT_STORAGE => match params[0] {
            p if p.fract() == 0.0 && (0.0..=4.0).contains(&p) => run(ACTION_MS),
            _ => Admission::Denied("storage action not supported"),
        },
        PREFLIGHT_REBOOT_SHUTDOWN => {
            if params[0] == 1.0 {
                run(ACTION_MS)
            } else {
                Admission::Denied("only reboot is supported")
            }
        }
        MARV_DEVICE => {
            let device = params[0];
            let listed = |list: &[u8]| list.iter().any(|d| f32::from(*d) == device);
            let ok = match params[1] {
                1.0 => listed(&RESTART),
                2.0 => listed(&SELF_TEST),
                _ => false,
            };
            if ok {
                run(ACTION_MS)
            } else {
                Admission::Denied("device action not supported for this device")
            }
        }
        DO_MOTOR_TEST => admit_motor_test(params),
        _ => Admission::Unsupported,
    }
}

/// The one action that runs.
#[derive(Debug, Clone, PartialEq)]
pub struct Action {
    pub command: u16,
    pub params: [f32; 7],
    /// Who asked: system and component, where the ACKs go.
    pub from: (u8, u8),
    start_ms: u64,
    duration_ms: u64,
    /// When the next progress ACK is due.
    next_progress_ms: u64,
    /// Set when the final ACK has been handed out; nothing is due after it.
    finished: bool,
}

/// What the running action owes next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Due {
    Progress(u8),
    Final,
}

impl Action {
    pub fn new(
        command: u16,
        params: [f32; 7],
        from: (u8, u8),
        now_ms: u64,
        duration_ms: u64,
    ) -> Self {
        Self {
            command,
            params,
            from,
            start_ms: now_ms,
            duration_ms,
            next_progress_ms: now_ms + PROGRESS_EVERY_MS,
            finished: false,
        }
    }

    pub fn is_calibration(&self) -> bool {
        self.command == PREFLIGHT_CALIBRATION
    }

    /// Percent done at `now_ms`, below 100.
    pub fn percent(&self, now_ms: u64) -> u8 {
        let elapsed = now_ms.saturating_sub(self.start_ms);
        (elapsed * 100 / self.duration_ms).min(99) as u8
    }

    /// The next ACK due at `now_ms`, if any; progress ACKs are due every 500 ms and the final one at
    /// the end. Called again until `None`.
    pub fn next_due(&mut self, now_ms: u64) -> Option<Due> {
        if self.finished {
            return None;
        }
        let end_ms = self.start_ms + self.duration_ms;
        if now_ms >= end_ms && self.next_progress_ms >= end_ms {
            self.finished = true;
            return Some(Due::Final);
        }
        if now_ms >= self.next_progress_ms && self.next_progress_ms < end_ms {
            let percent = self.percent(self.next_progress_ms);
            self.next_progress_ms += PROGRESS_EVERY_MS;
            return Some(Due::Progress(percent));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn progress_every_500_ms_then_the_final_ack_at_the_end() {
        let mut action = Action::new(PREFLIGHT_STORAGE, [1.0; 7], (255, 190), 1_000, ACTION_MS);
        assert_eq!(action.next_due(1_499), None);
        assert_eq!(action.next_due(1_500), Some(Due::Progress(25)));
        assert_eq!(action.next_due(1_500), None);
        // A late call catches up with every ACK in order.
        assert_eq!(action.next_due(3_000), Some(Due::Progress(50)));
        assert_eq!(action.next_due(3_000), Some(Due::Progress(75)));
        assert_eq!(action.next_due(3_000), Some(Due::Final));
    }

    #[test]
    fn a_calibration_runs_3_s() {
        let mut action = Action::new(
            PREFLIGHT_CALIBRATION,
            [1.0; 7],
            (255, 190),
            0,
            CALIBRATION_MS,
        );
        let mut got = Vec::new();
        for _ in 0..100 {
            let Some(due) = action.next_due(3_000) else {
                break;
            };
            got.push(due);
        }
        assert_eq!(got.len(), 6, "the action must run dry: {got:?}");
        assert_eq!(got.last(), Some(&Due::Final));
    }

    #[test]
    fn a_compass_calibration_runs_4_s_with_progress_every_500_ms() {
        let mut action = Action::new(
            PREFLIGHT_CALIBRATION,
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            (255, 190),
            0,
            COMPASS_CALIBRATION_MS,
        );
        let mut got = Vec::new();
        for _ in 0..100 {
            let Some(due) = action.next_due(4_000) else {
                break;
            };
            got.push(due);
        }
        assert_eq!(got.len(), 8, "7 progress ACKs and the final one: {got:?}");
        assert_eq!(got.first(), Some(&Due::Progress(12)));
        assert_eq!(got.last(), Some(&Due::Final));
    }

    #[test]
    fn admission_follows_the_table() {
        let p = |i: usize, v: f32| {
            let mut a = [0.0; 7];
            a[i] = v;
            a
        };
        assert_eq!(
            admit(241, &p(0, 1.0)),
            Admission::Run { duration_ms: 3_000 }
        );
        assert_eq!(
            admit(241, &p(4, 4.0)),
            Admission::Run { duration_ms: 3_000 }
        );
        assert_eq!(admit(241, &[0.0; 7]), Admission::Cancel);
        assert_eq!(
            admit(241, &p(1, 1.0)),
            Admission::Run { duration_ms: 4_000 }
        );
        let mut both = p(0, 1.0);
        both[1] = 1.0;
        assert!(matches!(admit(241, &both), Admission::Denied(_)));
        assert!(matches!(admit(241, &p(1, 2.0)), Admission::Denied(_)));
        assert!(matches!(admit(245, &p(0, 5.0)), Admission::Denied(_)));
        assert_eq!(
            admit(245, &p(0, 3.0)),
            Admission::Run { duration_ms: 2_000 }
        );
        assert_eq!(
            admit(246, &p(0, 1.0)),
            Admission::Run { duration_ms: 2_000 }
        );
        assert!(matches!(admit(246, &p(0, 2.0)), Admission::Denied(_)));
        let device = |d: f32, a: f32| {
            let mut x = [0.0; 7];
            x[0] = d;
            x[1] = a;
            x
        };
        assert_eq!(
            admit(42_430, &device(3.0, 1.0)),
            Admission::Run { duration_ms: 2_000 }
        );
        assert!(matches!(
            admit(42_430, &device(3.0, 2.0)),
            Admission::Denied(_)
        ));
        assert_eq!(
            admit(42_430, &device(4.0, 2.0)),
            Admission::Run { duration_ms: 2_000 }
        );
        assert!(matches!(
            admit(42_430, &device(7.0, 1.0)),
            Admission::Denied(_)
        ));
        assert_eq!(admit(400, &[0.0; 7]), Admission::Unsupported);
    }

    #[test]
    fn motor_test_admission_follows_the_table() {
        let spin =
            |motor: f32, percent: f32, seconds: f32| [motor, 0.0, percent, seconds, 1.0, 2.0, 0.0];
        assert_eq!(
            admit(209, &spin(2.0, 5.0, 1.0)),
            Admission::Spin {
                motor: 1,
                level: 100,
                duration_ms: 1_000
            }
        );
        assert_eq!(
            admit(209, &spin(4.0, 0.05, 0.1)),
            Admission::Spin {
                motor: 3,
                level: 1,
                duration_ms: 100
            }
        );
        assert_eq!(admit(209, &spin(3.0, 0.0, 0.0)), Admission::Stop);
        assert_eq!(
            admit(209, &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            Admission::Stop
        );
        for bad in [
            spin(0.0, 5.0, 1.0),
            spin(5.0, 5.0, 1.0),
            spin(1.5, 5.0, 1.0),
            spin(f32::NAN, 5.0, 1.0),
            spin(1.0, 5.01, 1.0),
            spin(1.0, -1.0, 1.0),
            spin(1.0, f32::NAN, 1.0),
            spin(1.0, 5.0, 0.0),
            spin(1.0, 5.0, 3.01),
            [1.0, 1.0, 5.0, 1.0, 1.0, 2.0, 0.0],
            [1.0, 0.0, 5.0, 1.0, 2.0, 2.0, 0.0],
            [1.0, 0.0, 5.0, 1.0, 1.0, 1.0, 0.0],
            [1.0, 0.0, 5.0, 1.0, 1.0, 2.0, 1.0],
            // A stop is checked on the other parameters too.
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        ] {
            assert!(matches!(admit(209, &bad), Admission::Denied(_)), "{bad:?}");
        }
    }
}
