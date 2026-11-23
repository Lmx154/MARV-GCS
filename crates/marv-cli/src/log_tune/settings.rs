//! Every threshold and window of `log tune`, as a named constant with its default, its unit, where the value comes
//! from, and the flag that overrides it. The table drives the argument parser, the header of the text output and
//! the `constants` list of the JSON document, so the three cannot differ.

use std::fmt::Write as _;

macro_rules! settings {
    ($( $id:ident, $name:literal, $flag:literal, $unit:literal, $default:expr, $lo:expr, $hi:expr, $kind:ident, $source:literal; )*) => {
        /// The index of a setting in [`Settings`].
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[allow(clippy::enum_variant_names)]
        pub enum Id { $( $id, )* }

        /// What a setting is: a number, a whole number, or a power of two (a Welch segment length).
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub enum Kind { Real, Whole, Odd, PowerOfTwo }

        /// One row of the table.
        pub struct Def {
            pub id: Id,
            pub name: &'static str,
            pub flag: &'static str,
            pub unit: &'static str,
            pub default: f64,
            pub lo: f64,
            pub hi: f64,
            pub kind: Kind,
            pub source: &'static str,
        }

        pub const DEFS: &[Def] = &[
            $( Def { id: Id::$id, name: $name, flag: $flag, unit: $unit, default: $default, lo: $lo, hi: $hi, kind: Kind::$kind, source: $source }, )*
        ];
    };
}

settings! {
    QuietBand, "QUIET_BAND_RADS", "quiet-band", "rad/s", 0.10, 0.001, 5.0, Real,
        "this tool's choice (5.7 deg/s), not from the earlier scripts: a rate target inside it counts as hands-off";
    QuietHold, "QUIET_HOLD_S", "quiet-hold", "s", 0.20, 0.0, 5.0, Real,
        "this tool's choice: an excursion ends when the target has stayed in the quiet band this long, so a doublet's crossing stays one event";
    MinEventPeak, "MIN_EVENT_PEAK_RADS", "min-event-peak", "rad/s", 0.30, 0.01, 20.0, Real,
        "this tool's choice (17 deg/s): a smaller excursion is not a usable event";
    PreQuiet, "PRE_QUIET_S", "pre-quiet", "s", 0.15, 0.01, 5.0, Real,
        "this tool's choice: quiet time needed before an event; its median is the baseline of both signals";
    EventTail, "EVENT_TAIL_S", "event-tail", "s", 0.60, 0.05, 10.0, Real,
        "this tool's choice: the response is followed this long after the target returns (analysis-292.md: back within 2 deg in 0.3-1.5 s)";
    MinTail, "MIN_TAIL_S", "min-tail", "s", 0.30, 0.0, 10.0, Real,
        "this tool's choice: an event with less room than this after it (next event, end of data) is not usable";
    MinEvents, "MIN_EVENTS", "min-events", "events", 3.0, 1.0, 1000.0, Whole,
        "this tool's choice: fewest usable events for the response measures and the frequency response of an axis";
    MaxDelay, "MAX_DELAY_S", "max-delay", "s", 0.30, 0.01, 2.0, Real,
        "this tool's choice: the largest delay searched (cross-correlation lag, dead time of the fit); analysis-292.md found 58-92 ms";
    FitMinR2, "FIT_MIN_R2", "fit-min-r2", "", 0.80, 0.0, 1.0, Real,
        "this tool's choice: an event whose first-order fit explains less of the achieved rate's variance is left out of the fit statistics";
    PlateauTol, "PLATEAU_TOL", "plateau-tol", "", 0.15, 0.01, 0.9, Real,
        "this tool's choice: the target is held while within this fraction of its peak";
    PlateauMin, "PLATEAU_MIN_S", "plateau-min", "s", 0.25, 0.01, 10.0, Real,
        "this tool's choice: shortest hold that counts for overshoot, settling and steady error";
    SettleTol, "SETTLE_TOL", "settle-tol", "", 0.05, 0.001, 0.9, Real,
        "this tool's choice: settled when within this fraction of the held target (5-point average of the achieved rate)";
    SteadyFrac, "STEADY_FRAC", "steady-frac", "", 0.30, 0.05, 1.0, Real,
        "this tool's choice: the steady error is the mean over this last fraction of the hold";
    RiseLo, "RISE_LO", "rise-lo", "", 0.10, 0.01, 0.5, Real,
        "convention: rise time from 10 % ...";
    RiseHi, "RISE_HI", "rise-hi", "", 0.90, 0.5, 0.99, Real,
        "convention: ... to 90 % of the achieved peak";
    FrfSeg, "FRF_SEG", "frf-seg", "samples", 4096.0, 64.0, 65536.0, PowerOfTwo,
        "t292-loops2.py: csd_sum n = 4096, linear detrend (Hann, 50 % overlap)";
    FrfMinSegments, "FRF_MIN_SEGMENTS", "frf-min-segments", "segments", 8.0, 2.0, 10000.0, Whole,
        "this tool's choice: coherence from fewer averaged segments is biased high (about 1/segments)";
    FrfSmooth, "FRF_SMOOTH_BINS", "frf-smooth", "bins", 3.0, 1.0, 101.0, Odd,
        "this tool's choice: the auto and cross spectra are averaged over this many neighbouring bins before H1 and the coherence are formed (the random error of one bin is large with a few segments)";
    Coherence, "COHERENCE_MIN", "coherence", "", 0.5, 0.0, 1.0, Real,
        "t292-loops2.py: coherent where coh > 0.5";
    FrfMinHz, "FRF_MIN_HZ", "frf-min-hz", "Hz", 0.25, 0.0, 100.0, Real,
        "this tool's choice: bins below it (segment leakage) are not read";
    FrfRefMaxHz, "FRF_REF_MAX_HZ", "frf-ref-max-hz", "Hz", 0.6, 0.1, 100.0, Real,
        "this tool's choice: the -3 dB point is taken from the median gain of the coherent bins from FRF_MIN_HZ to here (always at least the two lowest coherent bins), so a -3 dB point below about 1.5 Hz reads high";
    NoiseMinSegments, "NOISE_MIN_SEGMENTS", "noise-min-segments", "segments", 4.0, 1.0, 10000.0, Whole,
        "this tool's choice: a spectrum averaged over fewer segments is too noisy to name peaks or an rms above the split";
    NoiseSeg, "NOISE_SEG", "noise-seg", "samples", 2048.0, 64.0, 65536.0, PowerOfTwo,
        "t292-notch.py hf(): nperseg 2048 (t292-vib.py: 2048 when the interval has over 8192 samples), constant detrend";
    HfSplit, "HF_SPLIT_HZ", "hf-split-hz", "Hz", 50.0, 1.0, 400.0, Real,
        "t292-notch.py hf(): rms of the density at or above 50 Hz";
    Peaks, "PEAKS", "peaks", "peaks", 4.0, 1.0, 20.0, Whole,
        "t292-vib.py peaks(): n = 4";
    PeakMinHz, "PEAK_MIN_HZ", "peak-min-hz", "Hz", 20.0, 0.0, 400.0, Real,
        "t292-vib.py peaks(): lo = 20 Hz";
    PeakSepHz, "PEAK_SEP_HZ", "peak-sep-hz", "Hz", 4.0, 0.0, 400.0, Real,
        "t292-vib.py peaks(): distance 4 Hz";
    PeakHalfBins, "PEAK_HALF_BINS", "peak-half-bins", "bins", 2.0, 0.0, 50.0, Whole,
        "t292-vib.py peaks(): band rms over the peak bin +-2";
    RotorPct, "ROTOR_RANGE_PCT", "rotor-pct", "%", 2.0, 0.0, 49.0, Real,
        "this tool's choice: a rotor frequency range is the 2nd to 98th percentile of the valid values (an eRPM glitch does not widen it)";
    AirborneFrac, "AIRBORNE_THRUST_FRAC", "airborne-frac", "", 0.60, 0.05, 1.0, Real,
        "this tool's choice: airborne = the thrust demand (DMD.Thr) is at least this fraction of its 90th percentile over the armed span (the log holds no mass)";
    MinAirborne, "MIN_AIRBORNE_S", "min-airborne", "s", 2.0, 0.0, 3600.0, Real,
        "this tool's choice: an armed span with less airborne time is not analysed";
    AirborneMerge, "AIRBORNE_MERGE_S", "airborne-merge", "s", 0.25, 0.0, 60.0, Real,
        "this tool's choice: a thrust dip shorter than this does not end the airborne time";
    GapFactor, "GAP_FACTOR", "gap-factor", "x dt", 2.5, 1.1, 1000.0, Real,
        "this tool's choice: a step spacing above this many median spacings is a gap in the data; no window spans one";
    HoverSettle, "HOVER_SETTLE_S", "hover-settle", "s", 0.6, 0.0, 60.0, Real,
        "this tool's choice (= EVENT_TAIL_S): hover-like = all three rate targets in the quiet band for this long, so outside every event and its tail";
}

/// Welch overlap of every spectrum, a fixed fraction of the segment (the earlier scripts use scipy's default).
pub const WELCH_OVERLAP: f64 = 0.5;

/// Frequencies of the frequency-response table, Hz: the list of t292-loops2.py (0.5 to 5 Hz) with 8 and 10 Hz added.
pub const FRF_FREQS_HZ: [f64; 9] = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 8.0, 10.0];

/// The current values; indexed by [`Id`].
#[derive(Clone, Debug, PartialEq)]
pub struct Settings(Vec<f64>);

impl Default for Settings {
    fn default() -> Settings {
        Settings(DEFS.iter().map(|d| d.default).collect())
    }
}

impl Settings {
    pub fn get(&self, id: Id) -> f64 {
        self.0[id as usize]
    }

    pub fn count(&self, id: Id) -> usize {
        self.0[id as usize] as usize
    }

    /// Sets the setting called by `flag` (without the dashes) from `text`; the error is the whole message.
    pub fn set_flag(&mut self, flag: &str, text: &str) -> Result<(), String> {
        let def = DEFS
            .iter()
            .find(|d| d.flag == flag)
            .ok_or_else(|| format!("unknown option --{flag}"))?;
        let v: f64 = text
            .parse()
            .ok()
            .filter(|v: &f64| v.is_finite())
            .ok_or_else(|| format!("--{flag} needs a number, not {text:?}"))?;
        if v < def.lo || v > def.hi {
            return Err(format!(
                "--{flag} needs a number from {} to {}, not {text}",
                def.lo, def.hi
            ));
        }
        match def.kind {
            Kind::Real => {}
            Kind::Whole if v.fract() == 0.0 => {}
            Kind::PowerOfTwo if v.fract() == 0.0 && (v as u64).is_power_of_two() => {}
            Kind::Odd if v.fract() == 0.0 && v as u64 % 2 == 1 => {}
            Kind::Odd => return Err(format!("--{flag} needs an odd whole number, not {text}")),
            Kind::Whole => return Err(format!("--{flag} needs a whole number, not {text}")),
            Kind::PowerOfTwo => return Err(format!("--{flag} needs a power of two, not {text}")),
        }
        self.0[def.id as usize] = v;
        Ok(())
    }

    /// Whether a flag names a setting.
    pub fn is_flag(flag: &str) -> bool {
        DEFS.iter().any(|d| d.flag == flag)
    }

    /// One line per setting for the header: `NAME = value unit (--flag; source)`; a changed value says so.
    pub fn header(&self) -> String {
        let mut s = String::new();
        for d in DEFS {
            let v = self.get(d.id);
            let changed = if v == d.default {
                String::new()
            } else {
                format!(" [CHANGED by flag; default {}]", d.default)
            };
            let unit = if d.unit.is_empty() {
                String::new()
            } else {
                format!(" {}", d.unit)
            };
            let _ = writeln!(
                s,
                "    {} = {v}{unit}{changed} (--{}; {})",
                d.name, d.flag, d.source
            );
        }
        let _ = writeln!(
            s,
            "    WELCH_OVERLAP = {} of the segment (fixed; scipy's default, as the earlier scripts)",
            WELCH_OVERLAP
        );
        let _ = writeln!(
            s,
            "    FRF_FREQS_HZ = {} (fixed; t292-loops2.py FQ with 8 and 10 added)",
            FRF_FREQS_HZ
                .iter()
                .map(|f| f.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );
        s
    }

    pub fn to_json(&self) -> serde_json::Value {
        let mut list: Vec<serde_json::Value> = DEFS
            .iter()
            .map(|d| {
                serde_json::json!({
                    "name": d.name, "flag": format!("--{}", d.flag), "value": self.get(d.id),
                    "default": d.default, "unit": d.unit, "source": d.source,
                })
            })
            .collect();
        list.push(serde_json::json!({"name": "WELCH_OVERLAP", "flag": null, "value": WELCH_OVERLAP, "default": WELCH_OVERLAP, "unit": "", "source": "fixed; scipy's default, as the earlier scripts"}));
        serde_json::Value::Array(list)
    }
}
