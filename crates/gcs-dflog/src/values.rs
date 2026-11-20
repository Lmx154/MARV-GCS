//! The closing block of the flight report: the vehicle values the log supports, gathered from the results the
//! report already has (nothing is computed a second way, nothing is applied).

use crate::flight_report::{Flight, FlightReport, MotorFit, MotorFitResult};
use crate::vibration::IntervalKind;

/// Mean over the flights, weighted by duration, with the per-flight smallest and largest.
#[derive(Clone, Debug, PartialEq)]
pub struct WeightedStat {
    pub mean: f64,
    pub min: f64,
    pub max: f64,
    /// Flights that had the value.
    pub flights: usize,
}

/// The thrust constant: round 2, from [`crate::ThrustConstant`].
#[derive(Clone, Debug, PartialEq)]
pub struct ThrustValue {
    pub k2: f64,
    pub flights_min: f64,
    pub flights_max: f64,
    pub flights: usize,
}

/// The hover figures over all flights.
#[derive(Clone, Debug, PartialEq)]
pub struct HoverValues {
    pub flights: usize,
    /// `RCS.T`, 0..1.
    pub stick: Option<WeightedStat>,
    /// `SETP.Thr` over mass x g.
    pub collective_fraction: Option<WeightedStat>,
    /// The lowest and the highest of the flights' lowest battery voltage.
    pub volts_min_lowest: Option<f64>,
    pub volts_min_highest: Option<f64>,
}

/// The 1 g window shares over all flights, weighted by sample count.
#[derive(Clone, Debug, PartialEq)]
pub struct OneGValues {
    pub samples: u64,
    pub raw: f64,
    pub filtered: Vec<(f64, f64)>,
}

/// What the log supports.
#[derive(Clone, Debug, PartialEq)]
pub struct ValuesSupported {
    pub thrust: Option<ThrustValue>,
    /// The pooled fit of the motors' line.
    pub line: Option<MotorFitResult>,
    pub hover: HoverValues,
    /// `None`: no flight with IMU samples.
    pub one_g: Option<OneGValues>,
}

impl ValuesSupported {
    pub(crate) fn empty() -> ValuesSupported {
        ValuesSupported {
            thrust: None,
            line: None,
            hover: HoverValues {
                flights: 0,
                stick: None,
                collective_fraction: None,
                volts_min_lowest: None,
                volts_min_highest: None,
            },
            one_g: None,
        }
    }
}

/// The speed (rad/s) of a fitted line at `level`, and whether `level` is outside the fitted range.
pub fn line_speed(fit: &MotorFit, level: f64) -> (f64, bool) {
    (
        fit.a + fit.b * level,
        level < fit.level_min || level > fit.level_max,
    )
}

fn weighted(flights: &[&Flight], value: impl Fn(&Flight) -> Option<f64>) -> Option<WeightedStat> {
    let (mut sum, mut weight, mut n) = (0.0, 0.0, 0);
    let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
    for f in flights {
        let Some(v) = value(f) else { continue };
        let w = (f.end_us - f.start_us) as f64;
        sum += v * w;
        weight += w;
        n += 1;
        min = min.min(v);
        max = max.max(v);
    }
    (n > 0 && weight > 0.0).then(|| WeightedStat {
        mean: sum / weight,
        min,
        max,
        flights: n,
    })
}

pub(crate) fn values_of(r: &FlightReport) -> ValuesSupported {
    let flights: Vec<&Flight> = r.windows.iter().flat_map(|w| &w.flights).collect();
    let thrust = r.identification.thrust.as_ref().and_then(|t| {
        t.k2.map(|k2| ThrustValue {
            k2,
            flights_min: t.flights_min,
            flights_max: t.flights_max,
            flights: t.per_flight.len(),
        })
    });
    let volts: Vec<f64> = flights.iter().filter_map(|f| f.volts_min).collect();
    let hover = HoverValues {
        flights: flights.len(),
        stick: weighted(&flights, |f| f.stick_mean),
        collective_fraction: weighted(&flights, |f| f.collective_fraction),
        volts_min_lowest: volts.iter().copied().reduce(f64::min),
        volts_min_highest: volts.iter().copied().reduce(f64::max),
    };
    let ivs: Vec<_> = r
        .vibration
        .intervals
        .iter()
        .filter(|v| v.kind == IntervalKind::Flight && v.samples > 0)
        .filter_map(|v| v.one_g.as_ref().map(|g| (v.samples as f64, g)))
        .collect();
    let total: f64 = ivs.iter().map(|(n, _)| n).sum();
    let one_g = (total > 0.0).then(|| OneGValues {
        samples: total as u64,
        raw: ivs.iter().map(|(n, g)| n * g.raw).sum::<f64>() / total,
        filtered: ivs[0]
            .1
            .filtered
            .iter()
            .enumerate()
            .map(|(i, (hz, _))| {
                (
                    *hz,
                    ivs.iter().map(|(n, g)| n * g.filtered[i].1).sum::<f64>() / total,
                )
            })
            .collect(),
    });
    ValuesSupported {
        thrust,
        line: r.identification.pooled.clone(),
        hover,
        one_g,
    }
}
