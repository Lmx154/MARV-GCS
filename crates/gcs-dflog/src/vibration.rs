//! The vibration figures of the flight report: standard deviations, Welch spectra, the rotor band and the 1 g
//! window of the IMU samples of one interval (a flight or a ground run). See [`crate::flight_report_with`].
//!
//! Streaming: an interval holds the current 1024-sample buffer and accumulators, never the samples.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use crate::flight_report::GRAVITY;
use crate::reader::Record;

/// The Welch segment, samples (a power of two).
pub const SEGMENT: usize = 1024;
/// Below this sample rate (Hz) no spectrum is made.
pub const MIN_SPECTRUM_RATE_HZ: f64 = 400.0;
/// Peaks reported per axis.
pub const PEAKS: usize = 3;
/// The amplitude of a peak is the rms within this many bins of it.
pub const PEAK_HALF_WIDTH: usize = 2;
/// The rotor band is widened by this share of itself on each side.
pub const BAND_MARGIN: f64 = 0.1;
/// The 1 g window, in g.
pub const ONE_G_WINDOW: (f64, f64) = (0.9, 1.1);

/// In-place radix-2 FFT of `re` + i `im`; the length is a power of two (a panic otherwise).
pub fn fft_in_place(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let twiddles: Vec<(f64, f64)> = (0..n / 2)
        .map(|k| {
            let a = -TAU * k as f64 / n as f64;
            (a.cos(), a.sin())
        })
        .collect();
    fft(re, im, &twiddles);
}

fn fft(re: &mut [f64], im: &mut [f64], twiddles: &[(f64, f64)]) {
    let n = re.len();
    assert!(n.is_power_of_two() && im.len() == n && twiddles.len() == n / 2);
    let bits = n.trailing_zeros();
    for i in 0..n {
        let j = if bits == 0 {
            0
        } else {
            i.reverse_bits() >> (usize::BITS - bits)
        };
        if j > i {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let step = n / len;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = twiddles[k * step];
                let (a, b) = (start + k, start + k + len / 2);
                let tr = re[b] * wr - im[b] * wi;
                let ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len *= 2;
    }
}

/// What kind of stretch of time the figures are of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntervalKind {
    Flight,
    GroundRun,
}

/// One peak of an averaged spectrum.
#[derive(Clone, Debug, PartialEq)]
pub struct Peak {
    pub hz: f64,
    /// The rms of the signal within +-[`PEAK_HALF_WIDTH`] bins of the peak, in the signal's unit (a sine of
    /// amplitude A has rms A / sqrt 2).
    pub rms: f64,
}

/// A frequency band of the rotors: `lo_hz` and `hi_hz` are already widened by [`BAND_MARGIN`].
#[derive(Clone, Debug, PartialEq)]
pub struct RotorBand {
    pub lo_hz: f64,
    pub hi_hz: f64,
    /// The band reaches above half the sample rate: it is seen folded.
    pub folded: bool,
    /// The ends as seen in the spectrum: `|f - fs x round(f / fs)|` when folded, else the ends.
    pub shown_lo_hz: f64,
    pub shown_hi_hz: f64,
}

/// The averaged spectrum of one axis.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisSpectrum {
    /// The largest peaks, largest first.
    pub peaks: Vec<Peak>,
    /// Share of the total variance within the rotor band and within twice the band; `None` without a band.
    pub band_share: Option<[f64; 2]>,
}

/// The Welch spectra of an interval: gyro X, Y, Z, then accelerometer X, Y, Z.
#[derive(Clone, Debug, PartialEq)]
pub struct Spectra {
    pub segments: u32,
    pub axes: [AxisSpectrum; 6],
    /// The rotor band and twice the rotor band; `None` when the interval has no rotor speed.
    pub bands: Option<[RotorBand; 2]>,
}

/// The shares of accelerometer samples whose magnitude is within the 1 g window.
#[derive(Clone, Debug, PartialEq)]
pub struct OneGShares {
    pub raw: f64,
    /// Per cut-off (Hz), the share after the first-order low-pass.
    pub filtered: Vec<(f64, f64)>,
}

/// The vibration figures of one interval.
#[derive(Clone, Debug, PartialEq)]
pub struct IntervalVibration {
    pub kind: IntervalKind,
    /// The armed window, from 0.
    pub window: usize,
    /// Of its kind in the window, from 1.
    pub number: usize,
    pub start_us: u64,
    pub end_us: u64,
    pub samples: u64,
    /// The median spacing of the samples, as a rate.
    pub rate_hz: Option<f64>,
    pub too_slow: bool,
    pub gyro_std: [f64; 3],
    pub accel_std: [f64; 3],
    pub accel_mag_std: f64,
    /// Lowest and highest of the four motors' mean rotor frequency (Hz).
    pub rotor_hz: Option<(f64, f64)>,
    /// `None`: too slow, or fewer samples than one segment.
    pub spectra: Option<Spectra>,
    pub one_g: Option<OneGShares>,
}

/// Why the vibration section has, or has not, figures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VibrationState {
    Done,
    NeedsPoles,
    NoImu,
}

/// The vibration section.
#[derive(Clone, Debug, PartialEq)]
pub struct VibrationReport {
    pub state: VibrationState,
    pub intervals: Vec<IntervalVibration>,
}

/// An interval to measure, chosen from the bins before the IMU pass.
#[derive(Clone, Debug)]
pub(crate) struct Spec {
    pub kind: IntervalKind,
    pub window: usize,
    pub number: usize,
    pub start_us: u64,
    pub end_us: u64,
    pub rotor_hz: Option<(f64, f64)>,
}

#[derive(Clone, Default)]
struct Welford {
    n: u64,
    mean: f64,
    m2: f64,
}

impl Welford {
    fn add(&mut self, x: f64) {
        self.n += 1;
        let d = x - self.mean;
        self.mean += d / self.n as f64;
        self.m2 += d * (x - self.mean);
    }

    fn std(&self) -> f64 {
        if self.n > 0 {
            (self.m2 / self.n as f64).sqrt()
        } else {
            0.0
        }
    }
}

struct Lpf {
    cutoff: f64,
    y: [f64; 3],
    inside: u64,
}

struct Active {
    index: usize,
    n: u64,
    last_t: Option<u64>,
    spacing: BTreeMap<u64, u64>,
    stats: [Welford; 7],
    buf: Vec<[f64; 6]>,
    psd: [Vec<f64>; 6],
    segments: u32,
    raw_inside: u64,
    lpfs: Vec<Lpf>,
}

impl Active {
    fn new(index: usize, cutoffs: &[f64]) -> Active {
        Active {
            index,
            n: 0,
            last_t: None,
            spacing: BTreeMap::new(),
            stats: Default::default(),
            buf: Vec::with_capacity(SEGMENT),
            psd: std::array::from_fn(|_| vec![0.0; SEGMENT / 2 + 1]),
            segments: 0,
            raw_inside: 0,
            lpfs: cutoffs
                .iter()
                .map(|c| Lpf {
                    cutoff: *c,
                    y: [0.0; 3],
                    inside: 0,
                })
                .collect(),
        }
    }
}

/// Measures the intervals while the IMU records go by.
pub(crate) struct Runner {
    specs: Vec<Spec>,
    cutoffs: Vec<f64>,
    window: Vec<f64>,
    window_energy: f64,
    twiddles: Vec<(f64, f64)>,
    active: Option<Active>,
    done: Vec<Option<IntervalVibration>>,
}

fn in_window(m: [f64; 3]) -> bool {
    let mag = (m[0] * m[0] + m[1] * m[1] + m[2] * m[2]).sqrt();
    mag >= ONE_G_WINDOW.0 * GRAVITY && mag <= ONE_G_WINDOW.1 * GRAVITY
}

/// `|f - fs x round(f / fs)|`.
fn fold(f: f64, fs: f64) -> f64 {
    (f - fs * (f / fs).round()).abs()
}

/// Whether a signal in `[lo, hi]` Hz shows at `f` Hz when sampled at `fs`.
fn seen_at(f: f64, lo: f64, hi: f64, fs: f64) -> bool {
    let last = (hi / fs).ceil() as i64 + 1;
    (0..=last).any(|n| {
        let base = n as f64 * fs;
        let (a, b) = (base + f, base - f);
        (a >= lo && a <= hi) || (b >= lo && b <= hi)
    })
}

impl Runner {
    pub fn new(specs: Vec<Spec>, cutoffs: Vec<f64>) -> Runner {
        let window: Vec<f64> = (0..SEGMENT)
            .map(|i| 0.5 - 0.5 * (TAU * i as f64 / SEGMENT as f64).cos())
            .collect();
        let window_energy = window.iter().map(|w| w * w).sum();
        let twiddles = (0..SEGMENT / 2)
            .map(|k| {
                let a = -TAU * k as f64 / SEGMENT as f64;
                (a.cos(), a.sin())
            })
            .collect();
        let done = specs.iter().map(|_| None).collect();
        Runner {
            specs,
            cutoffs,
            window,
            window_energy,
            twiddles,
            active: None,
            done,
        }
    }

    /// One `IMU` record.
    pub fn feed(&mut self, r: &Record<'_>) {
        // A log without an instance column has one IMU.
        if r.number_of("I").unwrap_or(0.0) != 0.0 {
            return;
        }
        let Some(t) = r.time_us() else { return };
        let mut s = [0.0; 6];
        for (v, name) in s
            .iter_mut()
            .zip(["GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"])
        {
            let Some(x) = r.number_of(name) else { return };
            *v = x;
        }
        let at = self.specs.partition_point(|sp| sp.end_us <= t);
        if at >= self.specs.len() || self.specs[at].start_us > t {
            return;
        }
        if self.active.as_ref().is_none_or(|a| a.index != at) {
            self.finish_active();
            self.active = Some(Active::new(at, &self.cutoffs));
        }
        let Some(a) = self.active.as_mut() else {
            return;
        };
        let acc = [s[3], s[4], s[5]];
        let dt = a.last_t.map(|p| t.saturating_sub(p));
        if let Some(d) = dt {
            *a.spacing.entry(d).or_insert(0) += 1;
        }
        for (st, v) in a.stats.iter_mut().zip(s) {
            st.add(v);
        }
        a.stats[6].add((acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]).sqrt());
        a.raw_inside += u64::from(in_window(acc));
        for l in &mut a.lpfs {
            match dt {
                None => l.y = acc,
                Some(d) => {
                    let dt = d as f64 / 1e6;
                    let alpha = dt / (dt + 1.0 / (TAU * l.cutoff));
                    for (y, x) in l.y.iter_mut().zip(acc) {
                        *y += (x - *y) * alpha;
                    }
                }
            }
            l.inside += u64::from(in_window(l.y));
        }
        a.n += 1;
        a.last_t = Some(t);
        a.buf.push(s);
        if a.buf.len() == SEGMENT {
            Self::segment(a, &self.window, self.window_energy, &self.twiddles);
            a.buf.drain(..SEGMENT / 2);
        }
    }

    fn segment(a: &mut Active, window: &[f64], energy: f64, twiddles: &[(f64, f64)]) {
        let n = SEGMENT as f64;
        for axis in 0..6 {
            let mean = a.buf.iter().map(|s| s[axis]).sum::<f64>() / n;
            let mut re: Vec<f64> = a
                .buf
                .iter()
                .zip(window)
                .map(|(s, w)| (s[axis] - mean) * w)
                .collect();
            let mut im = vec![0.0; SEGMENT];
            fft(&mut re, &mut im, twiddles);
            for k in 0..=SEGMENT / 2 {
                let one_sided = if k == 0 || k == SEGMENT / 2 { 1.0 } else { 2.0 };
                a.psd[axis][k] += one_sided * (re[k] * re[k] + im[k] * im[k]) / (n * energy);
            }
        }
        a.segments += 1;
    }

    fn finish_active(&mut self) {
        let Some(a) = self.active.take() else { return };
        let spec = self.specs[a.index].clone();
        let mut median = None;
        let total: u64 = a.spacing.values().sum();
        let mut seen = 0;
        for (d, c) in &a.spacing {
            seen += c;
            if 2 * seen >= total {
                median = Some(*d);
                break;
            }
        }
        let rate_hz = median.filter(|d| *d > 0).map(|d| 1e6 / d as f64);
        let too_slow = rate_hz.is_some_and(|f| f < MIN_SPECTRUM_RATE_HZ);
        let spectra = match rate_hz {
            Some(fs) if !too_slow && a.segments > 0 => Some(spectra_of(&a, fs, spec.rotor_hz)),
            _ => None,
        };
        let nf = a.n as f64;
        self.done[a.index] = Some(IntervalVibration {
            kind: spec.kind,
            window: spec.window,
            number: spec.number,
            start_us: spec.start_us,
            end_us: spec.end_us,
            samples: a.n,
            rate_hz,
            too_slow,
            gyro_std: [a.stats[0].std(), a.stats[1].std(), a.stats[2].std()],
            accel_std: [a.stats[3].std(), a.stats[4].std(), a.stats[5].std()],
            accel_mag_std: a.stats[6].std(),
            rotor_hz: spec.rotor_hz,
            spectra,
            one_g: (a.n > 0).then(|| OneGShares {
                raw: a.raw_inside as f64 / nf,
                filtered: a
                    .lpfs
                    .iter()
                    .map(|l| (l.cutoff, l.inside as f64 / nf))
                    .collect(),
            }),
        });
    }

    /// The figures of every interval, in the order of the specs.
    pub fn finish(mut self) -> Vec<IntervalVibration> {
        self.finish_active();
        self.specs
            .iter()
            .zip(self.done)
            .map(|(spec, d)| {
                d.unwrap_or(IntervalVibration {
                    kind: spec.kind,
                    window: spec.window,
                    number: spec.number,
                    start_us: spec.start_us,
                    end_us: spec.end_us,
                    samples: 0,
                    rate_hz: None,
                    too_slow: false,
                    gyro_std: [0.0; 3],
                    accel_std: [0.0; 3],
                    accel_mag_std: 0.0,
                    rotor_hz: spec.rotor_hz,
                    spectra: None,
                    one_g: None,
                })
            })
            .collect()
    }
}

fn spectra_of(a: &Active, fs: f64, rotor_hz: Option<(f64, f64)>) -> Spectra {
    let segs = f64::from(a.segments);
    let df = fs / SEGMENT as f64;
    let half = fs / 2.0;
    let bands = rotor_hz.map(|(lo, hi)| {
        [1.0, 2.0].map(|m| {
            let (l, h) = (m * lo * (1.0 - BAND_MARGIN), m * hi * (1.0 + BAND_MARGIN));
            let folded = h > half;
            RotorBand {
                lo_hz: l,
                hi_hz: h,
                folded,
                shown_lo_hz: if folded { fold(l, fs) } else { l },
                shown_hi_hz: if folded { fold(h, fs) } else { h },
            }
        })
    });
    let axes = std::array::from_fn(|axis| {
        let s: Vec<f64> = a.psd[axis].iter().map(|v| v / segs).collect();
        let mut peaks: Vec<usize> = (1..SEGMENT / 2)
            .filter(|&k| s[k] > s[k - 1] && s[k] >= s[k + 1])
            .collect();
        peaks.sort_by(|x, y| s[*y].total_cmp(&s[*x]));
        let peaks = peaks
            .into_iter()
            .take(PEAKS)
            .map(|k| {
                let from = k.saturating_sub(PEAK_HALF_WIDTH);
                let to = (k + PEAK_HALF_WIDTH).min(SEGMENT / 2);
                Peak {
                    hz: k as f64 * df,
                    rms: s[from..=to].iter().sum::<f64>().sqrt(),
                }
            })
            .collect();
        let total: f64 = s.iter().sum();
        let band_share = bands.as_ref().map(|b| {
            b.each_ref().map(|band| {
                let inside: f64 = s
                    .iter()
                    .enumerate()
                    .filter(|(k, _)| seen_at(*k as f64 * df, band.lo_hz, band.hi_hz, fs))
                    .map(|(_, v)| v)
                    .sum();
                if total > 0.0 { inside / total } else { 0.0 }
            })
        });
        AxisSpectrum { peaks, band_share }
    });
    Spectra {
        segments: a.segments,
        axes,
        bands,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn the_fft_matches_the_definition() {
        let n = 16;
        let x: Vec<f64> = (0..n)
            .map(|i| ((i * 7 % 5) as f64).sin() + i as f64 * 0.1)
            .collect();
        let mut re = x.clone();
        let mut im = vec![0.0; n];
        fft_in_place(&mut re, &mut im);
        for k in 0..n {
            let (mut sr, mut si) = (0.0, 0.0);
            for (j, v) in x.iter().enumerate() {
                let a = -2.0 * PI * (j * k) as f64 / n as f64;
                sr += v * a.cos();
                si += v * a.sin();
            }
            assert!(
                (re[k] - sr).abs() < 1e-9 && (im[k] - si).abs() < 1e-9,
                "bin {k}"
            );
        }
    }

    #[test]
    fn folding_and_visibility() {
        assert!((fold(700.0, 1000.0) - 300.0).abs() < 1e-12);
        assert!((fold(1100.0, 1000.0) - 100.0).abs() < 1e-12);
        assert!(seen_at(300.0, 650.0, 750.0, 1000.0));
        assert!(!seen_at(100.0, 650.0, 750.0, 1000.0));
        assert!(seen_at(100.0, 1050.0, 1150.0, 1000.0));
    }
}
