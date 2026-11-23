//! The signal processing `log tune` needs, in f64 and with no dependency: a radix-2 FFT, Welch auto and cross
//! spectra (scipy `welch` and `csd` conventions: periodic Hann window, 50 % overlap, one-sided density), and small
//! helpers (median, percentile, interpolated peak).

use std::f64::consts::PI;

/// A radix-2 FFT plan for one power-of-two length.
pub struct Fft {
    n: usize,
    cos: Vec<f64>,
    sin: Vec<f64>,
    rev: Vec<usize>,
}

impl Fft {
    /// `n` must be a power of two and at least 2.
    pub fn new(n: usize) -> Fft {
        assert!(
            n >= 2 && n.is_power_of_two(),
            "FFT length {n} is not a power of two"
        );
        let bits = n.trailing_zeros();
        let rev = (0..n)
            .map(|i| i.reverse_bits() >> (usize::BITS - bits))
            .collect();
        let cos = (0..n / 2)
            .map(|k| (2.0 * PI * k as f64 / n as f64).cos())
            .collect();
        let sin = (0..n / 2)
            .map(|k| (2.0 * PI * k as f64 / n as f64).sin())
            .collect();
        Fft { n, cos, sin, rev }
    }

    /// In place: `X[k] = sum_n x[n] exp(-2 pi i k n / N)`.
    pub fn forward(&self, re: &mut [f64], im: &mut [f64]) {
        assert!(re.len() == self.n && im.len() == self.n);
        for i in 0..self.n {
            let j = self.rev[i];
            if j > i {
                re.swap(i, j);
                im.swap(i, j);
            }
        }
        let mut len = 2;
        while len <= self.n {
            let half = len / 2;
            let step = self.n / len;
            for start in (0..self.n).step_by(len) {
                for k in 0..half {
                    let (wr, wi) = (self.cos[k * step], -self.sin[k * step]);
                    let (a, b) = (start + k, start + k + half);
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
}

/// What each Welch segment has removed before the window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Detrend {
    /// The segment mean (scipy's default).
    Constant,
    /// The least-squares line.
    Linear,
}

/// Welch accumulator for one signal (`psd`) or a pair (`cross`): segments of `nperseg`, hop `nperseg / 2`, a periodic
/// Hann window, never across the end of a chunk handed to `add`.
pub struct Welch {
    fft: Fft,
    n: usize,
    window: Vec<f64>,
    scale: f64,
    detrend: Detrend,
    nseg: usize,
    sxx: Vec<f64>,
    syy: Vec<f64>,
    sxy_re: Vec<f64>,
    sxy_im: Vec<f64>,
}

/// The averaged one-sided spectra, bin k at `k * df`.
pub struct Spectra {
    pub df: f64,
    pub nseg: usize,
    pub sxx: Vec<f64>,
    pub syy: Vec<f64>,
    /// Cross spectrum `E[conj(X) Y]`: the transfer from x to y is `sxy / sxx`.
    pub sxy: Vec<(f64, f64)>,
}

impl Welch {
    pub fn new(nperseg: usize, fs: f64, detrend: Detrend) -> Welch {
        let window: Vec<f64> = (0..nperseg)
            .map(|i| 0.5 - 0.5 * (2.0 * PI * i as f64 / nperseg as f64).cos())
            .collect();
        let power: f64 = window.iter().map(|w| w * w).sum();
        let bins = nperseg / 2 + 1;
        Welch {
            fft: Fft::new(nperseg),
            n: nperseg,
            window,
            scale: 1.0 / (fs * power),
            detrend,
            nseg: 0,
            sxx: vec![0.0; bins],
            syy: vec![0.0; bins],
            sxy_re: vec![0.0; bins],
            sxy_im: vec![0.0; bins],
        }
    }

    /// The number of whole segments `len` samples hold.
    pub fn segments_in(&self, len: usize) -> usize {
        if len < self.n {
            0
        } else {
            (len - self.n) / (self.n / 2) + 1
        }
    }

    fn transform(&self, x: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut re: Vec<f64> = x.to_vec();
        match self.detrend {
            Detrend::Constant => {
                let mean = re.iter().sum::<f64>() / self.n as f64;
                re.iter_mut().for_each(|v| *v -= mean);
            }
            Detrend::Linear => {
                let n = self.n as f64;
                let tm = (n - 1.0) / 2.0;
                let ym = re.iter().sum::<f64>() / n;
                let (mut sty, mut stt) = (0.0, 0.0);
                for (i, v) in re.iter().enumerate() {
                    let dt = i as f64 - tm;
                    sty += dt * (v - ym);
                    stt += dt * dt;
                }
                let slope = sty / stt;
                for (i, v) in re.iter_mut().enumerate() {
                    *v -= ym + slope * (i as f64 - tm);
                }
            }
        }
        re.iter_mut().zip(&self.window).for_each(|(v, w)| *v *= w);
        let mut im = vec![0.0; self.n];
        self.fft.forward(&mut re, &mut im);
        (re, im)
    }

    /// Adds every whole segment of the chunk; `y` is `None` for an auto spectrum.
    pub fn add(&mut self, x: &[f64], y: Option<&[f64]>) {
        if let Some(y) = y {
            assert_eq!(x.len(), y.len());
        }
        let hop = self.n / 2;
        let mut start = 0;
        while start + self.n <= x.len() {
            let (xr, xi) = self.transform(&x[start..start + self.n]);
            let yt = y.map(|y| self.transform(&y[start..start + self.n]));
            for k in 0..=self.n / 2 {
                self.sxx[k] += xr[k] * xr[k] + xi[k] * xi[k];
                if let Some((yr, yi)) = &yt {
                    self.syy[k] += yr[k] * yr[k] + yi[k] * yi[k];
                    // conj(X) * Y
                    self.sxy_re[k] += xr[k] * yr[k] + xi[k] * yi[k];
                    self.sxy_im[k] += xr[k] * yi[k] - xi[k] * yr[k];
                }
            }
            self.nseg += 1;
            start += hop;
        }
    }

    pub fn segments(&self) -> usize {
        self.nseg
    }

    /// The one-sided density spectra (interior bins doubled), or `None` with no segment.
    pub fn finish(&self, fs: f64) -> Option<Spectra> {
        if self.nseg == 0 {
            return None;
        }
        let m = self.nseg as f64;
        let bins = self.n / 2 + 1;
        let side = |k: usize| if k == 0 || k == self.n / 2 { 1.0 } else { 2.0 };
        let f = |v: f64, k: usize| v / m * self.scale * side(k);
        Some(Spectra {
            df: fs / self.n as f64,
            nseg: self.nseg,
            sxx: (0..bins).map(|k| f(self.sxx[k], k)).collect(),
            syy: (0..bins).map(|k| f(self.syy[k], k)).collect(),
            sxy: (0..bins)
                .map(|k| (f(self.sxy_re[k], k), f(self.sxy_im[k], k)))
                .collect(),
        })
    }
}

/// `sqrt(sum of density over bins at or above lo_hz x df)`: the rms of the part of the signal above `lo_hz`
/// (the "above 50 Hz" metric of the earlier scripts).
pub fn rms_above(psd: &[f64], df: f64, lo_hz: f64) -> f64 {
    let sum: f64 = psd
        .iter()
        .enumerate()
        .filter(|(k, _)| *k as f64 * df >= lo_hz)
        .map(|(_, p)| p)
        .sum();
    (sum * df).sqrt()
}

/// One spectral peak.
#[derive(Clone, Copy, Debug)]
pub struct Peak {
    pub bin: usize,
    /// Frequency by parabolic interpolation of the three bins around the maximum, Hz.
    pub hz: f64,
    /// rms of the bins within `half_bins` of the maximum (the earlier scripts' band rms).
    pub rms: f64,
}

/// The `n` highest local maxima at or above `min_hz`, no two closer than `sep_hz` (the higher wins), by height.
pub fn peaks(
    psd: &[f64],
    df: f64,
    min_hz: f64,
    n: usize,
    sep_hz: f64,
    half_bins: usize,
) -> Vec<Peak> {
    let mut cand: Vec<usize> = (1..psd.len().saturating_sub(1))
        .filter(|&k| k as f64 * df >= min_hz && psd[k] > psd[k - 1] && psd[k] >= psd[k + 1])
        .collect();
    cand.sort_by(|a, b| psd[*b].total_cmp(&psd[*a]));
    let sep = (sep_hz / df).max(1.0);
    let mut kept: Vec<usize> = Vec::new();
    for k in cand {
        if kept.iter().all(|&j| (j as f64 - k as f64).abs() >= sep) {
            kept.push(k);
        }
        if kept.len() == n {
            break;
        }
    }
    kept.into_iter()
        .map(|k| {
            let (a, b, c) = (psd[k - 1], psd[k], psd[k + 1]);
            let den = a - 2.0 * b + c;
            let delta = if den < 0.0 {
                (0.5 * (a - c) / den).clamp(-0.5, 0.5)
            } else {
                0.0
            };
            let lo = k.saturating_sub(half_bins);
            let hi = (k + half_bins + 1).min(psd.len());
            let band: f64 = psd[lo..hi].iter().sum();
            Peak {
                bin: k,
                hz: (k as f64 + delta) * df,
                rms: (band * df).sqrt(),
            }
        })
        .collect()
}

/// The median (the mean of the two middle values for an even count); `None` for an empty slice.
pub fn median(values: &[f64]) -> Option<f64> {
    percentile(values, 50.0)
}

/// The `pct` percentile by linear interpolation between order statistics (numpy's default); `None` when empty.
pub fn percentile(values: &[f64], pct: f64) -> Option<f64> {
    let mut v: Vec<f64> = values.iter().copied().filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return None;
    }
    v.sort_by(f64::total_cmp);
    let pos = pct.clamp(0.0, 100.0) / 100.0 * (v.len() - 1) as f64;
    let (lo, hi) = (pos.floor() as usize, pos.ceil() as usize);
    Some(v[lo] + (v[hi] - v[lo]) * (pos - lo as f64))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A deterministic generator (xorshift64*), uniform in [0, 1).
    pub struct Rng(pub u64);
    impl Rng {
        pub fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
        }
        pub fn normal(&mut self) -> f64 {
            let (u, v) = (self.uniform().max(1e-300), self.uniform());
            (-2.0 * u.ln()).sqrt() * (2.0 * PI * v).cos()
        }
    }

    fn direct_dft(x: &[f64], y: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = x.len();
        let (mut re, mut im) = (vec![0.0; n], vec![0.0; n]);
        for k in 0..n {
            for j in 0..n {
                let a = -2.0 * PI * ((k * j) % n) as f64 / n as f64;
                re[k] += x[j] * a.cos() - y[j] * a.sin();
                im[k] += x[j] * a.sin() + y[j] * a.cos();
            }
        }
        (re, im)
    }

    #[test]
    fn fft_matches_a_direct_dft_to_1e_9() {
        let mut rng = Rng(0x1234_5678_9abc_def1);
        for n in [2usize, 8, 64, 256, 1024] {
            let x: Vec<f64> = (0..n).map(|_| rng.normal() * 3.0).collect();
            let y: Vec<f64> = (0..n).map(|_| rng.normal() * 3.0).collect();
            let (dr, di) = direct_dft(&x, &y);
            let (mut re, mut im) = (x.clone(), y.clone());
            Fft::new(n).forward(&mut re, &mut im);
            let worst = (0..n)
                .map(|k| (re[k] - dr[k]).abs().max((im[k] - di[k]).abs()))
                .fold(0.0, f64::max);
            assert!(worst < 1e-9, "n = {n}: largest difference {worst:e}");
        }
    }

    #[test]
    fn welch_density_integrates_to_the_variance_of_white_noise() {
        let mut rng = Rng(42);
        let fs = 803.86;
        let sigma = 0.7;
        let x: Vec<f64> = (0..1 << 17).map(|_| rng.normal() * sigma).collect();
        let var = x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64
            - (x.iter().sum::<f64>() / x.len() as f64).powi(2);
        for detrend in [Detrend::Constant, Detrend::Linear] {
            let mut w = Welch::new(2048, fs, detrend);
            w.add(&x, None);
            let s = w.finish(fs).expect("segments");
            let integral: f64 = s.sxx.iter().sum::<f64>() * s.df;
            let err = (integral / var - 1.0).abs();
            assert!(
                err < 0.02,
                "{detrend:?}: integral {integral}, variance {var}, error {err}"
            );
        }
    }

    #[test]
    fn welch_cross_spectrum_recovers_a_known_gain_and_delay() {
        // y = 0.5 x delayed by 8 samples: |H| = 0.5, phase = -2 pi f 8 / fs at every bin, coherence 1.
        let mut rng = Rng(7);
        let fs = 800.0;
        let x: Vec<f64> = (0..1 << 14).map(|_| rng.normal()).collect();
        let y: Vec<f64> = (0..x.len())
            .map(|i| if i >= 8 { 0.5 * x[i - 8] } else { 0.0 })
            .collect();
        let mut w = Welch::new(1024, fs, Detrend::Constant);
        w.add(&x, Some(&y));
        let s = w.finish(fs).expect("segments");
        let k = 40;
        let (re, im) = (s.sxy[k].0 / s.sxx[k], s.sxy[k].1 / s.sxx[k]);
        let f = k as f64 * s.df;
        assert!((re.hypot(im) - 0.5).abs() < 0.01, "gain {}", re.hypot(im));
        let want = -2.0 * PI * f * 8.0 / fs;
        assert!(
            (im.atan2(re) - want).abs() < 0.02,
            "phase {} want {want}",
            im.atan2(re)
        );
    }

    #[test]
    fn peaks_percentile_and_median() {
        assert_eq!(median(&[3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median(&[1.0, 2.0, 3.0, 4.0]), Some(2.5));
        assert_eq!(median(&[]), None);
        assert!((percentile(&[0.0, 10.0], 25.0).unwrap() - 2.5).abs() < 1e-12);
        let fs = 800.0;
        let x: Vec<f64> = (0..8192)
            .map(|i| (2.0 * PI * 123.4 * i as f64 / fs).sin())
            .collect();
        let mut w = Welch::new(2048, fs, Detrend::Constant);
        w.add(&x, None);
        let s = w.finish(fs).unwrap();
        let p = peaks(&s.sxx, s.df, 20.0, 2, 4.0, 2);
        assert!((p[0].hz - 123.4).abs() < s.df, "{}", p[0].hz);
    }
}
