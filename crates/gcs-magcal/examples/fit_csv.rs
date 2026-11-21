//! Fits raw magnetometer samples from a CSV file: `cargo run -p gcs-magcal --example fit_csv -- FILE.csv [RADIUS_UT]`.
//! Each line holds x,y,z in uT (body frame, raw); a line that does not parse (a header) is skipped. RADIUS_UT is the
//! starting radius, normally the model's field strength; without it PX4's default 0.4 gauss = 40 uT
//! (`MAG_SPHERE_RADIUS_DEFAULT`, mag_calibration.cpp:74).

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(path) = args.first() else {
        eprintln!("usage: fit_csv FILE.csv [RADIUS_UT]");
        std::process::exit(2);
    };
    let radius: f64 = args
        .get(1)
        .map(|a| a.parse().expect("a number"))
        .unwrap_or(40.0);
    let text = std::fs::read_to_string(path).expect("readable file");
    let samples: Vec<[f64; 3]> = text
        .lines()
        .filter_map(|l| {
            let v: Vec<f64> = l
                .split(',')
                .map(|t| t.trim().parse().ok())
                .collect::<Option<_>>()?;
            (v.len() == 3).then(|| [v[0], v[1], v[2]])
        })
        .collect();
    let f = gcs_magcal::fit::fit(&samples, radius);
    println!("samples          {}", samples.len());
    println!(
        "converged        sphere {}, ellipsoid {}",
        f.sphere_converged, f.ellipsoid_converged
    );
    println!(
        "offsets o        {:.3} {:.3} {:.3} uT",
        f.offsets[0], f.offsets[1], f.offsets[2]
    );
    println!(
        "diagonal         {:.4} {:.4} {:.4}",
        f.diag[0], f.diag[1], f.diag[2]
    );
    println!(
        "off-diagonal     xy {:.4} xz {:.4} yz {:.4}",
        f.offdiag[0], f.offdiag[1], f.offdiag[2]
    );
    println!("radius           {:.3} uT", f.radius_ut);
    println!(
        "fitness          {:.3} uT RMS (limit {})",
        f.fitness_ut,
        gcs_magcal::accept::FITNESS_LIMIT_UT
    );
}
