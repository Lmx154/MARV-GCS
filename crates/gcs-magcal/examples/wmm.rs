//! Prints the WMM2025 field for a place and date: `cargo run -p gcs-magcal --example wmm -- LAT LON HEIGHT_M YEAR`
//! (geodetic degrees, metres above the WGS-84 ellipsoid, decimal year).

fn main() {
    let args: Vec<f64> = std::env::args()
        .skip(1)
        .map(|a| a.parse().expect("a number"))
        .collect();
    let [lat, lon, height_m, year] = args[..] else {
        eprintln!("usage: wmm LAT_DEG LON_DEG HEIGHT_M DECIMAL_YEAR");
        std::process::exit(2);
    };
    match gcs_magcal::wmm::field(lat, lon, height_m, year) {
        Ok(f) => {
            println!("total intensity  {:.3} uT", f.intensity_ut);
            println!(
                "inclination      {:.3} deg (positive down)",
                f.inclination_deg
            );
            println!(
                "declination      {:.3} deg (positive east)",
                f.declination_deg
            );
            println!("horizontal       {:.3} uT", f.horizontal_ut);
            println!(
                "north east down  {:.3} {:.3} {:.3} uT",
                f.ned_ut[0], f.ned_ut[1], f.ned_ut[2]
            );
        }
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    }
}
