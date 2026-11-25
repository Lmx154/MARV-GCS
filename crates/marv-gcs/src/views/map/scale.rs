//! The scale bar in the map's bottom-left corner.

/// Metres on the ground that one pixel of the map covers, at latitude `lat` (degrees) and the map zoom `zoom`
/// (the whole world is 256 x 2^zoom pixels wide, as walkers draws it).
pub fn metres_per_pixel(lat: f64, zoom: f64) -> f64 {
    40_075_016.686 * lat.to_radians().cos() / (256.0 * 2f64.powf(zoom))
}

/// The longest round length (1, 2 or 5 times a power of ten, in metres) that is at most `max_pixels` wide, with
/// its width in pixels; `None` when the scale is not a number above zero.
pub fn bar(metres_per_pixel: f64, max_pixels: f32) -> Option<(f64, f32)> {
    if !(metres_per_pixel.is_finite() && metres_per_pixel > 0.0) {
        return None;
    }
    let longest = f64::from(max_pixels) * metres_per_pixel;
    let power = 10f64.powf(longest.log10().floor());
    let metres = [5.0, 2.0, 1.0]
        .into_iter()
        .map(|m| m * power)
        .find(|&m| m <= longest)?;
    Some((metres, (metres / metres_per_pixel) as f32))
}

/// "5 cm", "20 m", "2 km".
pub fn label(metres: f64) -> String {
    if metres >= 1000.0 {
        format!("{} km", (metres / 1000.0).round())
    } else if metres >= 1.0 {
        format!("{} m", metres.round())
    } else {
        format!("{} cm", (metres * 100.0).round())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_scale_at_the_equator_is_156543_metres_a_pixel_at_zoom_0() {
        assert!((metres_per_pixel(0.0, 0.0) - 156_543.03).abs() < 0.01);
    }

    #[test]
    fn at_the_deepest_zoom_a_metre_spans_hundreds_of_pixels_and_at_zoom_22_tens() {
        let at = |zoom| 1.0 / metres_per_pixel(26.19, zoom);
        assert!(at(26.0) > 400.0, "{}", at(26.0));
        assert!(at(22.0) > 25.0, "{}", at(22.0));
    }

    #[test]
    fn the_bar_is_a_round_length_within_the_width() {
        for (mpp, metres, text) in [
            (0.002, 0.2, "20 cm"),
            (0.01, 1.0, "1 m"),
            (0.5, 50.0, "50 m"),
            (1.0, 100.0, "100 m"),
            (30.0, 2000.0, "2 km"),
        ] {
            let (m, px) = bar(mpp, 140.0).unwrap();
            assert!((m - metres).abs() < 1e-9, "{mpp}: {m}");
            assert!(px <= 140.0 && px > 140.0 / 2.5, "{mpp}: {px}");
            assert_eq!(label(m), text);
        }
    }

    #[test]
    fn a_scale_that_is_not_a_number_has_no_bar() {
        assert_eq!(bar(0.0, 140.0), None);
        assert_eq!(bar(f64::NAN, 140.0), None);
        assert_eq!(bar(-1.0, 140.0), None);
    }
}
