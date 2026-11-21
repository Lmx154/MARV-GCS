//! The 3D quad of the Compass panel (decision `decision-compass-cal-20261120.md`, section 8): two arms, four rotor
//! circles as 16-gons, a red nose marker and a curved arrow for the turn, projected orthographically and sorted back
//! to front. Pure: a down vector in, 2D polygons out; [`draw`] only paints them.
//!
//! Frames. The body is FRD (x forward, y right, z down), units of one arm half-length. The world has x ahead (the
//! heading the quad is drawn with), y to the right and z down; the pose only fixes where world down lies in the
//! body, the heading is chosen so that the body's forward axis, projected on the horizontal, points ahead (for nose
//! up and nose down, where it is vertical, the body's down axis is used instead). The camera is south of the quad
//! and [`ELEVATION_DEG`] above the horizon, so ahead is up the screen and right is right.

use crate::theme;
use eframe::egui::{Color32, Painter, Pos2, Rect, Shape, Stroke};

/// Camera elevation above the horizon, degrees (a display choice).
pub const ELEVATION_DEG: f64 = 35.0;

const ARM_HALF: f64 = 0.7;
const ARM_WIDTH: f64 = 0.07;
const ROTOR_RADIUS: f64 = 0.32;
const ROTOR_SIDES: usize = 16;
/// A rotor sits this far above the arms (body z is down, so negative).
const ROTOR_Z: f64 = -0.06;
const NOSE_Z: f64 = -0.1;
const ARROW_RADIUS: f64 = 1.2;
const ARROW_WIDTH: f64 = 0.08;
const ARROW_STEPS: usize = 20;

/// What a polygon is, for its colour.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    Arm,
    Rotor,
    Nose,
    Arrow,
}

/// One filled polygon in screen units (x right, y down, the quad about 2.5 wide) and its depth (larger is nearer).
#[derive(Clone, Debug, PartialEq)]
pub struct Poly {
    pub part: Part,
    pub points: Vec<[f32; 2]>,
    pub depth: f32,
}

type V3 = [f64; 3];

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn unit(a: V3) -> Option<V3> {
    let n = dot(a, a).sqrt();
    (n > 1e-9 && n.is_finite()).then(|| [a[0] / n, a[1] / n, a[2] / n])
}

/// The unit down vector in the body from an accelerometer reading (specific force points up, so down is minus
/// its direction); `None` when the reading has no direction.
pub fn down_from_accel(accel: [f32; 3]) -> Option<[f64; 3]> {
    let a = accel.map(f64::from);
    unit([-a[0], -a[1], -a[2]])
}

/// The world axes in body coordinates for a body-frame down vector: (ahead, right, down).
fn world_axes(down: V3) -> Option<[V3; 3]> {
    let d = unit(down)?;
    let project = |v: V3| {
        let k = dot(v, d);
        unit([v[0] - k * d[0], v[1] - k * d[1], v[2] - k * d[2]])
    };
    let ahead = project([1.0, 0.0, 0.0]).or_else(|| project([0.0, 0.0, -1.0]))?;
    Some([ahead, cross(d, ahead), d])
}

/// The polygons of the quad whose body-frame down vector is `down`, farthest first; with `arrow` the curved arrow
/// of the turn about the vertical (fixed in the world, not in the body) is included. Empty for a `down` with no
/// direction.
pub fn quad(down: [f64; 3], arrow: bool) -> Vec<Poly> {
    let Some(axes) = world_axes(down) else {
        return Vec::new();
    };
    let (sin_e, cos_e) = ELEVATION_DEG.to_radians().sin_cos();
    // World (ahead, right, down) to the screen (right, down) and the depth.
    let project = |w: V3| -> ([f32; 2], f32) {
        let up = -w[2];
        let x = w[1];
        let y = -(up * cos_e + w[0] * sin_e);
        let depth = -w[0] * cos_e + up * sin_e;
        ([x as f32, y as f32], depth as f32)
    };
    let from_body = |p: V3| project([dot(p, axes[0]), dot(p, axes[1]), dot(p, axes[2])]);
    let polygon = |part: Part, world: bool, points: &[V3]| {
        let projected: Vec<([f32; 2], f32)> = points
            .iter()
            .map(|&p| if world { project(p) } else { from_body(p) })
            .collect();
        let depth = projected.iter().map(|p| p.1).sum::<f32>() / projected.len() as f32;
        Poly {
            part,
            points: projected.into_iter().map(|p| p.0).collect(),
            depth,
        }
    };

    let mut polys = Vec::new();
    // Two arms, along the diagonals of an X frame.
    for sign in [1.0, -1.0] {
        let along = unit([1.0, sign, 0.0]).unwrap_or([1.0, 0.0, 0.0]);
        let across = [-along[1], along[0], 0.0];
        let (l, w) = (ARM_HALF * 2f64.sqrt(), ARM_WIDTH);
        let corner = |a: f64, b: f64| {
            [
                a * along[0] + b * across[0],
                a * along[1] + b * across[1],
                0.0,
            ]
        };
        polys.push(polygon(
            Part::Arm,
            false,
            &[corner(-l, -w), corner(l, -w), corner(l, w), corner(-l, w)],
        ));
    }
    // Four rotor circles at the ends of the arms.
    for (sx, sy) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
        let ring: Vec<V3> = (0..ROTOR_SIDES)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / ROTOR_SIDES as f64;
                [
                    sx * ARM_HALF + ROTOR_RADIUS * t.cos(),
                    sy * ARM_HALF + ROTOR_RADIUS * t.sin(),
                    ROTOR_Z,
                ]
            })
            .collect();
        polys.push(polygon(Part::Rotor, false, &ring));
    }
    // The nose marker, a triangle ahead of the centre.
    polys.push(polygon(
        Part::Nose,
        false,
        &[
            [1.15, 0.0, NOSE_Z],
            [0.8, 0.14, NOSE_Z],
            [0.8, -0.14, NOSE_Z],
        ],
    ));
    if arrow {
        // An arc about the vertical, clockwise seen from above, from 40 to 300 degrees from ahead, with a head.
        let (from, to) = (40f64.to_radians(), 300f64.to_radians());
        let at = |t: f64, r: f64| [r * t.cos(), r * t.sin(), 0.0];
        let mut band: Vec<V3> = Vec::new();
        for i in 0..=ARROW_STEPS {
            let t = from + (to - from) * i as f64 / ARROW_STEPS as f64;
            band.push(at(t, ARROW_RADIUS + ARROW_WIDTH));
        }
        for i in (0..=ARROW_STEPS).rev() {
            let t = from + (to - from) * i as f64 / ARROW_STEPS as f64;
            band.push(at(t, ARROW_RADIUS - ARROW_WIDTH));
        }
        polys.push(polygon(Part::Arrow, true, &band));
        let tip = to + 14f64.to_radians();
        polys.push(polygon(
            Part::Arrow,
            true,
            &[
                at(to, ARROW_RADIUS + 3.0 * ARROW_WIDTH),
                at(tip, ARROW_RADIUS),
                at(to, ARROW_RADIUS - 3.0 * ARROW_WIDTH),
            ],
        ));
    }
    polys.sort_by(|a, b| a.depth.total_cmp(&b.depth));
    polys
}

/// Paints `polys` into `rect`, centred, `scale` pixels per screen unit; `ghost` draws them all in grey.
pub fn draw(painter: &Painter, rect: Rect, scale: f32, polys: &[Poly], ghost: bool) {
    let centre = rect.center();
    for poly in polys {
        let (fill, edge) = if ghost {
            (
                Color32::from_rgba_unmultiplied(150, 150, 150, 70),
                Color32::from_rgba_unmultiplied(170, 170, 170, 110),
            )
        } else {
            match poly.part {
                Part::Arm => (theme::BORDER_STRONG, theme::INSET),
                Part::Rotor => (theme::ACCENT, theme::INSET),
                Part::Nose => (theme::ERROR, theme::INSET),
                Part::Arrow => (theme::WARN, theme::INSET),
            }
        };
        let points: Vec<Pos2> = poly
            .points
            .iter()
            .map(|p| Pos2::new(centre.x + p[0] * scale, centre.y + p[1] * scale))
            .collect();
        painter.add(Shape::convex_polygon(points, fill, Stroke::new(1.0, edge)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEVEL: V3 = [0.0, 0.0, 1.0];
    const UPSIDE_DOWN: V3 = [0.0, 0.0, -1.0];

    fn centroid(polys: &[Poly], part: Part) -> [f32; 2] {
        let points: Vec<[f32; 2]> = polys
            .iter()
            .filter(|p| p.part == part)
            .flat_map(|p| p.points.iter().copied())
            .collect();
        let n = points.len() as f32;
        [
            points.iter().map(|p| p[0]).sum::<f32>() / n,
            points.iter().map(|p| p[1]).sum::<f32>() / n,
        ]
    }

    #[test]
    fn level_puts_the_nose_ahead_of_the_centre_up_the_screen() {
        let polys = quad(LEVEL, false);
        let nose = centroid(&polys, Part::Nose);
        let all: Vec<[f32; 2]> = polys
            .iter()
            .flat_map(|p| p.points.iter().copied())
            .collect();
        let centre_y = all.iter().map(|p| p[1]).sum::<f32>() / all.len() as f32;
        assert!(nose[1] < centre_y - 0.3, "{nose:?} {centre_y}");
        assert!(nose[0].abs() < 1e-6);
    }

    #[test]
    fn upside_down_mirrors_left_and_right_and_keeps_the_nose_ahead() {
        let level = quad(LEVEL, false);
        let flipped = quad(UPSIDE_DOWN, false);
        // The red marker stays on the centre line and ahead; it is no longer the nearest-to-camera part of the
        // body in the same order: its depth relative to the rotors changes sign (the rotors are below the arms).
        assert!(centroid(&flipped, Part::Nose)[1] < 0.0);
        assert!(centroid(&flipped, Part::Nose)[0].abs() < 1e-6);
        let depth = |polys: &[Poly], part| {
            let all: Vec<f32> = polys
                .iter()
                .filter(|p| p.part == part)
                .map(|p| p.depth)
                .collect();
            all.iter().sum::<f32>() / all.len() as f32
        };
        assert!(depth(&level, Part::Rotor) > depth(&level, Part::Arm));
        assert!(depth(&flipped, Part::Rotor) < depth(&flipped, Part::Arm));
    }

    #[test]
    fn the_right_hand_rotors_swap_sides_when_upside_down() {
        // The rotor at body (+x, +y) is the first rotor polygon pushed; find it by its position instead of its order.
        let right_x = |down| {
            let axes = world_axes(down).unwrap();
            let p = [ARM_HALF, ARM_HALF, 0.0];
            dot(p, axes[1])
        };
        assert!(right_x(LEVEL) > 0.5);
        assert!(right_x(UPSIDE_DOWN) < -0.5);
    }

    #[test]
    fn every_polygon_of_every_pose_is_finite_and_sorted_back_to_front() {
        for down in [
            LEVEL,
            UPSIDE_DOWN,
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.3, -0.2, 0.9],
        ] {
            for arrow in [false, true] {
                let polys = quad(down, arrow);
                assert_eq!(polys.iter().any(|p| p.part == Part::Arrow), arrow);
                assert_eq!(polys.iter().filter(|p| p.part == Part::Rotor).count(), 4);
                assert_eq!(polys.iter().filter(|p| p.part == Part::Arm).count(), 2);
                for p in &polys {
                    assert!(p.depth.is_finite());
                    assert!(p.points.len() >= 3);
                    assert!(
                        p.points
                            .iter()
                            .all(|q| q[0].is_finite() && q[1].is_finite())
                    );
                    if p.part == Part::Rotor {
                        assert_eq!(p.points.len(), ROTOR_SIDES);
                    }
                }
                assert!(polys.windows(2).all(|w| w[0].depth <= w[1].depth));
            }
        }
    }

    #[test]
    fn a_down_vector_without_a_direction_draws_nothing() {
        assert!(quad([0.0; 3], true).is_empty());
        assert!(quad([f64::NAN, 0.0, 1.0], true).is_empty());
        assert_eq!(down_from_accel([0.0; 3]), None);
        let d = down_from_accel([0.0, 0.0, -9.81]).unwrap();
        assert!((d[2] - 1.0).abs() < 1e-6);
    }
}
