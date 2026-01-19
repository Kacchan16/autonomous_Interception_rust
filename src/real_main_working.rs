use plotters::prelude::*;
use rand::Rng;

#[derive(Debug, Clone, Copy)]
pub struct Target {
    // Position in meters
    x: f64,
    y: f64,
    // Velocity components in m/s
    vx: f64,
    vy: f64,
}

impl Target {
    fn new(x: f64, y: f64, vx: f64, vy: f64) -> Self {
        Target { x, y, vx, vy }
    }

    fn update(&mut self, dt: f64) {
        self.x += self.vx * dt;
        self.y += self.vy * dt;
    }

    fn distance_to(&self, other: &Target) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

pub type Interceptor = Target;

// Solve for intercept time t in ||r + vt*t|| = vi*t
fn intercept_time(rx: f64, ry: f64, tvx: f64, tvy: f64, vi: f64) -> Option<f64> {
    // (vt·vt - vi^2) t^2 + 2(r·vt) t + (r·r) = 0
    let a = (tvx * tvx + tvy * tvy) - vi * vi;
    let b = 2.0 * (rx * tvx + ry * tvy);
    let c = rx * rx + ry * ry;

    // If a ~ 0, treat as linear
    if a.abs() < 1e-9 {
        if b.abs() < 1e-9 {
            return None;
        }
        let t = -c / b;
        if t > 0.0 { Some(t) } else { None }
    } else {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            None
        } else {
            let s = disc.sqrt();
            let t1 = (-b - s) / (2.0 * a);
            let t2 = (-b + s) / (2.0 * a);

            let mut best = None;
            for t in [t1, t2] {
                if t > 0.0 {
                    best = match best {
                        None => Some(t),
                        Some(cur) => Some(cur.min(t)),
                    };
                }
            }
            best
        }
    }
}

// Steering: lead to intercept point + lateral offset + terminal angle bias (but never lose LOS)
fn calculate_steering_direction(from: &Interceptor, to: &Target, interceptor_speed: f64) -> (f64, f64) {
    let rx = to.x - from.x;
    let ry = to.y - from.y;
    let r = (rx * rx + ry * ry).sqrt().max(1e-9);

    // Unit LOS (always closing)
    let lx = rx / r;
    let ly = ry / r;

    let tvx = to.vx;
    let tvy = to.vy;
    let tv = (tvx * tvx + tvy * tvy).sqrt();

    // Intercept time estimate (clamped)
    let mut t = intercept_time(rx, ry, tvx, tvy, interceptor_speed).unwrap_or(r / interceptor_speed);
    t = t.clamp(0.0, 8.0);

    // Predicted intercept point
    let px = to.x + tvx * t;
    let py = to.y + tvy * t;

    // Perp direction relative to target velocity (if available)
    let (mut nx, mut ny) = if tv > 1e-6 {
        (-tvy / tv, tvx / tv)
    } else {
        (-ly, lx) // perpendicular to LOS as fallback
    };

    // Fix side consistently (no left/right flipping)
    let cross = rx * tvy - ry * tvx;
    if cross < 0.0 {
        nx = -nx;
        ny = -ny;
    }

    // Lateral offset: capped so we ALWAYS still converge
    // rule: offset <= 0.25 * range
    let offset = (0.08 * r).min(0.25 * r).min(200.0);

    let ax = px + nx * offset;
    let ay = py + ny * offset;

    // Direction to aimpoint
    let sx = ax - from.x;
    let sy = ay - from.y;
    let sm = (sx * sx + sy * sy).sqrt().max(1e-9);
    let aimx = sx / sm;
    let aimy = sy / sm;

    // Terminal angle bias: blend toward a direction that is theta off target velocity,
    // BUT keep strong LOS so we don't "fly alongside" and miss.
    let terminal_dist = 500.0; // meters (start terminal inside this)
    let theta_deg:f64 = 22.0;      // force >5° with margin without getting too sideways
    let theta = theta_deg.to_radians();

    if r < terminal_dist && tv > 1e-6 {
        let tx = tvx / tv;
        let ty = tvy / tv;

        // angle-forcing direction relative to target velocity
        let c = theta.cos();
        let s = theta.sin();
        let angx = c * tx + s * nx;
        let angy = c * ty + s * ny;

        // Blend: keep LOS dominant so collision still happens
        // w ramps from 0 to 0.45 as we approach
        let w = (1.0 - (r / terminal_dist)).clamp(0.0, 0.45);

        let mut dx = (1.0 - w) * aimx + w * angx;
        let mut dy = (1.0 - w) * aimy + w * angy;

        let dm = (dx * dx + dy * dy).sqrt().max(1e-9);
        dx /= dm;
        dy /= dm;
        return (dx, dy);
    }

    (aimx, aimy)
}

fn calculate_angle_between_vectors(vx1: f64, vy1: f64, vx2: f64, vy2: f64) -> f64 {
    let dot = vx1 * vx2 + vy1 * vy2;
    let m1 = (vx1 * vx1 + vy1 * vy1).sqrt();
    let m2 = (vx2 * vx2 + vy2 * vy2).sqrt();
    if m1 > 1e-9 && m2 > 1e-9 {
        let cosv = (dot / (m1 * m2)).clamp(-1.0, 1.0);
        cosv.acos().to_degrees()
    } else {
        0.0
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // "Real-ish" sim settings
    let dt: f64 = 0.1;               // 100 ms per step
    let max_steps: usize = 20000;    // 2000 seconds max

    // Example speeds (m/s)
    let target_speed: f64 = 50.0;        // ~180 km/h
    let interceptor_speed: f64 = 55.55;   // faster than target

    // Initial: target at 300m altitude moving +x, interceptor at ground
    let mut target = Target::new(0.0, 300.0, target_speed, 0.0);
    let mut interceptor = Interceptor::new(0.0, 0.0, 0.0, 0.0);

    let mut rng = rand::thread_rng();

    let mut target_positions: Vec<(f64, f64)> = vec![];
    let mut interceptor_positions: Vec<(f64, f64)> = vec![];

    let collision_threshold = 5.0; // meters
    let mut collision_angle: Option<f64> = None;

    // Target "wobble": small heading perturbations, plus mild height hold
    let target_initial_height = 300.0;
    let correction_weight = 0.8;
    let p_gain = 0.02; // small because dt exists now

    for _ in 0..max_steps {
        // Check collision before update
        let dist = interceptor.distance_to(&target);
        if dist < collision_threshold {
            let angle = calculate_angle_between_vectors(target.vx, target.vy, interceptor.vx, interceptor.vy);
            collision_angle = Some(angle);
            break;
        }

        // --- Target update: small random heading change + height correction ---
        let random_angle_deg: f64 = rng.gen_range(-2.0..2.0); // smaller because dt
        let height_error = target.y - target_initial_height;
        let correction_angle_deg = -height_error * p_gain;

        let blended_deg = random_angle_deg * (1.0 - correction_weight) + correction_angle_deg * correction_weight;
        let ang = blended_deg.to_radians();

        let ca = ang.cos();
        let sa = ang.sin();

        // Rotate target velocity, keep speed approximately constant
        let vx = target.vx * ca - target.vy * sa;
        let vy = target.vx * sa + target.vy * ca;

        // Normalize back to target_speed
        let vm = (vx * vx + vy * vy).sqrt().max(1e-9);
        target.vx = vx / vm * target_speed;
        target.vy = vy / vm * target_speed;

        // --- Interceptor steering ---
        let (dirx, diry) = calculate_steering_direction(&interceptor, &target, interceptor_speed);
        interceptor.vx = dirx * interceptor_speed;
        interceptor.vy = diry * interceptor_speed;

        // Move both
        target.update(dt);
        interceptor.update(dt);

        target_positions.push((target.x, target.y));
        interceptor_positions.push((interceptor.x, interceptor.y));
    }

    match collision_angle {
        Some(a) => {
            println!("✅ Collision occurred");
            if a > 5.0 {
                println!("✅ Angle between velocities: {:.2}° (greater than 5°)", a);
            } else {
                println!("❌ Angle between velocities: {:.2}° (less than 5°)", a);
            }
        }
        None => println!("❌ No collision occurred"),
    }

    visualize_simulation(&target_positions, &interceptor_positions)?;
    Ok(())
}

fn visualize_simulation(
    target_positions: &[(f64, f64)],
    interceptor_positions: &[(f64, f64)],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("collision_simulation0.png", (1400, 900)).into_drawing_area();
    root.fill(&WHITE)?;

    let max_x = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(x, _)| *x)
        .fold(0.0, f64::max)
        .max(10.0) * 1.05;

    let min_y = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(_, y)| *y)
        .fold(f64::INFINITY, f64::min)
        .min(0.0) - 20.0;

    let max_y = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(_, y)| *y)
        .fold(0.0, f64::max)
        .max(10.0) * 1.05 + 20.0;

    let mut chart = ChartBuilder::on(&root)
        .caption("Target vs Interceptor Simulation (Stop at <0.5m distance)", ("sans-serif", 30))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(0f64..max_x, min_y..max_y)?;

    chart
        .configure_mesh()
        .x_label_style(("sans-serif", 15))
        .y_label_style(("sans-serif", 15))
        .y_desc("Height (m)")
        .x_desc("Distance (m)")
        .draw()?;

    // Lines only (clean)
    chart.draw_series(LineSeries::new(
        target_positions.iter().copied(),
        ShapeStyle::from(&RED).stroke_width(2),
    ))?;

    chart.draw_series(LineSeries::new(
        interceptor_positions.iter().copied(),
        ShapeStyle::from(&GREEN).stroke_width(2),
    ))?;

    // Sparse markers every N points (optional, keeps it readable)
    let stride = 50;
    chart.draw_series(
        target_positions.iter().step_by(stride).map(|p| Circle::new(*p, 2, RED.filled())),
    )?;
    chart.draw_series(
        interceptor_positions.iter().step_by(stride).map(|p| Circle::new(*p, 2, GREEN.filled())),
    )?;

    root.present()?;
    println!("✅ Graph saved as 'collision_simulation0.png'");
    Ok(())
}
