use plotters::prelude::*;
use rand::Rng;

#[derive(Debug, Clone, Copy)]
pub struct Target {
    // Position in meters
    x: f64,
    y: f64,
    // Velocity components in m/step
    vx: f64,
    vy: f64,
}

impl Target {
    fn new(x: f64, y: f64, vx: f64, vy: f64) -> Self {
        Target { x, y, vx, vy }
    }

    fn update(&mut self) {
        // Update position based on velocity
        // Advance position using the current velocity
        self.x += self.vx; // position = velocity * time
        self.y += self.vy; // position = velocity * time
    }

    fn distance_to(&self, other: &Target) -> f64 {
        // Calculate distance to another projectile
        // Euclidean distance between two projectiles
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

pub type Interceptor = Target;

// Calculate steering direction towards target (unit vector)
// Calculate unit steering direction from interceptor towards target
fn calculate_steering_direction(from: &Interceptor, to: &Target) -> (f64, f64) {
    // ----------------------------
    // Line of sight (LOS)
    // ----------------------------
    let rx = to.x - from.x;
    let ry = to.y - from.y;
    let r = (rx * rx + ry * ry).sqrt();

    // If we're already at the target (or numerically too close), return no direction.
    if r < 1e-9 {
        return (0.0, 0.0);
    }

    // Target velocity (available in Target struct)
    let tvx = to.vx;
    let tvy = to.vy;
    let tv = (tvx * tvx + tvy * tvy).sqrt();

    // Interceptor speed magnitude (available in Interceptor/Target struct)
    // Use a small floor to avoid division by zero.
    let ivx = from.vx;
    let ivy = from.vy;
    let iv = (ivx * ivx + ivy * ivy).sqrt().max(1e-6);

    // ----------------------------
    // 1) Lead pursuit (far range)
    // ----------------------------
    // Simple time-to-go estimate and clamp for stability.
    let t_go = (r / iv).clamp(0.0, 3.0);
    let lead_x = to.x + tvx * t_go;
    let lead_y = to.y + tvy * t_go;

    // Default aim point is the lead point
    let mut aim_x = lead_x;
    let mut aim_y = lead_y;

    // ----------------------------
    // 2) Terminal angle forcing (close range)
    //    Rotate LOS by +/- theta to guarantee a crossing component.
    // ----------------------------
    let terminal_dist = 300.0;      // start forcing inside this distance (tune 500..3000)
      // force this approach angle (tune 15..35)
    
    let terminal_angle_deg: f64 = 25.0;
    let theta = terminal_angle_deg.to_radians();

    if r < terminal_dist {
        // Unit LOS
        let lx = rx / r;
        let ly = ry / r;

        // Choose a consistent side using the sign of cross(LOS, target_vel)
        // cross = lx*tvy - ly*tvx
        let mut sign = 1.0;
        if tv > 1e-6 {
            let cross = lx * tvy - ly * tvx;
            if cross < 0.0 {
                sign = -1.0;
            }
        }

        // Rotate LOS by +/- theta
        let c = theta.cos();
        let s = theta.sin() * sign;

        let dirx = lx * c - ly * s;
        let diry = lx * s + ly * c;

        // Create a stable aim point along that rotated direction
        let aim_step = 300.0; // tune 100..600
        aim_x = from.x + dirx * aim_step;
        aim_y = from.y + diry * aim_step;
    } else {
        // ----------------------------
        // Optional: mild lateral offset even when far, to avoid pure tail-chase.
        // (Keeps things robust if terminal mode starts late.)
        // ----------------------------
        if tv > 1e-6 {
            // Perp to target velocity (unit)
            let mut nx = -tvy / tv;
            let mut ny = tvx / tv;

            // Keep side consistent using cross(relative_pos, target_vel)
            let cross = rx * tvy - ry * tvx;
            if cross < 0.0 {
                nx = -nx;
                ny = -ny;
            }

            // Small offset that grows with distance but is capped
            let offset = (0.06 * r).clamp(0.0, 120.0);
            aim_x = lead_x + nx * offset;
            aim_y = lead_y + ny * offset;
        }
    }

    // ----------------------------
    // Convert aim point to a unit steering direction
    // ----------------------------
    let sx = aim_x - from.x;
    let sy = aim_y - from.y;
    let s = (sx * sx + sy * sy).sqrt();

    if s < 1e-9 {
        (0.0, 0.0)
    } else {
        (sx / s, sy / s)
    }
}

// Calculate angle between two velocity vectors in degrees
fn calculate_angle_between_vectors(vx1: f64, vy1: f64, vx2: f64, vy2: f64) -> f64 {
    let dot_product = vx1 * vx2 + vy1 * vy2;
    let magnitude1 = (vx1 * vx1 + vy1 * vy1).sqrt();
    let magnitude2 = (vx2 * vx2 + vy2 * vy2).sqrt();
    
    if magnitude1 > 0.0 && magnitude2 > 0.0 {
        let cos_angle = dot_product / (magnitude1 * magnitude2);
        let angle_rad = cos_angle.acos();
        angle_rad.to_degrees()
    } else {
        0.0
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initial conditions: target starts elevated and moving right; interceptor on ground
    let mut target = Target::new(0.0, 30.0, 2.0, 0.0);
    let mut interceptor = Interceptor::new(0.0, 0.0, 0.0, 0.0);
    let interceptor_speed = 2.5; // Constant interceptor speed (m/step)
    let mut rng = rand::thread_rng();

    // Traces for plotting and collision reporting
    let mut target_positions = vec![];
    let mut interceptor_positions = vec![];
    let mut collision_point: Option<(f64, f64)> = None;
    let mut collision_angle: Option<f64> = None;

    let collision_threshold = 0.5; // Stop when closer than 0.5 m
    
    // Parameters that shape the target's evasive behavior
    let target_initial_height = 30.0; // Reference height for P correction
    let correction_weight = 0.8; // Blend between random and corrective steering Weight of correction (0.0 = pure random, 1.0 = pure correction)
    let p_gain = 0.2; // Proportional gain for height error

    // Run discrete-time simulation
    for _step in 0..10000 {
        // Collision detection before update: check if we're already close
        let distance = interceptor.distance_to(&target);
        
        if distance < collision_threshold {
            collision_point = Some((target.x, target.y));
            
            // Calculate angle between velocity vectors
            let angle = calculate_angle_between_vectors(target.vx, target.vy, interceptor.vx, interceptor.vy);
            collision_angle = Some(angle);
            
            break;
        }
        
        // Random deviation to target's heading between -5° and +5°
        let random_angle_deg: f64 = rng.gen_range(-5.0..5.0);
        
        // P controller: correction angle proportional to height error
        let height_error = target.y - target_initial_height; // Positive if above reference
        let correction_angle_deg = -height_error * p_gain; // Negative drives back up when below target height
        
        // Blend random angle and correction angle based on weight
        let blended_angle_deg = (random_angle_deg * (1.0 - correction_weight)) 
                                + (correction_angle_deg * correction_weight);
        
        let random_angle_rad = blended_angle_deg.to_radians();
        
        // Rotate the target's velocity vector by the random angle
        let cos_angle = random_angle_rad.cos();
        let sin_angle = random_angle_rad.sin();
        let rotated_vx = target.vx * cos_angle - target.vy * sin_angle;
        let rotated_vy = target.vx * sin_angle + target.vy * cos_angle;
        
        target.vx = rotated_vx;
        target.vy = rotated_vy;
        
        // Interceptor points directly toward the target every step
        let (mut dir_x, mut dir_y) = calculate_steering_direction(&interceptor, &target);
        
        // Normalize direction vector
        let dir_magnitude = (dir_x * dir_x + dir_y * dir_y).sqrt();
        if dir_magnitude > 0.0 {
            dir_x /= dir_magnitude;
            dir_y /= dir_magnitude;
        }
        
        interceptor.vx = dir_x * interceptor_speed;
        interceptor.vy = dir_y * interceptor_speed;

        // Move both actors one time step
        target.update();
        interceptor.update();

        // Store positions (both X and Y coordinates) // Store history for visualization
        target_positions.push((target.x, target.y));
        interceptor_positions.push((interceptor.x, interceptor.y));
        
    }

    // Print collision results after simulation ends
    if collision_point.is_some() {
        if let Some((cx, cy)) = collision_point {
            println!("✅ Collision at position x={:.1}, y={:.1}", cx, cy);
        }        
        if let Some(angle) = collision_angle {
            if angle > 5.0 {
                println!("✅ Angle between velocities is: {:.2}° (greater than 5°)", angle);
            } else {
                println!("❌ Angle between velocities is: {:.2}° (less than 5°)", angle);
            }
        }
    } else {
        println!("❌ No collision occurred within 10000 time steps");
    }

    // Visualization
    visualize_simulation(&target_positions, &interceptor_positions)?;

    Ok(())
}



fn visualize_simulation(
    target_positions: &[(f64, f64)],
    interceptor_positions: &[(f64, f64)],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("collision_simulation0.png", (1400, 900)).into_drawing_area();
    root.fill(&WHITE)?;

    // Dynamic plot bounds based on recorded trajectories
    let max_x = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(x, _)| *x)
        .fold(0.0, f64::max)
        .max(10.0) * 1.1; // Add 10% padding

    let max_y = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(_, y)| *y)
        .fold(0.0, f64::max)
        .max(10.0) * 1.1; // Add 10% padding

    // Configure chart canvas and axes
    let mut chart = ChartBuilder::on(&root)
        .caption("Target vs Interceptor Simulation (Stop at <0.5m distance)", ("sans-serif", 30))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(
            0f64..max_x,
            0f64..max_y,
        )?;

    // Draw target line
    chart
        .draw_series(LineSeries::new(
            target_positions.iter().copied(),
            ShapeStyle::from(&RED).stroke_width(2),
        ))?
        .label("Target (random evasion)");

    // Draw interceptor line
    chart
        .draw_series(LineSeries::new(
            interceptor_positions.iter().copied(),
            ShapeStyle::from(&GREEN).stroke_width(2),
        ))?
        .label("Interceptor (pursuing)");

    // Draw point markers for the target
    for pos in target_positions.iter() {
        chart.draw_series(std::iter::once(Circle::new(
            *pos,
            3,
            ShapeStyle::from(&RED).filled(),
        )))?;
    }

    // Draw point markers for the interceptor
    for pos in interceptor_positions.iter() {
        chart.draw_series(std::iter::once(Circle::new(
            *pos,
            3,
            ShapeStyle::from(&GREEN).filled(),
        )))?;
    }

    // Highlight last interceptor position (proxy for collision point)
    if let Some(&last_interceptor_pos) = interceptor_positions.last() {
        let (collision_x, collision_y) = last_interceptor_pos;
        
        chart.draw_series(std::iter::once(Circle::new(
            (collision_x, collision_y),
            25,
            ShapeStyle::from(&BLUE).stroke_width(3),
        )))?;
    }


    // Configure axes
    chart
        .configure_mesh()
        .x_label_style(("sans-serif", 15))
        .y_label_style(("sans-serif", 15))
        .y_desc("Height (m)")
        .x_desc("Distance (m)")
        .draw()?;

    root.present()?;
    println!("✅ Graph saved as 'collision_simulation0.png'");

    Ok(())
}
