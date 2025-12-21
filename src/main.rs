use plotters::prelude::*;

#[derive(Debug, Clone, Copy)]
struct Projectile {
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
}

impl Projectile {
    fn new(x: f64, y: f64, vx: f64, vy: f64) -> Self {
        Projectile { x, y, vx, vy }
    }

    fn update(&mut self) {
        // Update position based on velocity
        self.x += self.vx;
        self.y += self.vy;
    }

    fn distance_to(&self, other: &Projectile) -> f64 {
        // Calculate distance to another projectile
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

// Calculate steering direction towards target (unit vector)
fn calculate_steering_direction(from: &Projectile, to: &Projectile) -> (f64, f64) {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let distance = (dx * dx + dy * dy).sqrt();
    
    if distance > 0.1 {
        (dx / distance, dy / distance)
    } else {
        (0.0, 0.0)
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
    // Initialize projectiles
    let mut red = Projectile::new(0.0, 20.0, 2.0, 0.0); // Red: 20m height, horizontal
    let mut green = Projectile::new(0.0, 0.0, 0.0, 0.0); // Green: at ground level
    let green_speed = 2.5; // Speed of green projectile

    let mut red_positions = vec![];
    let mut green_positions = vec![];
    let mut collision_point: Option<(f64, f64)> = None;
    let mut collision_angle: Option<f64> = None;

    let collision_threshold = 1.0; // Stop at < 1m distance

    // Simulation for 1000 time steps
    for step in 0..1000 {
        // Green projectile steers directly towards red
        let distance = green.distance_to(&red);
        let (mut dir_x, mut dir_y) = calculate_steering_direction(&green, &red);
        
        // Normalize direction vector
        let dir_magnitude = (dir_x * dir_x + dir_y * dir_y).sqrt();
        if dir_magnitude > 0.0 {
            dir_x /= dir_magnitude;
            dir_y /= dir_magnitude;
        }
        
        green.vx = dir_x * green_speed;
        green.vy = dir_y * green_speed;

        // Update positions
        red.update();
        green.update();

        // Store positions
        red_positions.push((step as f64, red.y));
        green_positions.push((step as f64, green.y));

        if step % 100 == 0 {
            println!("Step {}: Red ({:.2}, {:.2}), Green ({:.2}, {:.2}), Distance: {:.2}m",
                step, red.x, red.y, green.x, green.y, distance);
        }

        // Collision detection: stop at < 1m
        if distance < collision_threshold {
            collision_point = Some((step as f64, red.y));
            
            // Calculate angle between velocity vectors
            let angle = calculate_angle_between_vectors(red.vx, red.vy, green.vx, green.vy);
            collision_angle = Some(angle);
            
            println!("\n💥 COLLISION! at time step: {}", step);
            println!("Red position: ({:.2}, {:.2})", red.x, red.y);
            println!("Green position: ({:.2}, {:.2})", green.x, green.y);
            println!("Distance: {:.2}m", distance);
            println!("Red velocity: ({:.2}, {:.2})", red.vx, red.vy);
            println!("Green velocity: ({:.2}, {:.2})", green.vx, green.vy);
            if angle > 20.0 {
                println!("✅ Angle between velocities is greater than 20°");
            } else {
                println!("❌ Angle between velocities is less than or equal to 20°");
            }
            break;
        }
    }

    // Visualization
    visualize_simulation(&red_positions, &green_positions, collision_point, collision_angle)?;

    Ok(())
}






fn visualize_simulation(
    red_positions: &[(f64, f64)],
    green_positions: &[(f64, f64)],
    collision_point: Option<(f64, f64)>,
    collision_angle: Option<f64>,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("collision_simulation.png", (1400, 900)).into_drawing_area();
    root.fill(&WHITE)?;

    let max_time = red_positions.len() as f64;
    let max_y = 30.0;

    let mut chart = ChartBuilder::on(&root)
        .caption("Projectile Collision Simulation (Stop at <1m distance)", ("sans-serif", 30))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .right_y_label_area_size(50)
        .build_cartesian_2d(
            0f64..max_time,
            0f64..max_y,
        )?
        .set_secondary_coord(0f64..max_time, 0f64..max_y);

    // Draw red projectile line
    chart
        .draw_series(LineSeries::new(
            red_positions.iter().copied(),
            ShapeStyle::from(&RED).stroke_width(2),
        ))?
        .label("Red Projectile (20m altitude)");

    // Draw green projectile line
    chart
        .draw_series(LineSeries::new(
            green_positions.iter().copied(),
            ShapeStyle::from(&GREEN).stroke_width(2),
        ))?
        .label("Green Projectile (pursuing)");

    // Draw points and arrows for each simulation step for red projectile
    for window in red_positions.windows(2) {
        let (x1, y1) = window[0];
        
        // Draw point at current position
        chart.draw_series(std::iter::once(Circle::new(
            (x1, y1),
            3,
            ShapeStyle::from(&RED).filled(),
        )))?;
    }

    // Draw points and arrows for each simulation step for green projectile
    for window in green_positions.windows(2) {
        let (x1, y1) = window[0];
        
        // Draw point at current position
        chart.draw_series(std::iter::once(Circle::new(
            (x1, y1),
            3,
            ShapeStyle::from(&GREEN).filled(),
        )))?;
    }

    // Draw X at collision point
    if let Some((collision_x, collision_y)) = collision_point {
        if let Some(angle) = collision_angle {
            if angle > 20.0 {
                // Draw green checkmark for angle > 20°
                let size = 1.5;
                
                // Checkmark: first part (bottom-left to middle)
                chart.draw_series(std::iter::once(PathElement::new(
                    vec![
                        (collision_x - size * 0.5, collision_y),
                        (collision_x - size * 0.2, collision_y + size * 0.3),
                    ],
                    ShapeStyle::from(&GREEN).stroke_width(4),
                )))?;
                
                // Checkmark: second part (middle to top-right)
                chart.draw_series(std::iter::once(PathElement::new(
                    vec![
                        (collision_x - size * 0.2, collision_y + size * 0.3),
                        (collision_x + size * 0.7, collision_y - size * 0.5),
                    ],
                    ShapeStyle::from(&GREEN).stroke_width(4),
                )))?;
            } else {
                // Draw red X for angle <= 20°
                let x_size = 1.5;
                
                // Diagonal line 1 (top-left to bottom-right)
                chart.draw_series(std::iter::once(PathElement::new(
                    vec![
                        (collision_x - x_size, collision_y - x_size),
                        (collision_x + x_size, collision_y + x_size),
                    ],
                    ShapeStyle::from(&RED).stroke_width(4),
                )))?;
                
                // Diagonal line 2 (top-right to bottom-left)
                chart.draw_series(std::iter::once(PathElement::new(
                    vec![
                        (collision_x + x_size, collision_y - x_size),
                        (collision_x - x_size, collision_y + x_size),
                    ],
                    ShapeStyle::from(&RED).stroke_width(4),
                )))?;
            }
        }
    }

    // Configure axes
    chart
        .configure_mesh()
        .x_label_style(("sans-serif", 15))
        .y_label_style(("sans-serif", 15))
        .y_desc("Height (m)")
        .x_desc("Time step")
        .draw()?;

    chart
        .configure_secondary_axes()
        .y_desc("Height (m)")
        .draw()?;

    root.present()?;
    println!("✅ Graph saved as 'collision_simulation.png'");

    Ok(())
}
