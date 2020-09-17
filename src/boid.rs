use nalgebra::*;
use wasm_bindgen::prelude::*;

/// Local flock range determines how close another boid must be before the current boid conciders
/// it while setting course.
pub const LOCAL_RANGE: f32 = 100.0;
pub const LOCAL_RANGE_SQ: f32 = LOCAL_RANGE * LOCAL_RANGE;

const MAX_SPEED: f32 = 400.0;
const MAX_ACC: f32 = 600.0;
/// Margin to avoid obsticles
const MARGIN: f32 = 100.0;
/// Acceleration away from obsticles
const OBS_ACC: f32 = MAX_ACC;

#[wasm_bindgen]
#[derive(Clone)]
#[repr(C)]
pub struct Boid {
    position: Point2<f32>,
    velocity: Vector2<f32>,
}

impl Boid {
    pub fn new(x: f32, y: f32) -> Boid {
        Boid {
            position: Point2::new(x, y),
            velocity: Vector2::x_axis().into_inner(),
        }
    }

    pub fn is_close(&self, other: &Boid) -> bool {
        let dist_sq = (self.position - other.position).magnitude_squared();
        dist_sq < LOCAL_RANGE_SQ
    }

    /// Update the boid based on current neighbors
    pub fn update(&mut self, neighbors: &Vec<Boid>, dt: f32, bounds: Vector2<f32>, obstacle: Option<Point2<f32>>) {
        let avoid_boids = self.avoid(&neighbors);
        let to_center = self.to_center(&neighbors);
        let match_heading = self.match_heading(&neighbors);
        let acc_to_target_speed = self.acc_to_target_speed();
        let avoid_bounds = self.avoid_bounds(bounds);
        let avoid_obstacle = self.avoid_obstacle(obstacle);

        let acc = 10.0 *
            (6.0 * avoid_boids
             + to_center
             + match_heading
             + acc_to_target_speed
             + avoid_bounds
             + avoid_obstacle
            );
        self.velocity += clamp_mag(acc, MAX_ACC) * dt;
        self.velocity = clamp_mag(self.velocity, MAX_SPEED);
    }

    fn acc_to_target_speed(&self) -> Vector2<f32> {
        let v_mag = self.velocity.magnitude();
        if v_mag > 0.001 {
            let diff = MAX_SPEED - v_mag;
            self.velocity * diff / v_mag
        } else {
            Vector2::new(1.0, 0.0)
        }
    }

    /// Vector to avoid nearby boids
    fn avoid(&self, group: &Vec<Boid>) -> Vector2<f32> {
        let mut total = Vector2::zeros();
        for boid in group {
            total += self.avoid_point(boid.position, LOCAL_RANGE);
        }
        if group.len() > 0 {
            total / (group.len() as f32)
        } else {
            Vector2::zeros()
        }
    }

    fn avoid_point(&self, point: Point2<f32>, radius: f32) -> Vector2<f32> {
        let diff = self.position - point;
        let diff_mag = diff.magnitude();
        if diff_mag > 0.001 {
            let avoid_mag = radius - diff_mag;
            if avoid_mag > 0.0 {
                diff * avoid_mag / diff_mag
            } else {
                Vector2::zeros()
            }
        } else {
            Vector2::zeros()
        }
    }

    fn avoid_obstacle(&self, obstacle: Option<Point2<f32>>) -> Vector2<f32> {
        match obstacle {
            Some(point) => self.avoid_point(point, 300.0),
            None => Vector2::zeros(),
        }
    }

    fn avoid_bounds(&self, bounds: Vector2<f32>) -> Vector2<f32> {
        let mut total = Vector2::zeros();

        if self.position.x < MARGIN {
            total += Vector2::new(OBS_ACC, 0.0);
        }
        if self.position.y < MARGIN {
            total += Vector2::new(0.0, OBS_ACC);
        }
        if self.position.x > bounds.x - MARGIN {
            total += Vector2::new(-OBS_ACC, 0.0);
        }
        if self.position.y > bounds.y - MARGIN {
            total += Vector2::new(0.0, -OBS_ACC);
        }

        total
    }

    /// Vector to go to the center of mass of the neighbors
    fn to_center(&self, group: &Vec<Boid>) -> Vector2<f32> {
        let com = center_of_mass(group).unwrap_or(self.position);
        com - self.position
    }

    fn match_heading(&self, group: &Vec<Boid>) -> Vector2<f32> {
        if let Some(group_heading) = average_heading(group) {
            group_heading - self.velocity
        } else {
            Vector2::zeros()
        }
    }

}

fn center_of_mass(group: &Vec<Boid>) -> Option<Point2<f32>> {
    if group.len() == 0 {
        return None;
    }
    let mut total = Vector2::zeros();
    let num = group.len() as f32;
    for boid in group {
        total += boid.position.coords;
    }
    Some(Point2::from(total / num))
}

fn average_heading(group: &Vec<Boid>) -> Option<Vector2<f32>> {
    if group.len() > 0 {
        let mut total = Vector2::zeros();
        for boid in group {
            total += boid.velocity;
        }
        Some(total / group.len() as f32)
    } else {
        None
    }
}

#[wasm_bindgen]
pub struct Flock {
    members: Vec<Boid>,
    /// Determines the boundry of the flock as the box from (0,0) to bounds
    bounds: Vector2<f32>,
    /// Location of a circle obstacle for the flock to avoid
    obstacle: Option<Point2<f32>>,
    /// Buffer for using neighbor calcualtions
    neighbor_buffer: Vec<Boid>,
}

#[wasm_bindgen]
impl Flock {
    pub fn empty(x: f32, y: f32) -> Flock {
        Flock {
            members: Vec::new(),
            bounds: Vector2::new(x, y),
            obstacle: None,
            neighbor_buffer: Vec::new(),
        }
    }

    pub fn test_flock(x: f32, y: f32) -> Flock {
        let rows = 10;
        let cols = 10;
        let mut members = Vec::with_capacity(rows * cols);
        for i in 0..rows{
            for j in 0..cols {
                members.push(Boid::new(i as f32 * 10.0, j as f32 * 10.0));
            }
        }
        Flock {
            members,
            bounds: Vector2::new(x, y),
            obstacle: None,
            neighbor_buffer: Vec::with_capacity(10),
        }
    }

    pub fn set_width(&mut self, width: f32) {
        self.bounds.x = width;
    }

    pub fn set_height(&mut self, width: f32) {
        self.bounds.x = width;
    }

    pub fn set_obstacle(&mut self, x: f32, y: f32) {
        self.obstacle = Some(Point2::new(x, y));
    }

    pub fn clear_obstacle(&mut self) {
        self.obstacle = None;
    }

    pub fn boids_ptr(&self) -> *const Boid {
        self.members.as_ptr()
    }

    pub fn num_boids(&self) -> u32 {
        self.members.len() as u32
    }

    /// Runs the simulation for one time step.
    pub fn update(&mut self, dt: f32) {
        for i in 0..self.members.len() {
            self.set_neighbor_buffer(i);
            self.members[i].update(&self.neighbor_buffer, dt, self.bounds, self.obstacle);
        }

        self.update_positions(dt);
    }

    fn update_positions(&mut self, dt: f32) {
        for boid in self.members.iter_mut() {
            boid.position = boid.position + dt * boid.velocity;
        }
    }
}

impl Flock {
    /// Returns a Vec of all boids near **member**,  but not including member
    /// Sets into self.neighbor_buffer, this reuses the same allocated space for every update
    fn set_neighbor_buffer(&mut self, member_index: usize) {
        let member = self.members[member_index].clone();
        self.neighbor_buffer.clear();

        for (i, boid) in self.members.iter().enumerate() {
            if i != member_index && member.is_close(boid) {
                self.neighbor_buffer.push(boid.clone());
            }
        }
    }
}

fn clamp_mag(v: Vector2<f32>, max: f32) -> Vector2<f32> {
    let mag = v.magnitude();
    if mag > max {
        v * max / mag
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::transmute;

    #[test]
    fn test_size() {
        assert_eq!(std::mem::size_of::<Boid>(), 16, "Boid should be 4 bytes");
    }

    #[test]
    fn test_data_layout() {
        let b = Boid::new(1.0, 2.0);
        unsafe {
            let vals = transmute::<Boid, [f32; 4]>(b);
            assert_eq!(vals[0], 1.0, "vals[0] value should be 1.0");
            assert_eq!(vals[1], 2.0, "vals[1] value should be 2.0");
            assert_eq!(vals[2], 1.0, "vals[2] value should be 0.0");
            assert_eq!(vals[3], 0.0, "vals[3] value should be 0.0");
        }
    }
}
