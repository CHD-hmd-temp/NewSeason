#![allow(dead_code)]
use bevy::ecs::system::Resource;
use crate::octree::octree::Octree;
use crate::data_reader::structor::Point3;

#[derive(Debug, Resource)]
pub struct ApfConfig {
    pub k_att: f32,   // Attractive force gain
    pub k_rep: f32,   // Repulsive force gain
    pub d0: f32,      // Influence radius
    pub step_size: f32,   // Step size
    pub epsilon: f32,  // Goal radius
    pub max_steps: u32, // Maximum iteration steps
}

impl Default for ApfConfig {
    fn default() -> Self {
        Self {
            k_att: 0.1,
            k_rep: 0.1,
            d0: 1.0,
            step_size: 0.1,
            epsilon: 0.1,
            max_steps: 1000,
        }
    }
}

#[derive(Debug)]
pub enum ApfError {
    LocalMinimum,
    MaxStepsReached,
}

fn distance(a: &Point3, b: &Point3) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx*dx + dy*dy + dz*dz).sqrt()
}

fn compute_attractive_force(
    current: &Point3,
    goal: &Point3,
    k_att: f32
) -> Point3 {
    let vec = goal.sub(current);
    Point3 {
        x: vec.x * k_att,
        y: vec.y * k_att,
        z: vec.z * k_att,
    }
}

fn compute_repulsive_force(
    current: &Point3,
    obstacles: Vec<(f32, [f32; 3])>,
    k_rep: f32,
    d0: f32,
) -> Point3 {
    let mut f_rep = Point3 { x: 0.0, y: 0.0, z: 0.0 };

    for (d, point) in obstacles {
        if d <= d0 && d > 0.0 {
            let term = (1.0/d - 1.0/d0) * k_rep / d.powi(2);
            let vec = current.sub(&Point3::new(point[0], point[1], point[2]));
            f_rep.x += vec.x * term;
            f_rep.y += vec.y * term;
            f_rep.z += vec.z * term;
        }
    }

    f_rep
}

fn add_forces(f_att: Point3, f_rep: Point3) -> Point3 {
    Point3 {
        x: f_att.x + f_rep.x,
        y: f_att.y + f_rep.y,
        z: f_att.z + f_rep.z,
    }
}

pub fn apf_plan(
    start: Point3,
    goal: Point3,
    octree: &Octree,
    config: ApfConfig,
) -> Result<Vec<Point3>, ApfError> {
    let mut path = vec![start];
    let mut current_pos = start;
    let mut steps = 0;
    let octree_map = octree.get_laser_points();

    while distance(&current_pos, &goal) > config.epsilon && steps < config.max_steps {
        let f_att = compute_attractive_force(&current_pos, &goal, config.k_att);

        let mut obstacle_list: Vec<(f32, [f32; 3])> = Vec::new();

        for (_, points) in &octree_map {
            for point in points {
                let x = point.x;
                let y = point.y;
                let z = point.z;
                let distance = distance(&current_pos, &Point3::new(x, y, z));
                if distance < config.d0 {
                    obstacle_list.push((distance, [x, y, z]));
                }
            }
        }

        let f_rep = compute_repulsive_force(&current_pos, obstacle_list, config.k_rep, config.d0);

        let f_total = add_forces(f_att, f_rep);

        // 处理零向量（局部极小）
        if let Some(direction) = f_total.normalize() {
            current_pos = Point3 {
                x: current_pos.x + direction.x * config.step_size,
                y: current_pos.y + direction.y * config.step_size,
                z: current_pos.z + direction.z * config.step_size,
            };
            path.push(current_pos);
        } else {
            return Err(ApfError::LocalMinimum);
        }

        steps += 1;
    }

    if distance(&current_pos, &goal) <= config.epsilon {
        Ok(path)
    } else {
        Err(ApfError::MaxStepsReached)
    }
}