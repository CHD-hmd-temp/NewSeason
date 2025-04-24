#![allow(dead_code)]
use bevy::render::render_resource::encase::private::Length;
use std::collections::HashMap;
use crate::octree::octree::Octree;
use crate::prelude::*;
use crate::config::ApfConfig;

fn compute_attractive_force(
    current: &Point3f,
    goal: &Point3f,
    k_att: f32
) -> Point3f {
    let vec = goal - current;
    Point3f::new(
        vec.x * k_att,
        vec.y * k_att,
        vec.z * k_att,
    )
}

fn compute_repulsive_force(
    current: &Point3f,
    obstacles: Vec<(f32, Point3f)>,
    k_rep: f32,
    d0: f32,
) -> Point3f {
    let mut f_rep = Point3f::new(0.0, 0.0, 0.0);

    for (d, point) in obstacles {
        if d <= d0 && d > 0.0 {
            let term = (1.0/d - 1.0/d0) * k_rep / d.powi(2);
            let vec = current - point;
            f_rep.x += vec.x * term;
            f_rep.y += vec.y * term;
            f_rep.z += vec.z * term;
        }
    }

    f_rep
}

fn add_forces(f_att: Point3f, f_rep: Point3f) -> Point3f {
    Point3f::new(
        f_att.x + f_rep.x,
        f_att.y + f_rep.y,
        f_att.z + f_rep.z,
    )
}

pub fn apf_plan(
    start: Point3f,
    goal: Point3f,
    octree: &Octree,
    config: &ApfConfig,
) -> Result<Vec<Point3f>, ApfError<Point3f>> {
    if goal == Point3f::new(0.0, 0.0, 0.0) {
        return Ok(Vec::new());
    }
    let mut path = vec![start];
    let mut current_pos = start;
    let mut steps = 0;
    let octree_map = octree.get_laser_points();

    while distance(&current_pos, &goal) > config.epsilon && steps < config.max_steps {
        let f_att = compute_attractive_force(&current_pos, &goal, config.k_att);
        let mut obstacle_list: Vec<(f32, Point3f)> = Vec::new();

        for (_, points) in &octree_map {
            for point in points {
                let distance = distance(&current_pos, &point.1.coordinate);
                if distance < config.d0 {
                    obstacle_list.push((distance, point.1.coordinate));
                }
            }
        }

        let f_rep = compute_repulsive_force(&current_pos, obstacle_list, config.k_rep, config.d0);
        let f_total = add_forces(f_att, f_rep);

        if let Some(direction) = normalize(&f_total) {
            current_pos = Point3f::new(
                current_pos.x + direction.x * config.step_size,
                current_pos.y + direction.y * config.step_size,
                current_pos.z + direction.z * config.step_size,
            );
            path.push(current_pos);
        } else {
            return Err(ApfError::LocalMinimum(Vec::new()));
        }

        steps += 1;
    }

    if distance(&current_pos, &goal) <= config.epsilon {
        Ok(path)
    } else {
        Err(ApfError::MaxStepsReached(path))
    }
}

pub fn apf_plan_mavlink(
    start: Point3f,
    goal: Point3f,
    octree_map: &HashMap<u32, Vec<(f32, LaserPoint)>>,
    config: &ApfConfig,
) -> Result<Vec<crate::api::MavlinkArgs>, ApfError<MavlinkArgs>> {
    use crate::api::MavlinkArgs;
    let mut mavlink_vec = Vec::new();
    if goal == Point3f::new(0.0, 0.0, 0.0) {
        mavlink_vec.push(MavlinkArgs::default());
        return Ok(mavlink_vec);
    }
    let mut path = vec![start];
    let mut current_pos = start;
    let mut steps = 0;
    let mut mavlink_args: MavlinkArgs = MavlinkArgs::default();

    while distance(&current_pos, &goal) > config.epsilon && steps < config.max_steps {
        let f_att = compute_attractive_force(&current_pos, &goal, config.k_att);
        let mut obstacle_list: Vec<(f32, Point3f)> = Vec::new();

        for (_, points) in octree_map {
            for point in points {
                let distance = distance(&current_pos, &point.1.coordinate);
                if distance < config.d0 {
                    obstacle_list.push((distance, point.1.coordinate));
                }
            }
        }
        let f_rep = compute_repulsive_force(&current_pos, obstacle_list, config.k_rep, config.d0);
        let f_total = add_forces(f_att, f_rep);

        if let Some(direction) = normalize(&f_total) {
            current_pos = Point3f::new(
                current_pos.x + direction.x * config.step_size,
                current_pos.y + direction.y * config.step_size,
                current_pos.z + direction.z * config.step_size,
            );
            path.push(current_pos);
            if path.length() == 1 {
                continue;
            }
            let velocity = path[path.length() - 1] - path[path.length() - 2];
            mavlink_args = MavlinkArgs {
                target_component: 0,
                target_system: 0,
                time_boot_ms: 0,
                coordinate_frame: 0,
                type_mask: 0,
                x: 0.,
                y: 0.,
                z: 0.,
                vx: velocity.x,
                vy: velocity.y,
                vz: velocity.z,
                afx: 0.,
                afy: 0.,
                afz: 0.,
                yaw: 0.0,
                yaw_rate: 0.0,
            };
            mavlink_vec.push(mavlink_args);
        } else {
            return Err(ApfError::LocalMinimum(mavlink_vec));
        }

        steps += 1;
    }

    if distance(&current_pos, &goal) <= config.epsilon {
        Ok(mavlink_vec)
    } else {
        Err(ApfError::MaxStepsReached(mavlink_vec))
    }
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum NavState {
    FollowHallway,
    TurnLeft,
    MoveToGoal,
    Hover,
}

/// 检测是否位于拐角，即右侧存在墙体而左侧没有墙体，且前方存在墙体
use crate::api::MavlinkArgs;
use super::crash_detector::has_obstacle_in_direction;
pub fn l_shape_navigation(
    octree_map: &HashMap<u32, Vec<(f32, LaserPoint)>>,
    state: NavState,
    d0: f32,
) -> NavState {
    let front = has_obstacle_in_direction(octree_map, 0.0, d0);
    let left = has_obstacle_in_direction(octree_map, 90.0, d0);
    let right = has_obstacle_in_direction(octree_map, 270.0, d0);
    let mut new_state = NavState::FollowHallway;
    println!("front: {}, left: {}, right: {}", front, left, right);
    match state {
        NavState::FollowHallway => {
            if front && !left && right {
                new_state = NavState::TurnLeft;
            }
        }
        NavState::TurnLeft => {
            if !left && front {
                new_state = NavState::MoveToGoal;
            }
            else {
                new_state = NavState::TurnLeft;
            }
        }
        NavState::MoveToGoal => {
            if left && !right && front {
                new_state = NavState::Hover;
            }
            else {
                new_state = NavState::MoveToGoal;
            }
        }
        NavState::Hover => {
            new_state = NavState::Hover;
        }
    };

    new_state
}

fn normalize(v: &Point3f) -> Option<Point3f> {
    let norm = (v.x.powi(2) + v.y.powi(2) + v.z.powi(2)).sqrt();
    if norm > 0.0 {
        Some(Point3f::new(v.x / norm, v.y / norm, v.z / norm))
    } else {
        None
    }
}