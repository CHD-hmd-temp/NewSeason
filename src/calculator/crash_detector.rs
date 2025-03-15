use crate::octree::octree;
use crate::prelude::*;
use crate::calculator::coordinate_switch::mid360_to_frd;

pub fn crash_warn_for_octree(
    octree_input: &octree::Octree,
    warn_trigger_distance: f32,
) -> (bool, Vec<(f32, Point3f)>) {
    // TODO: change input to map
    let octree_map = octree_input.get_laser_points();
    let mut result = false;
    let mut obstacle_list: Vec<(f32, Point3f)> = Vec::new();
    let alert_distance = (warn_trigger_distance * 3.0).floor() as u32;

    for (distance, points)  in octree_map {
        if distance > alert_distance {
            continue;
        }
        for point in points {
            if point.0 <= warn_trigger_distance {
                result = true;
            }
            obstacle_list.push((distance as f32, point.1.coordinate));
        }
    }
    return (result, obstacle_list);
}

// TODO: implement speed_factor
/// Return velocity vector to avoid obstacles
pub fn obstacle_avoidance(
    obstacle_list: &Vec<(f32, Point3f)>,
    warn_trigger_distance: f32,
    //mavlink_args: &MavlinkArgs,
) -> MavlinkArgs {
    const EPSILON: f32 = 1e-6;
    const MAX_SPEED: f32 = 1.0;
    let mut result = MavlinkArgs::new(
        0,
        1,
        1,
        9,
        0b0000001000000000,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    );
    if obstacle_list.is_empty() {
        return result;
    }
    let mut sorted_obstacle_list = obstacle_list.clone();
    sorted_obstacle_list.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    if sorted_obstacle_list[0].0 > warn_trigger_distance {
        return result;
    }
    let (mut sum_x, mut sum_y, mut sum_z) = (0.0, 0.0, 0.0);
    for &(distance, coordinate) in &sorted_obstacle_list {
        let weight = 1.0 / (distance.powi(3) + EPSILON);
        sum_x += -coordinate.x * weight;
        sum_y += -coordinate.y * weight;
         sum_z += -coordinate.z * weight;
    }

    let _minimum_distance = sorted_obstacle_list[0].0;
    let speed = MAX_SPEED;

    let magnitude = (sum_x.powi(2) + sum_y.powi(2) + sum_z.powi(2)).sqrt();
    if magnitude < EPSILON {
        let (_, coordinate_closest) = sorted_obstacle_list[0];
        let dir_mag = distance(&coordinate_closest, &Point3f::new(0.0, 0.0, 0.0));
        if dir_mag < EPSILON {
            result.type_mask = 0b010111111111;
            result.yaw_rate = 0.5; // TODO: stop after a while
            return result;
        }
        else {
            result.type_mask = 0b0000001000000000;
            let vx = speed * -coordinate_closest.x / dir_mag;
            let vy = speed * -coordinate_closest.y / dir_mag; // Change O-XYZ to O-FRD
            let vz = speed * -coordinate_closest.z / dir_mag;
            let (vx, vy, vz) = mid360_to_frd(vx, vy, vz);
            result.vx = vx;
            result.vy = vy;
            result.vz = vz;
            return result;
        }
    }
    else {
        result.type_mask = 0b0000001000000000;
        let vx = speed * sum_x / magnitude;
        let vy = speed * sum_y / magnitude; // Change O-XYZ to O-FRD
        let vz = speed * sum_z / magnitude;
        let (vx, vy, vz) = mid360_to_frd(vx, vy, vz);
        result.vx = vx;
        result.vy = vy;
        result.vz = vz;
        return result;
    }
}
