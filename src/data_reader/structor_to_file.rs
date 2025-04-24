#![allow(dead_code)]
use crate::octree::octree;
use crate::api::MavlinkArgs;
use std::io::Write;

// TODO: implement octree_to_csv
pub fn octree_to_csv(_octree_input: octree::Octree, file_name: &str) {
    let mut file = std::fs::File::create(file_name)
        .unwrap_or_else(|_| panic!("Failed to create {}", file_name));

    writeln!(file, "x,y,z,reflectivity").expect("Failed to write to file");
}

#[allow(non_snake_case)]
pub fn MavlinkArgs_to_file(mavlink_args: &MavlinkArgs, file_name: &str) {
    let mut file = std::fs::File::create(file_name)
        .unwrap_or_else(|_| panic!("Failed to create {}", file_name));

    writeln!(file, "time_boot_ms: {}", mavlink_args.time_boot_ms).expect("Failed to write to file");
    writeln!(file, "target_system: {}", mavlink_args.target_system).expect("Failed to write to file");
    writeln!(file, "target_component: {}", mavlink_args.target_component).expect("Failed to write to file");
    writeln!(file, "coordinate_frame: {}", mavlink_args.coordinate_frame).expect("Failed to write to file");
    writeln!(file, "type_mask: {}", mavlink_args.type_mask).expect("Failed to write to file");
    writeln!(file, "x: {}", mavlink_args.x).expect("Failed to write to file");
    writeln!(file, "y: {}", mavlink_args.y).expect("Failed to write to file");
    writeln!(file, "z: {}", mavlink_args.z).expect("Failed to write to file");
    writeln!(file, "vx: {}", mavlink_args.vx).expect("Failed to write to file");
    writeln!(file, "vy: {}", mavlink_args.vy).expect("Failed to write to file");
    writeln!(file, "vz: {}", mavlink_args.vz).expect("Failed to write to file");
    writeln!(file, "afx: {}", mavlink_args.afx).expect("Failed to write to file");
    writeln!(file, "afy: {}", mavlink_args.afy).expect("Failed to write to file");
    writeln!(file, "afz: {}", mavlink_args.afz).expect("Failed to write to file");
    writeln!(file, "yaw: {}", mavlink_args.yaw).expect("Failed to write to file");
    writeln!(file, "yaw_rate: {}", mavlink_args.yaw_rate).expect("Failed to write to file");
}