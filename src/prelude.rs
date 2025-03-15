use nalgebra::{Matrix3, Vector3, Point3};
use bevy::ecs::system::Resource;

pub type Point3f = Point3<f32>;
pub type Vector3f = Vector3<f32>;
pub type Matrix3f = Matrix3<f32>;

pub struct NodeForRender {
    pub coordinate: Point3f,
    pub reflectivity: u8,
}

#[derive(Debug, Clone)]
pub struct LaserPoint {
    pub coordinate: Point3f,
    pub reflectivity: u8,
    // #[serde(rename = "Tag")]
    // pub tag: u8,
}

impl LaserPoint {
    pub fn new(point: Point3f, reflectivity: u8) -> Self {
        Self {
            coordinate: point,
            reflectivity,
        }
    }
}

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

#[derive(Debug, Clone, Copy)]
pub struct MavlinkArgs {
    pub time_boot_ms: u32,
    pub target_system: u8,
    pub target_component: u8,
    pub coordinate_frame: u8,
    pub type_mask: u16,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub afx: f32,
    pub afy: f32,
    pub afz: f32,
    pub yaw: f32,
    pub yaw_rate: f32,
}

impl MavlinkArgs {
    pub fn new(
        time_boot_ms: u32,
        target_system: u8,
        target_component: u8,
        coordinate_frame: u8,
        type_mask: u16,
        x: f32,
        y: f32,
        z: f32,
        vx: f32,
        vy: f32,
        vz: f32,
        afx: f32,
        afy: f32,
        afz: f32,
        yaw: f32,
        yaw_rate: f32,
    ) -> Self {
        Self {
            time_boot_ms,
            target_system,
            target_component,
            coordinate_frame,
            type_mask,
            x,
            y,
            z,
            vx,
            vy,
            vz,
            afx,
            afy,
            afz,
            yaw,
            yaw_rate,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Resource)]
pub struct ImuData {
    pub version: u8,
    pub length: u16,
    pub time_interval: u16,
    pub dot_num: u16,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: u8,
    pub time_type: u8,
    pub reserved: Vec<u8>,
    pub crc32: u32,
    pub timestamp: u64,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
}


#[allow(dead_code)]
#[derive(Debug)]
pub struct LaserData {
    pub version: u8,
    pub length: u16,
    pub time_interval: u16,
    pub dot_num: u16,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: u8,
    pub time_type: u8,
    pub reserved: Vec<u8>,
    pub crc32: u32,
    pub timestamp: u64,
    pub points: Vec<LaserPoint>,
}

pub fn distance(a: &Point3f, b: &Point3f) -> f32 {
    let diff = a - b;
    diff.norm()
}