use nalgebra::{Matrix3, Vector3, Point3};
use bevy::ecs::system::Resource;

pub type Point3f = Point3<f32>;
pub type Vector3f = Vector3<f32>;
pub type Matrix3f = Matrix3<f32>;

pub struct NodeForRender {
    pub coordinate: Point3f,
    pub reflectivity: u8,
}

#[derive(Debug, Clone, Copy)]
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

#[derive(Debug, Clone, Copy, Resource)]
pub struct ImuBias {
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}

impl Default for ImuBias {
    fn default() -> Self {
        Self {
            acc_x: 0.0,
            acc_y: 0.0,
            acc_z: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
        }
    }
    
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