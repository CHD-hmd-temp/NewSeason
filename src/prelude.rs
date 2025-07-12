use nalgebra::{Matrix3, Vector3, Point3, Point2};
use bevy::ecs::system::Resource;

pub type Point3f = Point3<f32>;
pub type Vector3f = Vector3<f32>;
pub type Matrix3f = Matrix3<f32>;
pub type Point2f = Point2<f32>;

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

#[derive(Debug)]
pub enum ApfError<T> {
    LocalMinimum(Vec<T>),
    MaxStepsReached(Vec<T>),
}

#[allow(dead_code)]
#[derive(Debug, Clone, Resource)]
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
impl ImuData {
    pub fn new() -> Self {
        Self {
            version: 0,
            length: 0,
            time_interval: 0,
            dot_num: 0,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: 0,
            time_type: 0,
            reserved: vec![0; 12],
            crc32: 0,
            timestamp: 0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            acc_x: 0.0,
            acc_y: 0.0,
            acc_z: 0.0,
        }
    }
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

#[allow(dead_code)]
impl LaserData {
    pub fn new() -> Self {
        Self {
            version: 0,
            length: 0,
            time_interval: 0,
            dot_num: 0,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: 0,
            time_type: 0,
            reserved: vec![0; 12],
            crc32: 0,
            timestamp: 0,
            points: Vec::new(),
        }
    }
}

pub fn distance(a: &Point3f, b: &Point3f) -> f32 {
    let diff = a - b;
    diff.norm()
}

#[derive(Debug, Clone, Copy)]
pub struct LivoxPointCloud2 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub tag: u8,
    pub line: u8,
    pub timestamp: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct DronePosition {
    pub position: Point3f,
    pub orientation: Vector3f, // 简化的欧拉角表示 (roll, pitch, yaw)
    pub timestamp: f64,
}

impl DronePosition {
    pub fn new(position: Point3f, orientation: Vector3f, timestamp: f64) -> Self {
        Self {
            position,
            orientation,
            timestamp,
        }
    }
    
    pub fn default() -> Self {
        Self {
            position: Point3f::new(0.0, 0.0, 0.0),
            orientation: Vector3f::new(0.0, 0.0, 0.0),
            timestamp: 0.0,
        }
    }
}