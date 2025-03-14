use serde::Deserialize;

#[allow(dead_code)]
#[derive(Debug, Deserialize, Clone)]
pub struct LaserPoint {
    #[serde(rename = "X")]
    pub x: f32,
    #[serde(rename = "Y")]
    pub y: f32,
    #[serde(rename = "Z")]
    pub z: f32,
    #[serde(rename = "Reflectivity")]
    pub reflectivity: u8,
    // #[serde(rename = "Tag")]
    // pub tag: u8,
}

impl LaserPoint {
    pub fn new(x: f32, y: f32, z: f32, reflectivity: u8) -> Self {
        Self {
            x,
            y,
            z,
            reflectivity,
        }
    }
}

#[derive(PartialEq)]
pub struct PlanePosition {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    // 向量减法
    pub fn sub(&self, other: &Point3) -> Point3 {
        Point3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    // 向量模长
    pub fn norm(&self) -> f32 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    // 单位向量
    pub fn normalize(&self) -> Option<Point3> {
        let n = self.norm();
        if n == 0.0 {
            None
        } else {
            Some(Point3 {
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            })
        }
    }
}

pub struct NodeForRender {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub reflectivity: u8,
}