use nalgebra::Rotation3;

use crate::prelude::*;
use crate::calculator::imu::ImuIntegrator;

pub struct LaserFrame2D {
    pub points: Vec<Point2f>,
    pub sensor_pos: Point3f,
    #[allow(dead_code)]
    pub timestamp: u64,
}

pub fn transfer_laserframe_to_point2d(
    laser_frame: &Vec<LaserPoint>,
    imu_integrator: &ImuIntegrator,
    timestamp: u64,
) -> LaserFrame2D {
    // 获得当前无人机姿态
    // FLU坐标系
    let yaw = imu_integrator.yaw;
    let roll = imu_integrator.roll;
    let pitch = imu_integrator.pitch;

    // 将激光点云坐标转换为2D坐标系，只考虑与雷达同高度的点云
    // 根据姿态提取水平面点云切片，FLU坐标系

    // 1. 创建旋转矩阵
    let rotation_matrix = Rotation3::from_euler_angles(
        roll.to_radians(),
        pitch.to_radians(),
        yaw.to_radians()
    );

    let sensor_pos = Point3f::new(
        imu_integrator.x,
        imu_integrator.y,
        imu_integrator.z
    );

    // 2. 转换点云坐标并筛选阈值内的点云，认为其在水平面上
    let mut points_2d = Vec::new();
    for point in laser_frame {
        // 2.1 旋转点云坐标
        let rotated_point = rotation_matrix * Vector3f::new(
            point.coordinate.x,
            point.coordinate.y,
            point.coordinate.z
        );

        // 2.2 筛选点云
        if rotated_point.z.abs() < 0.1 {
            points_2d.push(Point2f::new(rotated_point.x, rotated_point.y));
        }
    }
    LaserFrame2D {
        points: points_2d,
        sensor_pos,
        timestamp,
    }
}