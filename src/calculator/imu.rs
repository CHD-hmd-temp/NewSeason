use crate::prelude::*;
use crate::data_reader::udp_reader::{self, ConnectionState};
use crate::calculator::kalman_filter::ImuKalmanFilter;
use nalgebra::UnitQuaternion;
use std::time::{Duration, Instant};
use std::net::UdpSocket;
use bevy::prelude::*;

#[derive(Clone, Resource, Event, Copy)]
pub struct ImuIntegrator {
    pub imu_bias: ImuBias,
    orientation: UnitQuaternion<f32>,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl ImuIntegrator {
    pub fn new(imu_bias: ImuBias) -> Self {
        Self {
            imu_bias,
            orientation: UnitQuaternion::identity(),
            acc_x: 0.0,
            acc_y: 0.0,
            acc_z: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }

    pub fn update_with_kalman_filter(&mut self, imu_data: ImuData, dt: f32, kalman_filter: &mut ImuKalmanFilter) {
        let imu_data_sub_bias = self.subtract_imu_bias(imu_data);
        let mut filtered_data = kalman_filter.process(&imu_data_sub_bias);

        if is_acc_eqls_to_g(&filtered_data, self.imu_bias.clone()) {
            filtered_data = zero_acc_update(&mut filtered_data);
        }

        if is_gyro_eqls_to_zero(&filtered_data, self.imu_bias.clone()) {
            filtered_data = zero_gyro_update(&mut filtered_data);
        }
        
        // 1. 使用四元数更新姿态（陀螺仪积分）
        let delta_angle = Vector3f::new(
            filtered_data.gyro_x * dt,
            filtered_data.gyro_y * dt,
            filtered_data.gyro_z * dt,
        );
        let delta_q = UnitQuaternion::from_scaled_axis(delta_angle);
        self.orientation = delta_q * self.orientation; // 应用旋转
        self.orientation = UnitQuaternion::new_normalize(self.orientation.into_inner()); // 保持单位四元数

        // 2. 加速度计辅助姿态修正
        let accel = Vector3f::new(
            filtered_data.acc_x,
            filtered_data.acc_y,
            filtered_data.acc_z,
        ).normalize();

        // 计算目标姿态（将加速度对齐到重力方向）
        let target_up = Vector3f::z_axis();
        let measured_up = self.orientation.inverse() * accel; // 机体坐标系下的加速度
        let correction_q = UnitQuaternion::rotation_between(&measured_up, &target_up)
            .unwrap_or(UnitQuaternion::identity());

        // 互补滤波融合
        let alpha = 0.02; // 加速度计权重（1-alpha 是陀螺仪权重）
        self.orientation = self.orientation.slerp(&(correction_q * self.orientation), alpha);

        // 3. 坐标变换（加速度转换到世界坐标系）
        let rotation_matrix: Matrix3f = self.orientation.to_rotation_matrix().matrix().cast();
        let accel_body = Vector3f::new(
            filtered_data.acc_x,
            filtered_data.acc_y,
            filtered_data.acc_z,
        );
        let world_accel = rotation_matrix * accel_body;
        
        self.vx += world_accel.x * dt;
        self.vy += world_accel.y * dt;
        self.vz += world_accel.z * dt;

        self.x += self.vx * dt;
        self.y += self.vy * dt;
        self.z += self.vz * dt;

        // 4. 计算欧拉角
        let euler = self.orientation.euler_angles();
        self.roll = euler.0.to_degrees();
        self.pitch = euler.1.to_degrees();
        self.yaw = euler.2.to_degrees();

        // 5. 更新加速度计数据
        self.acc_x = filtered_data.acc_x;
        self.acc_y = filtered_data.acc_y;
        self.acc_z = filtered_data.acc_z;
    }

    pub fn subtract_imu_bias(&mut self, imu_data: ImuData) -> ImuData {
        let mut imu_data_sub_bias = imu_data;
        imu_data_sub_bias.gyro_x -= self.imu_bias.gyro_x;
        imu_data_sub_bias.gyro_y -= self.imu_bias.gyro_y;
        imu_data_sub_bias.gyro_z -= self.imu_bias.gyro_z;
        imu_data_sub_bias.acc_x -= self.imu_bias.acc_x;
        imu_data_sub_bias.acc_y -= self.imu_bias.acc_y;
        imu_data_sub_bias.acc_z -= self.imu_bias.acc_z;

        imu_data_sub_bias
    }
}

pub fn imu_init(init_time: u64) -> ImuBias {
    let socket_imu = UdpSocket::bind("0.0.0.0:56401").unwrap();
    let time = Instant::now();
    let mut count = 1;
    let mut accel_sum = Vector3f::zeros();
    let mut gyro_sum  = Vector3f::zeros();
    loop {
        let imu_data =  udp_reader::read_imu_data(&socket_imu);
            // Ok(data) => {
            //     accel_sum.x += data.acc_x;
            //     accel_sum.y += data.acc_y;
            //     accel_sum.z += data.acc_z;
            //     gyro_sum.x += data.gyro_x;
            //     gyro_sum.y += data.gyro_y;
            //     gyro_sum.z += data.gyro_z;
            //     count += 1;
            // }
            // Err(e) => {
            //     eprintln!("Error reading IMU data: {}", e);
            // }
            match imu_data.status {
                ConnectionState::Connected => {
                    if let Some(imu_data) = imu_data.data {
                        accel_sum.x += imu_data.acc_x;
                        accel_sum.y += imu_data.acc_y;
                        accel_sum.z += imu_data.acc_z;
                        gyro_sum.x += imu_data.gyro_x;
                        gyro_sum.y += imu_data.gyro_y;
                        gyro_sum.z += imu_data.gyro_z;
                        count += 1;
                    }
                }

                _ => {
                    eprintln!("Error reading IMU data: {:#?}", imu_data.status);
                    continue;
                }
            }

        if time.elapsed() > Duration::from_secs(init_time) {
            break;
        }
    }
    let mut imu_bias = ImuBias::default();
    let acc_bias: Vector3f = accel_sum / count as f32;
    let gyro_bias: Vector3f = gyro_sum / count as f32;
    imu_bias.acc_x = acc_bias.x;
    imu_bias.acc_y = acc_bias.y;
    imu_bias.acc_z = acc_bias.z;
    imu_bias.gyro_x = gyro_bias.x;
    imu_bias.gyro_y = gyro_bias.y;
    imu_bias.gyro_z = gyro_bias.z;

    imu_bias
}

fn is_acc_eqls_to_g(imu_data: &ImuData, imu_bias: ImuBias) -> bool {
    let acc = Vector3f::new(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
    let acc_bias = Vector3f::new(imu_bias.acc_x, imu_bias.acc_y, imu_bias.acc_z);
    let acc_with_gravity = Vector3f::new(
        imu_data.acc_x + imu_bias.acc_x,
        imu_data.acc_y + imu_bias.acc_y,
        imu_data.acc_z + imu_bias.acc_z
    );
    if acc.norm() < 0.05 {
        return true;
    };
    if (acc - acc_bias).norm() < 0.05 {
        return true;
    };
    if acc_with_gravity.norm() > 9.8 * 0.95 && acc_with_gravity.norm() < 9.8 * 1.05 {
        return true;
    };
    false
}

fn is_gyro_eqls_to_zero(imu_data: &ImuData, imu_bias: ImuBias) -> bool {
    let gyro = Vector3f::new(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
    let gyro_bias = Vector3f::new(imu_bias.gyro_x, imu_bias.gyro_y, imu_bias.gyro_z);
    if gyro.norm() < 0.05 {
        return true;
    };
    if (gyro - gyro_bias).norm() < 0.05 {
        return true;
    };
    false
}

fn zero_gyro_update(filtered_imu_data: &mut ImuData) -> ImuData {
    filtered_imu_data.gyro_x = 0.0;
    filtered_imu_data.gyro_y = 0.0;
    filtered_imu_data.gyro_z = 0.0;
    filtered_imu_data.clone()
}

fn zero_acc_update(filtered_imu_data: &mut ImuData) -> ImuData {
    filtered_imu_data.acc_x = 0.0;
    filtered_imu_data.acc_y = 0.0;
    filtered_imu_data.acc_z = 0.0;
    filtered_imu_data.clone()
}