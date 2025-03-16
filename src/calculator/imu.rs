use crate::prelude::*;
use crate::data_reader::udp_reader;
use std::time::{Duration, Instant};
use std::net::UdpSocket;

pub fn imu_init() -> ImuBias {
    let socket_imu = UdpSocket::bind("0.0.0.0:56401").unwrap();
    let time = Instant::now();
    let mut count = 1;
    let mut accel_sum = Vector3f::zeros();
    let mut gyro_sum  = Vector3f::zeros();
    loop {
        match udp_reader::read_imu(&socket_imu) {
            Ok(data) => {
                accel_sum.x += data.acc_x;
                accel_sum.y += data.acc_y;
                accel_sum.z += data.acc_z;
                gyro_sum.x += data.gyro_x;
                gyro_sum.y += data.gyro_y;
                gyro_sum.z += data.gyro_z;
                count += 1;
            }
            Err(e) => {
                eprintln!("Error reading IMU data: {}", e);
            }
        }
        if time.elapsed() > Duration::from_secs(10) {
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