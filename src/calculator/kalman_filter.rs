use crate::prelude::*;
use crate::data_reader;
use bevy::prelude::*;

#[derive(Clone, Resource)]
pub struct KalmanFilter1D {
    pub x: f32, // 状态量
    pub p: f32, // 状态协方差
    pub q: f32, // 状态转移协方差
    pub r: f32, // 观测噪声协方差
}

impl KalmanFilter1D {
    pub fn new(initial_x: f32, initial_p: f32, q: f32, r: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            q,
            r,
        }
    }

    fn predict(&mut self) {
        self.p += self.q;
    }

    fn update(&mut self, measurement: f32) {
        let k = self.p / (self.p + self.r);
        self.x += k * (measurement - self.x);
        self.p *= 1.0 - k;
    }
}

#[derive(Clone, Resource)]
pub struct ImuKalmanFilter {
    filters: [KalmanFilter1D; 6],
}

impl ImuKalmanFilter {
    pub fn new(
        init_values: &ImuData,
        q: f32,
        r: f32,
    ) -> Self {
        Self {
            filters: [
                KalmanFilter1D::new(init_values.gyro_x, 1.0, q, r),
                KalmanFilter1D::new(init_values.gyro_y, 1.0, q, r),
                KalmanFilter1D::new(init_values.gyro_z, 1.0, q, r),
                KalmanFilter1D::new(init_values.acc_x, 1.0, q, r),
                KalmanFilter1D::new(init_values.acc_y, 1.0, q, r),
                KalmanFilter1D::new(init_values.acc_z, 1.0, q, r),
            ],
        }
    }

    pub fn process(&mut self, imu_data: &ImuData) -> ImuData {
        let mut result = imu_data.clone();
        
        self.process_single(&mut result.acc_x, 0);
        self.process_single(&mut result.acc_y, 1);
        self.process_single(&mut result.acc_z, 2);

        self.process_single(&mut result.gyro_x, 3);
        self.process_single(&mut result.gyro_y, 4);
        self.process_single(&mut result.gyro_z, 5);
        
        result
    }

    fn process_single(&mut self, value: &mut f32, index: usize) {
        self.filters[index].predict();
        self.filters[index].update(*value);
        *value = self.filters[index].x;
    }
}

pub fn imu_kalman_filter_init(
    imu_socket: std::net::UdpSocket,
    q: f32,
    r: f32,
) -> ImuKalmanFilter {
    let init_values = data_reader::udp_reader::read_imu(&imu_socket).unwrap();
    ImuKalmanFilter::new(&init_values, q, r)
}