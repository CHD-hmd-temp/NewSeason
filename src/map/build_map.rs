use crossbeam_channel::Receiver;
use crate::data_reader::udp_reader::SensorMessage;
use crate::prelude::LaserPoint;
use crate::calculator::imu;
use super::{occupancy_map, point_filter_2d};

#[allow(unused)]
pub fn build_occupancy_map(
    laser_data_rx: Receiver<SensorMessage<Vec<LaserPoint>>>,
    imu_data_rx: Receiver<SensorMessage<imu::ImuIntegrator>>,
    map: &mut occupancy_map::OccupancyGrid,
    map_tx: crossbeam_channel::Sender<occupancy_map::OccupancyGrid>,
) -> occupancy_map::OccupancyGrid {
    loop {
        // 1. 接收激光数据
        let laser_data_msg = laser_data_rx.recv().unwrap();
        if let Some(laser_data) = laser_data_msg.data {
            // 2. 接收IMU数据
            let imu_data_msg = imu_data_rx.recv().unwrap();
            if let Some(imu_data) = imu_data_msg.data {
                let timestamp = laser_data_msg.timestamp;
                // 3. 转换激光数据到2D坐标系
                let laser_frame_2d = point_filter_2d::transfer_laserframe_to_point2d(
                    &laser_data,
                    &imu_data,
                    timestamp,
                );
                // 4. 更新占据栅格地图
                map.integrate_scan(
                    &laser_frame_2d,
                    imu_data.yaw,
                );

                // 5. 发送更新后的地图（全量克隆）
                let map_clone = occupancy_map::OccupancyGrid {
                    log_odds: map.log_odds.clone(),
                    res: map.res,
                    width: map.width,
                    height: map.height,
                    origin: map.origin.clone(),
                };
                let _ = map_tx.send(map_clone);
            }
        }
    }
}