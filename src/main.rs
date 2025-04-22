use crossbeam_channel::Receiver;
mod octree;
mod data_reader;
mod visualization;
mod calculator;
mod prelude;
mod python_api;
mod map;
mod config;

fn main() {
    if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
        panic!("Sensors are not online");
    }

    let config_path = "H:/Project/Drones/src/WorldWithoutAnime/config.toml";
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });

    let (lidar_rx, imu_rx, msgs_rx) = run_bevy_via_tunnel(&config);
    visualization::rendering_components::run_bevy(
        &config,
        lidar_rx,
        imu_rx,
        msgs_rx,
    );
}

/// Unsafe, unfinished, untested
#[allow(unused)]
fn run_lidar(map_tx: crossbeam_channel::Sender<map::occupancy_map::OccupancyGrid>) {
    use crossbeam_channel::unbounded;
    use std::time::Instant;
    use data_reader::udp_reader::{ConnectionState, SensorMessage};
    use prelude::{LaserData, LaserPoint, ImuData};
    let config_path = "config.toml";
    let config = config::load_config(config_path).unwrap_or_else(|_| {
        eprintln!("Failed to load config, using default values.");
        config::AppConfig::default()
    });

    // Initialize the IMU bias and Kalman filter
    let (mut imu_integrator, mut imu_kalman) = create_imu_integrator(&config);

    let timestamp = Instant::now();
    let (laser_data_tx, laser_data_rx) = unbounded();
    std::thread::spawn(move || {
        let socket = std::net::UdpSocket::bind(config.hardware_config.lidar_socket).unwrap();
        loop {
            let vec_laserdata_msg: SensorMessage<Vec<LaserData>> = data_reader::udp_reader::read_pointcloud(
                &socket,
                config.lidar_config.dt
            );

            let mut points = Vec::new();

            match vec_laserdata_msg.status {
                ConnectionState::Connected => {
                    if let Some(data) = vec_laserdata_msg.data {
                        for laserdata_frame in data {
                            points.extend(laserdata_frame.points);
                        }
                    }
                    let laser_data_msg: SensorMessage<Vec<LaserPoint>> = SensorMessage {
                        status: vec_laserdata_msg.status,
                        data: Some(points),
                        timestamp: timestamp.elapsed().as_millis() as u64,
                    };
                    let _ = laser_data_tx.send(laser_data_msg);
                }
                ConnectionState::Disconnected | ConnectionState::Error(_) => {
                    let laser_data_tx_msg: SensorMessage<Vec<LaserPoint>> = SensorMessage {
                        status: vec_laserdata_msg.status.clone(),
                        data: None,
                        timestamp: timestamp.elapsed().as_millis() as u64,
                    };
                    let _ = laser_data_tx.send(laser_data_tx_msg);
                }
            }
        }
    });
    let (imu_data_tx, imu_data_rx) = unbounded();
    std::thread::spawn(move || {
        let socket = std::net::UdpSocket::bind(config.hardware_config.imu_socket).unwrap();
        loop {
            let dt = Instant::now();
            let imu_data_msg: SensorMessage<ImuData> = data_reader::udp_reader::read_imu_data(&socket);
            match imu_data_msg.status {
                ConnectionState::Connected => {
                    if let Some(imu_data) = imu_data_msg.data {
                        imu_integrator.update_with_kalman_filter(imu_data, dt.elapsed().as_secs_f32(), &mut imu_kalman);
                        let imu_tx_msg = SensorMessage {
                            status: imu_data_msg.status,
                            data: Some(imu_integrator.clone()),
                            timestamp: timestamp.elapsed().as_millis() as u64,
                        };
                        let _ = imu_data_tx.send(imu_tx_msg);
                    }
                }
                ConnectionState::Disconnected | ConnectionState::Error(_) => {
                    let imu_tx_msg = SensorMessage {
                        status: imu_data_msg.status,
                        data: Some(imu_integrator.clone()),
                        timestamp: timestamp.elapsed().as_millis() as u64,
                    };
                    let _ = imu_data_tx.send(imu_tx_msg);
                }
            }
        }
    });

    let mut map = map::occupancy_map::OccupancyGrid {
        width: config.occupancy_map_config.width,
        height: config.occupancy_map_config.height,
        res: config.occupancy_map_config.res,
        origin: config.occupancy_map_config.origin,
        log_odds: vec![0.0; (config.occupancy_map_config.width * config.occupancy_map_config.height) as usize],
    };

    std::thread::spawn(move || {
        map::build_map::build_occupancy_map(
            laser_data_rx,
            imu_data_rx,
            &mut map,
            map_tx,
        );
    });
}

fn create_imu_integrator(config: &config::AppConfig) -> (calculator::imu::ImuIntegrator, calculator::kalman_filter::ImuKalmanFilter) {
    use std::net::UdpSocket;
    let imu_bias = calculator::imu::imu_init(config.imu_config.init_time);
    let imu_socket = UdpSocket::bind(&config.hardware_config.imu_socket).expect("Port bind failed");
    let imu_kalman = calculator::kalman_filter::imu_kalman_filter_init(
        imu_socket,
        config.kalman_filter_config.q,
        config.kalman_filter_config.r,
    );
    let imu_integrator = calculator::imu::ImuIntegrator::new(imu_bias);

    (imu_integrator, imu_kalman)
}

use data_reader::udp_reader::{ConnectionState, SensorMessage};
pub fn run_bevy_via_tunnel(
    config_origin: &config::AppConfig
) -> (
    Receiver<SensorMessage<crate::octree::octree::Octree>>,   // lidar_rx
    Receiver<SensorMessage<crate::calculator::imu::ImuIntegrator>>, // imu_rx
    Receiver<SensorMessage<crate::visualization::rendering_components::Msgs>>,     // msgs_rx
) {
    use crossbeam_channel::unbounded;
    use std::net::UdpSocket;
    use std::time::Instant;
    use bevy::prelude::Vec3;
    use crate::calculator::imu;
    use crate::calculator::coordinate_switch;
    use crate::calculator::apf;
    use crate::calculator::crash_detector;
    use crate::data_reader::udp_reader;
    use crate::calculator::voxel_grid::voxel_grid_filter;
    use crate::octree::creat_octree::creat_octree_from_vec;
    use crate::prelude::Point3f;
    use crate::visualization::rendering_components::Msgs;


    println!("IMU initialization...");
    let imu_bias = imu::imu_init(config_origin.imu_config.init_time);
    let imu_socket = UdpSocket::bind(config_origin.hardware_config.imu_socket.clone()).expect("Port bind failed");
    let mut imu_kalman = calculator::kalman_filter::imu_kalman_filter_init(imu_socket, 0.01, 0.01);
    let mut imu_integrator = imu::ImuIntegrator::new(imu_bias);

    let config = config_origin.clone();
    let (imu_tx, imu_rx) = unbounded();
    std::thread::spawn(move || {
        let imu_socket = UdpSocket::bind(config.hardware_config.imu_socket).expect("Imu Port bind failed");
        loop {
            let dt = Instant::now();
            let imu_data_msg = udp_reader::read_imu_data(&imu_socket);
            match imu_data_msg.status {
                ConnectionState::Connected => {
                    if let Some(imu_data) = imu_data_msg.data {
                        imu_integrator.update_with_kalman_filter(imu_data, dt.elapsed().as_secs_f32(), &mut imu_kalman);
                        let imu_tx_msg = SensorMessage {
                            status: imu_data_msg.status,
                            data: Some(imu_integrator.clone()),
                            timestamp: imu_data_msg.timestamp,
                        };
                        let _ = imu_tx.send(imu_tx_msg);
                    }
                }

                ConnectionState::Disconnected | ConnectionState::Error(_) => {
                    let imu_tx_msg = SensorMessage {
                        status: imu_data_msg.status,
                        data: Some(imu_integrator.clone()),
                        timestamp: imu_data_msg.timestamp,
                    };
                    let _ = imu_tx.send(imu_tx_msg);
                }
            }
        }
    });

    let (lidar_tx, lidar_rx) = unbounded();
    let (msg_tx, msgs_rx) = unbounded();
    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind(config.hardware_config.lidar_socket.clone()).expect("Lidar Port bind failed");
        let apf_path_distance = config.octree_config.boundary * 0.7;
        let apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
        let apf_config = config.apf_config.clone();
        let octree_config = config.octree_config.clone();
        loop {
            let vec_laserdata = udp_reader::read_pointcloud(
                &lidar_socket,
                config.lidar_config.dt.clone(),
            );

            let mut points = Vec::new();

            match vec_laserdata.status {
                ConnectionState::Connected => {
                    if let Some(data) = vec_laserdata.data {
                        for laserdata_frame in data {
                            points.extend(laserdata_frame.points);
                        }
                    }
                }
                ConnectionState::Disconnected | ConnectionState::Error(_) => {
                    let lidar_tx_msg = SensorMessage {
                        status: vec_laserdata.status.clone(),
                        data: None,
                        timestamp: vec_laserdata.timestamp.clone(),
                    };
                    let _ = lidar_tx.send(lidar_tx_msg);
                    let msgs_tx_msg = SensorMessage {
                        status: vec_laserdata.status.clone(),
                        data: None,
                        timestamp: vec_laserdata.timestamp.clone(),
                    };
                    let _ = msg_tx.send(msgs_tx_msg);
                }
            }

            let voxeled_points = voxel_grid_filter(&points, octree_config.voxel_size);
            let mut octree = creat_octree_from_vec(octree_config.boundary, octree_config.max_depth, voxeled_points);
            octree.optimize();

            let apf_path = apf::apf_plan(
                Point3f::new(0.0, 0.0, 0.0),
                apf_goal,
                &octree,
                &apf_config,
            );

            let vec = match apf_path {
                Ok(path) => {
                    path
                }
                Err(e) => {
                    println!("Error: {:?}", e);
                    Vec::new()
                }
            };

            let warn_trigger_distance = apf_config.d0;
            let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
            let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
            let velocity = match mavlink_message.type_mask {
                0b0000001000000000 => {
                    let (x, y, z) = coordinate_switch::frd_to_bevy(mavlink_message.vx, mavlink_message.vy, mavlink_message.vz);
                    Vec3::new(x, y, z)
                }
                _ => Vec3::ZERO,
            };

            let msg = Msgs {
                apf_path: vec,
                velocity,
            };
            let octree_tx_msg = SensorMessage {
                status: vec_laserdata.status.clone(),
                data: Some(octree.clone()),
                timestamp: vec_laserdata.timestamp,
            };
            let msgs_tx_msg = SensorMessage {
                status: vec_laserdata.status.clone(),
                data: Some(msg.clone()),
                timestamp: vec_laserdata.timestamp,
            };
            let _ = msg_tx.send(msgs_tx_msg);
            let _ = lidar_tx.send(octree_tx_msg);
        }
    });

    (lidar_rx, imu_rx, msgs_rx)
}