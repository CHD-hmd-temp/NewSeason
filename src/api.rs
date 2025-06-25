use core::time;

use pyo3::prelude::*;

#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct MavlinkArgs {
    #[pyo3(get)]
    pub time_boot_ms: u32,
    #[pyo3(get)]
    pub target_system: u8,
    #[pyo3(get)]
    pub target_component: u8,
    #[pyo3(get)]
    pub coordinate_frame: u8,
    #[pyo3(get)]
    pub type_mask: u16,
    #[pyo3(get)]
    pub x: f32,
    #[pyo3(get)]
    pub y: f32,
    #[pyo3(get)]
    pub z: f32,
    #[pyo3(get)]
    pub vx: f32,
    #[pyo3(get)]
    pub vy: f32,
    #[pyo3(get)]
    pub vz: f32,
    #[pyo3(get)]
    pub afx: f32,
    #[pyo3(get)]
    pub afy: f32,
    #[pyo3(get)]
    pub afz: f32,
    #[pyo3(get)]
    pub yaw: f32,
    #[pyo3(get)]
    pub yaw_rate: f32,
}

impl MavlinkArgs {
    pub fn default() -> Self {
        Self {
            time_boot_ms: 0,
            target_system: 0,
            target_component: 0,
            coordinate_frame: 0,
            type_mask: 0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        }
    }

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

fn get_mavlink_args(
    config_origin: &config::AppConfig
) -> Receiver<Vec<MavlinkArgs>>
{   
    use crate::data_reader::udp_reader::ConnectionState;
    use crossbeam_channel::unbounded;
    use std::net::UdpSocket;
    use crate::calculator::apf;
    use crate::data_reader::udp_reader;
    use crate::calculator::voxel_grid::voxel_grid_filter;
    use crate::octree::creat_octree::creat_octree_from_vec;
    use crate::prelude::{Point3f, ApfError};
    use crate::api::MavlinkArgs;

    println!("Start");

    let config = config_origin.clone();
    let (mavlink_tx, mavlink_rx) = unbounded();
    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind(config.hardware_config.lidar_socket.clone()).expect("Lidar Port bind failed");
        let apf_path_distance = config.octree_config.boundary * 0.7;
        let apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
        let apf_config = config.apf_config.clone();
        let octree_config = config.octree_config.clone();
        let mut mavlink_vec: Vec<MavlinkArgs> = Vec::new();
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
                    mavlink_vec.push(MavlinkArgs::default());
                    let _ = mavlink_tx.send(mavlink_vec.clone());
                }
            }

            let voxeled_points = voxel_grid_filter(&points, octree_config.voxel_size);
            let mut octree = creat_octree_from_vec(octree_config.boundary, octree_config.max_depth, voxeled_points);
            octree.optimize();
            let octree_map = octree.get_laser_points();

            let apf_path = apf::apf_plan_mavlink(
                Point3f::new(0.0, 0.0, 0.0),
                apf_goal,
                &octree_map,
                &apf_config
            );

            match apf_path {
                Ok(path) => {
                    let _ = mavlink_tx.send(path.clone());
                }
                Err(ApfError::LocalMinimum(partial_path)) => {
                    println!("Error: Local minimum reached");
                    let _ = mavlink_tx.send(partial_path);
                }
                Err(ApfError::MaxStepsReached(partial_path)) => {
                    println!("Error: Max steps reached");
                    let _ = mavlink_tx.send(partial_path);
                }
            };
        }
    });

    mavlink_rx
}

use crossbeam_channel::Receiver;
use crate::calculator::apf::l_shape_navigation;
use crate::calculator::crash_detector::crash_warn_for_octree;
use crate::calculator::crash_detector::obstacle_avoidance;
use crate::config;
use crate::data_reader;
#[pyfunction]
fn run_mid360(config_path: &str, callback: Py<PyAny>) -> PyResult<()> {
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });

    if !data_reader::sensor_detect::is_imu_sensor_online(&config) || !data_reader::sensor_detect::is_lidar_online(&config) {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }
    let mavlink_rx = get_mavlink_args(&config);

    // 创建一个新的线程来处理接收的消息
    std::thread::spawn(move || {
        for received in mavlink_rx {
            Python::with_gil(|py| {
                // 调用Python回调
                if let Err(e) = callback.call1(py, (received,)) {
                    e.print_and_set_sys_last_vars(py);
                }
            });
        }
    });

    Ok(())
}


/// Contains L-shape navigation and APF path planning
#[allow(non_snake_case)]
pub fn get_mavlink_args_EPIAC_special_edition(config_origin: &config::AppConfig) -> Receiver<Vec<MavlinkArgs>> {
    use crate::data_reader::udp_reader::ConnectionState;
    use crossbeam_channel::unbounded;
    use std::net::UdpSocket;
    use std::time::{Instant, Duration};
    use crate::calculator::apf;
    use crate::data_reader::udp_reader;
    use crate::calculator::voxel_grid::voxel_grid_filter;
    use crate::calculator::apf::NavState;
    use crate::octree::creat_octree::creat_octree_from_vec;
    use crate::prelude::{Point3f, ApfError};
    use crate::api::MavlinkArgs;

    println!("Start");

    let config = config_origin.clone();
    let (mavlink_tx, mavlink_rx) = unbounded();
    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind(config.hardware_config.lidar_socket.clone()).expect("Lidar Port bind failed");
        let apf_path_distance = config.octree_config.boundary * 0.7;
        let mut apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
        let apf_config = config.apf_config.clone();
        let octree_config = config.octree_config.clone();
        let l_shape_navigation_config = config.l_shape_navigation_config.clone();
        let mut mavlink_vec: Vec<MavlinkArgs> = Vec::new();
        let mut state = NavState::FollowHallway;
        let mut state_switch_flag: u8 = 0;
        let mut start_time = Instant::now();
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
                    mavlink_vec.push(MavlinkArgs::default());
                    let _ = mavlink_tx.send(mavlink_vec.clone());
                }
            }

            let voxeled_points = voxel_grid_filter(&points, octree_config.voxel_size);
            let mut octree = creat_octree_from_vec(octree_config.boundary, octree_config.max_depth, voxeled_points);
            octree.optimize();
            let octree_map = octree.get_laser_points();

            // obstacle avoidance
            let warn_trigger_distance = apf_config.d0;
            let tup_obstacle_result = crash_warn_for_octree(&octree, warn_trigger_distance);

            match tup_obstacle_result.0 {
                true => {
                    let mavlink_message = obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
                    let mavlink_vec = Vec::from([mavlink_message]);
                    let _ = mavlink_tx.send(mavlink_vec);
                    // Stop the thread for 2 seconds if an obstacle is detected
                    std::thread::sleep(Duration::from_secs(2));
                }
                false => {}
            }

            // L-shape navigation
            let next_state = l_shape_navigation(&octree_map, state.clone(), l_shape_navigation_config.d0);
            if next_state != state && state_switch_flag >= 10{
                state = next_state;
                state_switch_flag = 0;
                println!("State changed to {:?}", state);
            } else if next_state != state && state_switch_flag < 10 {
                state_switch_flag += 1;
            }
            else {
                state_switch_flag = 0;
            }

            match state {
                NavState::MoveToGoal | NavState::FollowHallway => {
                    apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
                }
                NavState::TurnLeft => {
                    apf_goal = Point3f::new(0.0, apf_path_distance, 0.0);
                }
                NavState::Hover => {
                    apf_goal = Point3f::new(0.0, 0.0, 0.0);
                }
            }
            
            // Check if 5 seconds have passed since the last path planning
            if start_time.elapsed() >= Duration::from_secs(5) {
                start_time = Instant::now(); // Reset the timer
                let apf_path = apf::apf_plan_mavlink(
                    Point3f::new(0.0, 0.0, 0.0),
                    apf_goal,
                    &octree_map,
                    &apf_config
                );

                match apf_path {
                    Ok(path) => {
                        let _ = mavlink_tx.send(path.clone());
                    }
                    Err(ApfError::LocalMinimum(partial_path)) => {
                        println!("Error: Local minimum reached");
                        let _ = mavlink_tx.send(partial_path);
                    }
                    Err(ApfError::MaxStepsReached(partial_path)) => {
                        println!("Error: Max steps reached");
                        let _ = mavlink_tx.send(partial_path);
                    }
                };
            }
        }
    });

    mavlink_rx
}

#[pyfunction]
fn run_mid360_special_edition(config_path: &str, callback: Py<PyAny>) -> PyResult<()> {
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });
    
    if !data_reader::sensor_detect::is_imu_sensor_online(&config) || !data_reader::sensor_detect::is_lidar_online(&config) {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }
    let mavlink_rx = get_mavlink_args_EPIAC_special_edition(&config);

    // 创建一个新的线程来处理接收的消息
    std::thread::spawn(move || {
        for received in mavlink_rx {
            Python::with_gil(|py| {
                // 调用Python回调
                if let Err(e) = callback.call1(py, (received,)) {
                    e.print_and_set_sys_last_vars(py);
                }
            });
        }
    });

    Ok(())
}

use std::sync::Mutex;
use std::collections::VecDeque;
use once_cell::sync::Lazy;
static MAVLINK_QUEUE: Lazy<Mutex<VecDeque<Vec<MavlinkArgs>>>> = Lazy::new(|| Mutex::new(VecDeque::new()));
#[pyfunction]
fn run_mid360_special_edition_with_queue(config_path: &str) -> PyResult<()> {
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });

    if !data_reader::sensor_detect::is_imu_sensor_online(&config) || !data_reader::sensor_detect::is_lidar_online(&config) {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }
    let mavlink_rx = get_mavlink_args_EPIAC_special_edition(&config);

    std::thread::spawn(move || {
        for received in mavlink_rx {
            MAVLINK_QUEUE.lock().unwrap().push_back(received);
        }
    });

    Ok(())
}

#[pyfunction]
fn poll_mavlink_args() -> PyResult<Option<Vec<MavlinkArgs>>> {
    let mut queue = MAVLINK_QUEUE.lock().unwrap();
    Ok(queue.pop_front())
}

#[pyfunction]
fn run_mid360_with_bevy(config_path: &str, special_edition: bool) -> PyResult<()> {
    use crate::visualization;
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });

    if !data_reader::sensor_detect::is_imu_sensor_online(&config) || !data_reader::sensor_detect::is_lidar_online(&config) {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }    

    let (lidar_rx, imu_rx, msgs_rx) = run_bevy_via_tunnel(&config, special_edition);
    visualization::rendering_components::run_bevy(
        &config,
        lidar_rx,
        imu_rx,
        msgs_rx,
    );  
    Ok(())
}

/// Unsafe, unfinished, untested
#[allow(unused)]
fn run_lidar(map_tx: crossbeam_channel::Sender<crate::map::occupancy_map::OccupancyGrid>) {
    use crossbeam_channel::unbounded;
    use std::time::Instant;
    use data_reader::udp_reader::{ConnectionState, SensorMessage};
    use crate::prelude::{LaserData, LaserPoint, ImuData};
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

    let mut map = crate::map::occupancy_map::OccupancyGrid {
        width: config.occupancy_map_config.width,
        height: config.occupancy_map_config.height,
        res: config.occupancy_map_config.res,
        origin: config.occupancy_map_config.origin,
        log_odds: vec![0.0; (config.occupancy_map_config.width * config.occupancy_map_config.height) as usize],
    };

    std::thread::spawn(move || {
        crate::map::build_map::build_occupancy_map(
            laser_data_rx,
            imu_data_rx,
            &mut map,
            map_tx,
        );
    });
}

fn create_imu_integrator(config: &config::AppConfig) -> (crate::calculator::imu::ImuIntegrator, crate::calculator::kalman_filter::ImuKalmanFilter) {
    use std::net::UdpSocket;
    use crate::calculator;
    let imu_bias = calculator::imu::imu_init(config.imu_config.init_time, &config);
    let imu_socket = UdpSocket::bind(&config.hardware_config.imu_socket).expect("Port bind failed");
    let imu_kalman = calculator::kalman_filter::imu_kalman_filter_init(
        imu_socket,
        config.kalman_filter_config.q,
        config.kalman_filter_config.r,
    );
    let imu_integrator = calculator::imu::ImuIntegrator::new(imu_bias);

    (imu_integrator, imu_kalman)
}

#[allow(unused)]
/// @brief Run Bevy via tunnel
/// @param config_origin: The configuration for the application
/// @param special_edition: Whether to use special edition or not
/// @return: A tuple containing three receivers for lidar, imu, and messages respectively
use data_reader::udp_reader::{ConnectionState, SensorMessage};
pub fn run_bevy_via_tunnel(
    config_origin: &config::AppConfig,
    special_edition: bool,
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
    use crate::calculator::kalman_filter;
    use crate::calculator::apf::NavState;
    use crate::data_reader::udp_reader;
    use crate::calculator::voxel_grid::voxel_grid_filter;
    use crate::octree::creat_octree::creat_octree_from_vec;
    use crate::prelude::{Point3f, ApfError};
    use crate::visualization::rendering_components::Msgs;


    println!("IMU initialization...");
    let imu_bias = imu::imu_init(config_origin.imu_config.init_time, &config_origin);
    let imu_socket = UdpSocket::bind(config_origin.hardware_config.imu_socket.clone()).expect("Port bind failed");
    let mut imu_kalman = kalman_filter::imu_kalman_filter_init(imu_socket, 0.01, 0.01);
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
    let (msgs_tx, msgs_rx) = unbounded();
    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind(config.hardware_config.lidar_socket.clone()).expect("Lidar Port bind failed");
        let apf_path_distance = config.octree_config.boundary * 0.7;
        let mut apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
        let apf_config = config.apf_config.clone();
        let octree_config = config.octree_config.clone();
        let l_shape_navigation_config = config.l_shape_navigation_config.clone();
        let mut state = NavState::FollowHallway;
        let mut state_switch_flag: u8 = 0;
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
                    let _ = msgs_tx.send(msgs_tx_msg);
                }
            }

            let voxeled_points = voxel_grid_filter(&points, octree_config.voxel_size);
            let mut octree = creat_octree_from_vec(octree_config.boundary, octree_config.max_depth, voxeled_points);
            octree.optimize();

            let octree_map = octree.get_laser_points();
            if special_edition {
                let next_state = l_shape_navigation(&octree_map, state.clone(), l_shape_navigation_config.d0);
                if next_state != state && state_switch_flag >= 10{
                    state = next_state;
                    state_switch_flag = 0;
                    println!("State changed to {:?}", state);
                } else if next_state != state && state_switch_flag < 10 {
                    state_switch_flag += 1;
                }
                else {
                    state_switch_flag = 0;
                }

                match state {
                    NavState::MoveToGoal | NavState::FollowHallway => {
                        apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
                    }
                    NavState::TurnLeft => {
                        apf_goal = Point3f::new(0.0, apf_path_distance, 0.0);
                    }
                    NavState::Hover => {
                        apf_goal = Point3f::new(0.0, 0.0, 0.0);
                    }
                }
            }

            let apf_path = apf::apf_plan(
                Point3f::new(0.0, 0.0, 0.0),
                apf_goal,
                &octree,
                &apf_config
            );

            let vec = match apf_path {
                Ok(path) => {
                    path
                }
                Err(ApfError::LocalMinimum(_)) => {
                    println!("Error: Local minimum reached");
                    Vec::new()
                }
                Err(ApfError::MaxStepsReached(_)) => {
                    println!("Error: Max steps reached");
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
            let _ = msgs_tx.send(msgs_tx_msg);
            let _ = lidar_tx.send(octree_tx_msg);
        }
    });

    (lidar_rx, imu_rx, msgs_rx)
}

pub fn _get_mavlink_args(
    config_origin: &config::AppConfig
) -> Receiver<Vec<MavlinkArgs>>     // msgs_r
{
    use crossbeam_channel::unbounded;
    use std::net::UdpSocket;
    use crate::calculator::apf;
    use crate::data_reader::udp_reader;
    use crate::calculator::voxel_grid::voxel_grid_filter;
    use crate::octree::creat_octree::creat_octree_from_vec;
    use crate::prelude::{Point3f, ApfError};

    let config = config_origin.clone();
    let (mavlink_tx, mavlink_rx) = unbounded();
    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind(config.hardware_config.lidar_socket.clone()).expect("Lidar Port bind failed");
        let apf_path_distance = config.octree_config.boundary * 0.7;
        let apf_goal = Point3f::new(apf_path_distance, 0.0, 0.0);
        let apf_config = config.apf_config.clone();
        let octree_config = config.octree_config.clone();
        let mut mavlink_vec: Vec<MavlinkArgs> = Vec::new();
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
                    mavlink_vec.push(MavlinkArgs::default());
                }
            }

            let voxeled_points = voxel_grid_filter(&points, octree_config.voxel_size);
            let mut octree = creat_octree_from_vec(octree_config.boundary, octree_config.max_depth, voxeled_points);
            octree.optimize();
            let octree_map = octree.get_laser_points();

            let apf_path = apf::apf_plan_mavlink(
                Point3f::new(0.0, 0.0, 0.0),
                apf_goal,
                &octree_map,
                &apf_config
            );

            match apf_path {
                Ok(path) => {
                    let _ = mavlink_tx.send(path.clone());
                }
                Err(ApfError::LocalMinimum(partial_path)) => {
                    println!("Error: Local minimum reached");
                    let _ = mavlink_tx.send(partial_path);
                }
                Err(ApfError::MaxStepsReached(partial_path)) => {
                    println!("Error: Max steps reached");
                    let _ = mavlink_tx.send(partial_path);
                }
            };
        }
    });

    mavlink_rx
}

#[pymodule]
fn world_without_anime(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MavlinkArgs>()?;
    m.add_function(wrap_pyfunction!(run_mid360, m)?)?;
    m.add_function(wrap_pyfunction!(run_mid360_special_edition, m)?)?;
    m.add_function(wrap_pyfunction!(run_mid360_with_bevy, m)?)?;
    m.add_function(wrap_pyfunction!(run_mid360_special_edition_with_queue, m)?)?;
    m.add_function(wrap_pyfunction!(poll_mavlink_args, m)?)?;
    Ok(())
}