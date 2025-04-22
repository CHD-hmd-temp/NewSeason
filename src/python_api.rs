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
) -> Receiver<Vec<MavlinkArgs>>     // msgs_r
{   
    use crate::data_reader::udp_reader::ConnectionState;
    use crossbeam_channel::unbounded;
    use std::net::UdpSocket;
    use crate::calculator::apf;
    use crate::data_reader::udp_reader;
    use crate::calculator::voxel_grid::voxel_grid_filter;
    use crate::octree::creat_octree::creat_octree_from_vec;
    use crate::prelude::Point3f;
    use crate::python_api::MavlinkArgs;

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

            let apf_path = apf::apf_plan_mavlink(
                Point3f::new(0.0, 0.0, 0.0),
                apf_goal,
                &octree,
                &apf_config,
            );

            match apf_path {
                Ok(path) => {
                    let _ = mavlink_tx.send(path.clone());
                }
                Err(e) => {
                    println!("Error: {:?}", e);
                    mavlink_vec.push(MavlinkArgs::default());
                    let _ = mavlink_tx.send(mavlink_vec.clone());
                }
            };
        }
    });

    mavlink_rx
}

use crossbeam_channel::Receiver;
use crate::config;
use crate::data_reader;
#[pyfunction]
fn run_mid360(config_path: &str, callback: Py<PyAny>) -> PyResult<()> {
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });
    
    if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
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

// #[pyfunction]
// fn run_mid360_with_bevy() -> PyResult<()> {
//     if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
//         return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
//     }    

//     visualization::rendering_components::run_bevy();   
//     Ok(())
// }

// #[pyfunction]
// fn run_mid360(callback: Py<PyAny>) -> PyResult<()> {
//     if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
//         return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
//     }

//     let octree_config = OctreeConfig {
//         boundary: 2.0,
//         max_depth: 5,
//         voxel_size: 0.05,
//         frame_integration_time: 100,
//     };

//     let warn_trigger_distance = 0.5;

//     let socket_laserpoint = UdpSocket::bind("0.0.0.0:56301").expect("couldn't bind to address");
//     loop {
//         let points = udp_reader::read_laserpoint(
//             &socket_laserpoint,
//             octree_config.frame_integration_time,
//         ).unwrap();

//         let voxeled_points = voxel_grid::voxel_grid_filter(
//             &points,
//             octree_config.voxel_size
//         );
//         let mut octree = creat_octree::creat_octree_from_vec(
//             octree_config.boundary,
//             octree_config.max_depth,
//             voxeled_points,
//         );

//         octree.optimize();

//         let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
//         let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
        
//         Python::with_gil(|py| -> PyResult<()> {
//             callback.call1(py, (mavlink_message,))?;
//             Ok(())
//         })?;
//     }
// }

#[pymodule]
fn world_without_anime(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MavlinkArgs>()?;
    m.add_function(wrap_pyfunction!(run_mid360, m)?)?;
    Ok(())
}