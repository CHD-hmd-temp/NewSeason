use std::net::UdpSocket;
use pyo3::prelude::*;
use crate::calculator::voxel_grid;
use crate::calculator::crash_detector;
use crate::data_reader::udp_reader;
use crate::prelude::*;
use crate::data_reader;
use crate::visualization;
use crate::octree::creat_octree;

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

#[pyfunction]
fn run_mid360_with_bevy() -> PyResult<()> {
    if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }    

    visualization::rendering_components_octree::run_bevy();   
    Ok(())
}

#[pyfunction]
fn run_mid360(callback: Py<PyAny>) -> PyResult<()> {
    if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }

    let octree_config = OctreeConfig {
        boundary: 2.0,
        max_depth: 5,
        voxel_size: 0.05,
        frame_integration_time: 100,
    };

    let warn_trigger_distance = 0.5;

    let socket_laserpoint = UdpSocket::bind("0.0.0.0:56301").expect("couldn't bind to address");
    loop {
        let points = udp_reader::read_laserpoint(
            &socket_laserpoint,
            octree_config.frame_integration_time,
        ).unwrap();

        let voxeled_points = voxel_grid::voxel_grid_filter(
            &points,
            octree_config.voxel_size
        );
        let mut octree = creat_octree::creat_octree_from_vec(
            octree_config.boundary,
            octree_config.max_depth,
            voxeled_points,
        );

        octree.optimize();

        let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
        let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
        
        Python::with_gil(|py| -> PyResult<()> {
            callback.call1(py, (mavlink_message,))?;
            Ok(())
        })?;
    }
}

#[pymodule]
fn world_without_anime(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MavlinkArgs>()?;
    m.add_function(wrap_pyfunction!(run_mid360, m)?)?;
    m.add_function(wrap_pyfunction!(run_mid360_with_bevy, m)?)?;
    Ok(())
}