use pyo3::prelude::*;
use crate::data_reader;
use crate::visualization;

#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct MavlinkArgs {
    pub time_boot_ms: u32,
    pub target_system: u8,
    pub target_component: u8,
    pub coordinate_frame: u8,
    pub type_mask: u16,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub afx: f32,
    pub afy: f32,
    pub afz: f32,
    pub yaw: f32,
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
fn run_mid360() -> PyResult<()> {
    if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
        return Err(pyo3::exceptions::PyException::new_err("IMU or LiDAR is not online"));
    }    

    visualization::rendering_components_octree::run_bevy();   
    Ok(())
}

#[pymodule]
fn world_without_anime(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MavlinkArgs>()?;
    m.add_function(wrap_pyfunction!(run_mid360, m)?)?;
    Ok(())
}