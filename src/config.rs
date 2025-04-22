use serde::{Deserialize, Serialize};
use crate::prelude::Point2f;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct AppConfig {
    pub hardware_config: HardwareConfig,
    pub lidar_config: LidarConfig,
    pub imu_config: ImuConfig,
    pub kalman_filter_config: KalmanFilterConfig,
    pub occupancy_map_config: OccupancyMapConfig,
    pub octree_config: OctreeConfig,
    pub apf_config: ApfConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct HardwareConfig {
    pub lidar_socket: String,
    pub imu_socket: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct LidarConfig {
    pub dt: u32, // in milliseconds
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ImuConfig {
    pub init_time: u64, // in seconds
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct KalmanFilterConfig {
    pub q: f32, // process noise covariance
    pub r: f32, // measurement noise covariance
    pub p: f32, // initial estimation error covariance
    pub k: f32, // Kalman gain
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OccupancyMapConfig {
    pub res: f32, // the size of each cell in meters
    pub width: usize,   // number of cells in the x direction
    pub height: usize,  // number of cells in the y direction
    #[serde(with = "point2_serde")]
    pub origin: Point2f,    // the origin of the grid in world coordinates
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OctreeConfig {
    pub boundary: f32,
    pub max_depth: u32,
    pub voxel_size: f32,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ApfConfig {
    pub k_att: f32,   // Attractive force gain
    pub k_rep: f32,   // Repulsive force gain
    pub d0: f32,      // Influence radius
    pub step_size: f32,   // Step size
    pub epsilon: f32,  // Goal radius
    pub max_steps: u32, // Maximum iteration steps
}


mod point2_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use crate::prelude::Point2f;

    pub fn serialize<S>(point: &Point2f, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        [point.x, point.y].serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Point2f, D::Error>
    where
        D: Deserializer<'de>,
    {
        let [x, y] = <[f32; 2]>::deserialize(deserializer)?;
        Ok(Point2f::new(x, y))
    }
}

impl Default for AppConfig {
    fn default() -> Self {
        AppConfig {
            hardware_config: HardwareConfig {
                lidar_socket: "0.0.0.0:56301".to_string(),
                imu_socket: "0.0.0.0:56401".to_string(),
            },
            lidar_config: LidarConfig {
                dt: 100, // 100ms
            },
            imu_config: ImuConfig {
                init_time: 1,
            },
            kalman_filter_config: KalmanFilterConfig {
                q: 0.01, // process noise covariance
                r: 0.01, // measurement noise covariance
                p: 1.0,  // initial estimation error covariance
                k: 0.5,  // Kalman gain
            },
            occupancy_map_config: OccupancyMapConfig {
                res: 0.05, // 5cm each cell
                width: 1000,
                height: 1000,
                origin: Point2f::new(0.0, 0.0), // origin at (0, 0)
            },
            octree_config: OctreeConfig {
                boundary: 2.0,
                max_depth: 5,
                voxel_size: 0.08,
            },
            apf_config: ApfConfig {
                k_att: 0.1,
                k_rep: 0.1,
                d0: 0.5,
                step_size: 0.05,
                epsilon: 0.1,
                max_steps: 250,
            },
        }
    }
}

use std::fs::read_to_string;
#[allow(dead_code)]
pub fn load_config(config_path: &str) -> Result<AppConfig, Box<dyn std::error::Error>> {
    // 创建默认配置
    let mut config = AppConfig::default();

    // 尝试读取配置文件
    if let Ok(config_file) = read_to_string(config_path) {
        // 使用合并策略：文件配置覆盖默认值
        let file_config: AppConfig = toml::from_str(&config_file)?;
        config.occupancy_map_config = file_config.occupancy_map_config;
        config.octree_config = file_config.octree_config;
        config.apf_config = file_config.apf_config;
        config.hardware_config = file_config.hardware_config;
        config.lidar_config = file_config.lidar_config;
        config.imu_config = file_config.imu_config;
        config.kalman_filter_config = file_config.kalman_filter_config;
    }

    Ok(config)
}

