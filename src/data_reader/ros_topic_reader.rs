use rosrust;
use crate::calculator::crash_detector::crash_warn_for_octree;
use crate::{msg, prelude::Point3f};
use crate::prelude::{LaserPoint, DronePosition};
use crate::{config, octree};
use crate::api::MavlinkArgs;
use std::time::{Duration, Instant};
use std::sync::{mpsc, Arc};
use std::thread;
use crate::calculator::{
    voxel_grid,
    crash_detector,
};

pub fn subscribe_lidar_point(
    config: config::AppConfig,
) {
    // Initialize node
    rosrust::init("metasequoia");

    let (tx, rx) = mpsc::channel::<Vec<LaserPoint>>();
    let (pos_tx, pos_rx) = mpsc::channel::<DronePosition>();
    let velocity_pub = rosrust::publish::<msg::geometry_msgs::Twist>("/new_season", 100).unwrap();

    // Create subscriber for lidar data
    // The subscriber is stopped when the returned object is destroyed
    let _subscriber_lidar = rosrust::subscribe("/livox/lidar", 100, move |v: msg::livox_ros_driver2::CustomMsg| {
        // Callback for handling received messages
        //rosrust::ros_info!("Received: {:?}", v.points);
        let points: Vec<LaserPoint> = v.points.iter().map(|p| {
            LaserPoint {
                coordinate: Point3f::new(p.x, p.y, p.z),
                reflectivity: p.reflectivity,
            }
        }).collect();
        tx.send(points).unwrap();
        
    }).unwrap();

    // Create subscriber for position data
    let _subscriber_position = rosrust::subscribe("/Odometry", 10, move |v: msg::nav_msgs::Odometry| {
        let position = Point3f::new(
            v.pose.pose.position.x as f32,
            v.pose.pose.position.y as f32,
            v.pose.pose.position.z as f32
        );
        
        // Convert quaternion to euler angles (simplified for yaw only)
        let quat = &v.pose.pose.orientation;
        let yaw = 2.0 * (quat.w * quat.z + quat.x * quat.y).atan2(
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        ) as f32;

        let orientation = crate::prelude::Vector3f::new(0.0, 0.0, yaw);
        let timestamp = v.header.stamp.sec as f64 + v.header.stamp.nsec as f64 * 1e-9;
        
        let drone_pos = DronePosition::new(position, orientation, timestamp);
        pos_tx.send(drone_pos).unwrap();
        
        rosrust::ros_info!("Received position: ({:.2}, {:.2}, {:.2}), yaw: {:.2}",
            position.x, position.y, position.z, yaw);
    }).unwrap();

    thread::spawn(move || {
        // read data from the channel
        let mut buffer: Vec<LaserPoint> = Vec::new();
        let mut last_update = Instant::now();
        let mut current_position = DronePosition::default();
        let publisher = Arc::new(velocity_pub);
        
        loop {
            // Check for new position data (non-blocking)
            match pos_rx.try_recv() {
                Ok(pos) => {
                    current_position = pos;
                }
                Err(_) => {} // No new position data, continue with old position
            }
            
            match rx.recv() {
                Ok(points) => {
                    buffer.extend(points);
                    if last_update.elapsed() >= Duration::from_millis(config.lidar_config.dt as u64) {
                        if !buffer.is_empty() {
                            // Perform voxel grid filtering
                            let voxeled_points = voxel_grid::voxel_grid_filter(
                                &buffer,
                                config.octree_config.voxel_size
                            );
                            
                            let mut local_octree = octree::creat_octree::creat_octree_from_vec(
                                config.octree_config.boundary,
                                config.octree_config.max_depth,
                                voxeled_points
                            );
                            local_octree.optimize();

                            let warn_trigger_distance = config.apf_config.d0;
                            let tuple_obstacle_result = crash_warn_for_octree(
                                &local_octree,
                                warn_trigger_distance
                            );

                            let mavlink_message = match tuple_obstacle_result.0 {
                                true => crash_detector::obstacle_avoidance(&tuple_obstacle_result.1, warn_trigger_distance),
                                false => MavlinkArgs::default(),
                            };
                            let twist_msg = msg::geometry_msgs::Twist {
                                linear: msg::geometry_msgs::Vector3 {
                                    x: mavlink_message.vx as f64,
                                    y: mavlink_message.vy as f64,
                                    z: mavlink_message.vz as f64,
                                    ..Default::default()
                                },
                                angular: msg::geometry_msgs::Vector3 {
                                    x: 0.0,
                                    y: 0.0,
                                    z: 0.0,
                                },
                            };
                            publisher.send(twist_msg).unwrap();

                            rosrust::ros_info!("Published velocity: ({:.2}, {:.2}, {:.2}) | Drone position: ({:.2}, {:.2}, {:.2})",
                                mavlink_message.vx,
                                mavlink_message.vy,
                                mavlink_message.vz,
                                current_position.position.x,
                                current_position.position.y,
                                current_position.position.z
                            );
                            
                            buffer.clear(); // Clear the buffer after processing
                        }

                        last_update = Instant::now();
                    }
                },
                Err(_) => {
                    rosrust::ros_warn!("Error receiving data from channel");
                    break;
                }
            }
            thread::sleep(Duration::from_millis(10)); // Prevent busy waiting
        }
    });

    // Block the thread until a shutdown signal is received
    rosrust::spin();
}

// Function to get current drone position (this could be expanded to use a shared state)
pub fn get_current_drone_position() -> DronePosition {
    // This is a placeholder implementation
    // In a real implementation, you might want to use Arc<Mutex<DronePosition>> 
    // to share the position between threads
    DronePosition::default()
}