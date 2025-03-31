mod octree;
mod data_reader;
mod visualization;
mod calculator;
mod prelude;
mod python_api;

fn main() {
    if !data_reader::sensor_detect::is_imu_sensor_online() || !data_reader::sensor_detect::is_lidar_online() {
        panic!("Sensors are not online");
    }

    visualization::rendering_components::run_bevy();
}


#[allow(dead_code)]
fn test_icp() {
    let mut icp = calculator::icp::ICPOdometry::new(calculator::icp::ICPConfig {
        num_samples: 500,
        max_correspondence_dist: 1.0,
        max_iterations: 20,
        tolerance: 1e-5,
    });

    for _ in 0..10 {
        let socket = std::net::UdpSocket::bind("0.0.0.0:56301").unwrap();
        let points = data_reader::udp_reader::read_laserpoint(&socket, 100).unwrap();
        let pose = icp.process_frame(&points);
        println!("Pose: {:?}", pose);
    }
}