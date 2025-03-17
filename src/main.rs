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

    visualization::rendering_components_octree::run_bevy();   
    //test_icp();
}

#[allow(dead_code)]
fn async_main() {
    // 创建 Tokio 运行时
    let rt = tokio::runtime::Runtime::new().unwrap();

    // 在 Tokio 运行时中运行异步任务
    rt.spawn(async {
        let imu_socket = tokio::net::UdpSocket::bind("0.0.0.0:56401")
            .await
            .expect("Port bind failed");
        data_reader::udp_reader::read_imu_async(&imu_socket).await.unwrap();
        
    });
    // 在主线程运行 Bevy
    visualization::rendering_components_octree::run_bevy();

    // 保持 Tokio 运行时存活（注意：Bevy 可能会无限阻塞，此代码可能无法到达）
    // 通常 Bevy 会接管主线程，Tokio 任务在后台运行
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