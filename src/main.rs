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

    //visualization::rendering_components_octree::run_bevy();   
    visualization::rendering_components_channel::run_bevy();
    //test_icp();
    //test_imu();
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

#[allow(dead_code)]
fn test_imu() {
    use std::time::Instant;
    let imu_bias = calculator::imu::imu_init(5);
    let mut imu_integrator = calculator::imu::ImuIntegrator::new(imu_bias);
    let thread_time = Instant::now();
    let imu_socket = std::net::UdpSocket::bind("0.0.0.0:56401").unwrap();
    let q = 0.01;
    let r = 0.01;
    let mut kalman = calculator::kalman_filter::imu_kalman_filter_init(imu_socket, q, r);
    while (Instant::now() - thread_time).as_secs() < 60 {
        let dt = Instant::now();
        let imu_socket = std::net::UdpSocket::bind("0.0.0.0:56401").unwrap();
        let imu_data = data_reader::udp_reader::read_imu(&imu_socket).unwrap();
        imu_integrator.update_with_kalman_filter(imu_data, dt.elapsed().as_secs_f32(), &mut kalman);
    }

    println!("pitch: {}, roll: {}, yaw: {}", imu_integrator.pitch, imu_integrator.roll, imu_integrator.yaw);
    println!("vx: {}, vy: {}, vz: {}", imu_integrator.vx, imu_integrator.vy, imu_integrator.vz);
    println!("x: {}, y: {}, z: {}", imu_integrator.x, imu_integrator.y, imu_integrator.z);
}