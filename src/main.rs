mod octree;
mod data_reader;
mod visualization;
mod calculator;
mod prelude;
mod api;
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

    let (lidar_rx, imu_rx, msgs_rx) = api::run_bevy_via_tunnel(&config, false);
    visualization::rendering_components::run_bevy(
        &config,
        lidar_rx,
        imu_rx,
        msgs_rx,
    );
}