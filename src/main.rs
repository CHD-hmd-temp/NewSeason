mod octree;
mod data_reader;
mod visualization;
mod calculator;
mod prelude;
mod api;
mod config;
mod msg;

fn main() {
    let config_path = "config.toml";
    let config = config::load_config(config_path)
    .unwrap_or_else(|err| {
        eprintln!("Error loading `{}`: {}", config_path, err);
        config::AppConfig::default()
    });
    data_reader::ros_topic_reader::subscribe_lidar_point(config.clone());
}