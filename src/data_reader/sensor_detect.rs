#![allow(dead_code)]
use std::net::UdpSocket;
use crate::data_reader::udp_reader;
use crate::config::AppConfig;

pub fn is_imu_sensor_online(config: &AppConfig) -> bool {
    // Check if port 56401 is open
    let socket = UdpSocket::bind(config.hardware_config.imu_socket.clone()).expect("Port bind failed");
    socket.set_read_timeout(Some(std::time::Duration::from_secs(1))).expect("Set timeout failed");

    let mut buf = [0; 65536];
    let mut data_buffer = Vec::new();

    match socket.recv_from(&mut buf) {
        Ok((size, _addr)) => {
            data_buffer.extend_from_slice(&buf[..size]);
            match udp_reader::parse_imu(&data_buffer) {
                _imu_data => {
                    data_buffer.clear();
                    return true;
                }
            }
        }
        Err(_) => {
            //eprintln!("Error receiving UDP packet: {}", e);
            drop(socket);
            data_buffer.clear();
            return false;
        }
    }
}

pub fn is_lidar_online(config: &AppConfig) -> bool {
    // Check if port 56301 is open
    // Timeout is set to 1 second
    let socket = UdpSocket::bind(config.hardware_config.lidar_socket.clone()).expect("Port bind failed");
    socket.set_read_timeout(Some(std::time::Duration::from_secs(1))).expect("Set timeout failed");

    let mut buf = [0; 65536];
    let mut data_buffer = Vec::new();

    match socket.recv_from(&mut buf) {
        Ok((size, _addr)) => {
            data_buffer.extend_from_slice(&buf[..size]);
            match udp_reader::parse_laserpoint(&data_buffer) {
                _laser_data => {
                    data_buffer.clear();
                    return true;
                }
            }
        }
        Err(_) => {
            drop(socket);
            data_buffer.clear();
            return false;
        }
    }
}