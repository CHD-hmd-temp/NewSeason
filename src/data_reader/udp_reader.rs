use bevy::ecs::event::Event;
use byteorder::{LittleEndian, ReadBytesExt};
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use std::io::{Cursor, Error, ErrorKind, Read};
use crate::prelude::*;

#[derive(Event)]
pub struct SensorMessage<T> {
    pub status: ConnectionState,
    pub data: Option<T>,
    #[allow(dead_code)]
    pub timestamp: u64,
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub enum ConnectionState {
    Connected,
    Disconnected,
    Error(String),
}

pub fn parse_laserpoint(data: &[u8]) -> Result<LaserData, Error> {
    const HEADER_SIZE: usize = 36;
    const POINT_SIZE: usize = 14;

    if data.len() < HEADER_SIZE {
        return Err(Error::new(
            ErrorKind::UnexpectedEof,
            format!("Packet too small ({} < {})", data.len(), HEADER_SIZE),
        ));
    }

    let mut cursor = Cursor::new(data);

    // 解析数据包头部(LaserPoint&IMU数据包)
    let version = cursor.read_u8()?; // 0: 协议版本
    let length = cursor.read_u16::<LittleEndian>()?; // 1-2: UDP 包长度
    let time_interval = cursor.read_u16::<LittleEndian>()?; // 3-4: 时间间隔
    let dot_num = cursor.read_u16::<LittleEndian>()?; // 5-6: data包含点云数量
    let udp_cnt = cursor.read_u16::<LittleEndian>()?; // 7-8: UDP包计数
    let farme_cnt = cursor.read_u8()?; // 9: 帧计数
    let data_type = cursor.read_u8()?; // 10: 数据类型
    let time_type = cursor.read_u8()?; // 11: 时间戳类型
    
    // 12-23: 保留字段 - 读取12字节
    let mut reserved = vec![0u8; 12];
    cursor.read_exact(&mut reserved)?;
    
    let crc32 = cursor.read_u32::<LittleEndian>()?; // 24-27: CRC32校验码
    let timestamp = cursor.read_u64::<LittleEndian>()?; // 28-35: 时间戳

    if data_type != 0x01 {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!("Unsupported data type: {}", data_type),
        ));
    }

    let length_usize = length as usize;
    if length_usize > data.len() {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!("Length field ({}) exceeds actual data ({})", length, data.len()),
        ));
    }

    // 解析点云数据
    let payload = &data[HEADER_SIZE..length_usize];
    if payload.len() % POINT_SIZE != 0 || dot_num as usize != payload.len() / POINT_SIZE {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!("Payload size {} is not multiple of point size {}", payload.len(), POINT_SIZE),
        ));
    }

    let point_count = payload.len() / POINT_SIZE;
    let mut points = Vec::with_capacity(point_count);
    let mut payload_cursor = Cursor::new(payload);
    let minimun_distance = 0.1f32;

    for _ in 0..point_count {
        let x = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let y = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let z = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let reflectivity = payload_cursor.read_u8()?;
        let _tag = payload_cursor.read_u8()?;

        if x.abs() < minimun_distance && y.abs() < minimun_distance && z.abs() < minimun_distance {
            continue;
        }

        let coordinate = Point3f::new(x, y, z);

        points.push(LaserPoint {
            coordinate,
            reflectivity,
            //tag,
        });
    }

    let laser_data = LaserData {
        version,
        length,
        time_interval,
        dot_num,
        udp_cnt,
        frame_cnt: farme_cnt,
        data_type,
        time_type,
        reserved,
        crc32,
        timestamp,
        points,
    };

    Ok(laser_data)
}

/// Deprecated
#[allow(dead_code)]
fn read_laserpoint(socket: &std::net::UdpSocket, duration: u32) -> std::io::Result<Vec<LaserPoint>> {
    // let socket = UdpSocket::bind("0.0.0.0:56301")?;
    // println!("Listening for UDP data on port 56301..");

    let mut buf = [0; 65536];
    let mut data_buffer = Vec::new();
    let mut data_string = Vec::new();
    let start_time = Instant::now();

    loop {
        match socket.recv_from(&mut buf) {
            Ok((size, _addr)) => {
                data_buffer.extend_from_slice(&buf[..size]);

                match parse_laserpoint(&data_buffer) {
                    Ok(laser_data) => {
                        data_string.extend(laser_data.points);
                    }
                    Err(e) => {
                        eprintln!("Error parsing UDP packet: {}", e);
                    }
                }
                data_buffer.clear();

                if start_time.elapsed() > Duration::from_millis(duration as u64) {
                    return Ok(data_string);
                }
            }
            Err(e) => {
                eprintln!("Error receiving UDP packet: {}", e);
                continue;
            }
        }
    }
}

pub fn parse_imu(data: &[u8]) -> Option<ImuData> {
    if data.len() < 24 {
        return None;
    }

    let mut cursor = Cursor::new(data);
    let version = cursor.read_u8().unwrap();
    let length = cursor.read_u16::<LittleEndian>().unwrap();
    let time_interval = cursor.read_u16::<LittleEndian>().unwrap();
    let dot_num = cursor.read_u16::<LittleEndian>().unwrap();
    let udp_cnt = cursor.read_u16::<LittleEndian>().unwrap();
    let frame_cnt = cursor.read_u8().unwrap();
    let data_type = cursor.read_u8().unwrap();
    let time_type = cursor.read_u8().unwrap();
    let reserved = (0..12).map(|_| cursor.read_u8().unwrap()).collect::<Vec<u8>>();
    let crc32 = cursor.read_u32::<LittleEndian>().unwrap();
    let timestamp = cursor.read_u64::<LittleEndian>().unwrap();
    if data_type == 0 {
        let gyro_x = cursor.read_f32::<LittleEndian>().unwrap();
        let gyro_y = cursor.read_f32::<LittleEndian>().unwrap();
        let gyro_z = cursor.read_f32::<LittleEndian>().unwrap();
        let acc_x = cursor.read_f32::<LittleEndian>().unwrap() * 9.81;
        let acc_y = cursor.read_f32::<LittleEndian>().unwrap() * 9.81;
        let acc_z = cursor.read_f32::<LittleEndian>().unwrap() * 9.81;

        Some(ImuData {
            version,
            length,
            time_interval,
            dot_num,
            udp_cnt,
            frame_cnt,
            data_type,
            time_type,
            reserved,
            crc32,
            timestamp,
            gyro_x,
            gyro_y,
            gyro_z,
            acc_x,
            acc_y,
            acc_z,
        })
    }
    else {
        None
    }
}

#[allow(unused)]
pub fn read_pointcloud(
    socket: &UdpSocket,
    duration: u32,
) -> SensorMessage<Vec<LaserData>> {
    let mut buf = [0; 65536];
    let mut data_buffer = Vec::new();
    let start_time = Instant::now();
    let mut vec_laser_data = Vec::new();
    socket.set_read_timeout(Some(Duration::from_millis(50))).expect("Failed to set Lidar socket timeout");

    loop {
        match socket.recv_from(&mut buf) {
            Ok((size, _addr)) => {
                if size == 0 {
                    return SensorMessage {
                        status: ConnectionState::Disconnected,
                        data: None,
                        timestamp: 0,
                    };
                }
                data_buffer.extend_from_slice(&buf[..size]);

                match parse_laserpoint(&data_buffer) {
                    Ok(data) => {
                        vec_laser_data.push(data);
                    }
                    Err(e) => {
                        return SensorMessage {
                            status: ConnectionState::Error(e.to_string()),
                            data: None,
                            timestamp: 0,
                        };
                    }
                }
                data_buffer.clear();

                if start_time.elapsed() > Duration::from_millis(duration as u64) {
                    return SensorMessage {
                        status: ConnectionState::Connected,
                        data: Some(vec_laser_data),
                        timestamp: start_time.elapsed().as_millis() as u64,
                    };
                }
            }
            Err(e) => {
                match e.kind() {
                    ErrorKind::WouldBlock | ErrorKind::TimedOut => {
                        return SensorMessage {
                            status: ConnectionState::Disconnected,
                            data: None,
                            timestamp: 0,
                        };
                    }
                    _ => {
                        return SensorMessage {
                            status: ConnectionState::Error(e.to_string()),
                            data: None,
                            timestamp: 0,
                        };
                    }
                }
            }
        }
    }
}

pub fn read_imu_data(
    socket: &UdpSocket,
) -> SensorMessage<ImuData> {
    let mut buf = [0; 2048];
    let mut data_buffer = Vec::new();
    socket.set_read_timeout(Some(Duration::from_millis(20))).expect("Failed to set IMU socket timeout");

    match socket.recv_from(&mut buf) {
        Ok((size, _addr)) => {
            data_buffer.extend_from_slice(&buf[..size]);

            match parse_imu(&data_buffer) {
                Some(imu_data) => {
                    let timestamp = imu_data.timestamp;
                    return SensorMessage {
                        status: ConnectionState::Connected,
                        data: Some(imu_data),
                        timestamp: timestamp,
                    };
                }
                None => {
                    return SensorMessage {
                        status: ConnectionState::Error("Failed to parse IMU data".to_string()),
                        data: None,
                        timestamp: 0,
                    };
                }
            }
        }
        Err(e) => {
            match e.kind() {
                ErrorKind::WouldBlock | ErrorKind::TimedOut => {
                    return SensorMessage {
                        status: ConnectionState::Disconnected,
                        data: None,
                        timestamp: 0,
                    };
                }
                _ => {
                    return SensorMessage {
                        status: ConnectionState::Error(e.to_string()),
                        data: None,
                        timestamp: 0,
                    };
                }
            }
        }
    }
}