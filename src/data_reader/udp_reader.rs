use byteorder::{LittleEndian, ReadBytesExt};
use std::time::{Duration, Instant};
use std::io::{Cursor, Error, ErrorKind};
use crate::prelude::*;

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
    let length = cursor.read_u16::<LittleEndian>()?; // 1: UDP 包长度
    let time_interval = cursor.read_u16::<LittleEndian>()?; // 3-4: 时间间隔
    let dot_num = cursor.read_u16::<LittleEndian>()?; // 5-6: data包含点云数量
    let udp_cnt = cursor.read_u16::<LittleEndian>()?; // 7-8: UDP包计数
    let farme_cnt = cursor.read_u8()?; // 9: 帧计数
    let data_type = cursor.read_u8()?; // 10: 数据类型
    let time_type = cursor.read_u8()?; // 11: 时间戳类型
    let reserved1 = cursor.read_u8()?; // 12: 保留字段
    let crc32 = cursor.read_u32::<LittleEndian>()?; // 13-16: CRC32校验码
    let timestamp = cursor.read_u64::<LittleEndian>()?; // 17-24: 时间戳

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

    for _ in 0..point_count {
        let x = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let y = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let z = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let reflectivity = payload_cursor.read_u8()?;
        let _tag = payload_cursor.read_u8()?;

        if x == 0.0 && y == 0.0 && z == 0.0 {
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
        reserved: vec![reserved1],
        crc32,
        timestamp,
        points,
    };

    Ok(laser_data)
}

pub fn read_laserpoint(socket: &std::net::UdpSocket, duration: u32) -> std::io::Result<Vec<LaserPoint>> {
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

#[allow(dead_code)]
/// Test: Asynchronous UDP reader for IMU data
/// Not stable!
pub async fn read_imu_async(socket: &tokio::net::UdpSocket) -> std::io::Result<()> {
    let mut buf = [0; 2048];
    
    loop {
        // 异步接收数据
        let (size, _addr) = socket.recv_from(&mut buf).await?;
        let packet = &buf[..size];
        
        // 直接解析当前数据包
        if let Some(_data) = parse_imu(packet) {
            continue;
            //println!("{:#?}", data);
            //println!("gyro_x: {}, gyro_y: {}, gyro_z: {}", data.gyro_x, data.gyro_y, data.gyro_z);
        } else {
            eprintln!("Failed to parse packet");
        }
    }
}

pub fn read_imu(
    socket: &std::net::UdpSocket,
) -> std::io::Result<ImuData> {
    let mut buf = [0; 2048];
    let mut data_buffer = Vec::new();

    loop {
        match socket.recv_from(&mut buf) {
            Ok((size, _addr)) => {
                data_buffer.extend_from_slice(&buf[..size]);

                match parse_imu(&data_buffer) {
                    Some(imu_data) => {
                        return Ok(imu_data);
                    }
                    None => {
                        eprintln!("Failed to parse packet");
                    }
                }
            }
            Err(e) => {
                eprintln!("Error receiving UDP packet: {}", e);
                continue;
            }
        }
    }
}