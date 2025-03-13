use crate::data_reader;
use crate::octree::octree::*;
use crate::calculator::voxel_grid;
use std::net::UdpSocket;

pub fn creat_octree_from_udp(boundary: f32, max_depth: u32, voxel_size: f32, frame_integration_time: u32) -> Octree {
    let mut max_depth = max_depth;
    let socket_laserpoint = UdpSocket::bind("0.0.0.0:56301").expect("Port bind failed");
    let mut points = data_reader::udp_reader::read_laserpoint(
        &socket_laserpoint,
        frame_integration_time
    ).unwrap();
    if voxel_size >= 0.05 {
        points = voxel_grid::voxel_grid_filter(&points, voxel_size);
    }

    let mut octree = Octree::new([[-boundary; 3], [boundary; 3]]);
    if max_depth < 1 {
        max_depth = 6;
    }

    for point in points {
        octree.insert(point, max_depth).unwrap();
    }

    octree
}