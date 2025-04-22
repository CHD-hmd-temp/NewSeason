use crate::octree::octree::Octree;
use crate::prelude::{
    LaserPoint,
    Point3f,
};

#[allow(unused)]
pub fn creat_octree_from_vec(boundary: f32, max_depth: u32, points: Vec<LaserPoint>) -> Octree {
    let mut max_depth = max_depth;

    let mut octree = Octree::new(
        [
            Point3f::new(-boundary, -boundary, -boundary),
            Point3f::new(boundary, boundary, boundary)
        ]
    );
    if max_depth < 1 {
        max_depth = 6;
    }

    for point in points {
        octree.insert(point, max_depth, boundary).unwrap();
    }

    octree
}