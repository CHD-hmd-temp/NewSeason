use crate::prelude::*;
use crate::octree::octree::*;
use std::collections::HashMap;


#[allow(dead_code)]
pub fn divide_points(
    points: Vec<LaserPoint>,
) -> HashMap<u8, Vec<LaserPoint>> {
    let mut grouped_points: HashMap<u8, Vec<LaserPoint>> = HashMap::new();

    for point in points {
        grouped_points
            .entry(point.reflectivity) // Get the group corresponding to the Reflectivity
            .or_insert_with(Vec::new) // Initialize a new vector if it doesn't exist
            .push(point);
    }

    grouped_points
}

pub fn divide_nodes(
    nodes: Vec<OctreeNode>,
) -> HashMap<u8, Vec<NodeForRender>> {
    let mut grouped_nodes: HashMap<u8, Vec<NodeForRender>> = HashMap::new();

    for node in nodes {
        match node {
            OctreeNode::Leaf { center, reflectivity, .. } => {
                let coordinate = Point3f::new(center[0], center[1], center[2]);
                let node = NodeForRender {
                    coordinate,
                    reflectivity: node_reflectivity_calculator(&reflectivity),
                };
                grouped_nodes
                    .entry(node.reflectivity) // Get the group corresponding to the Reflectivity
                    .or_insert_with(Vec::new) // Initialize a new vector if it doesn't exist
                    .push(node);
            },
            OctreeNode::Internal { .. } => {}
        }
    }

    grouped_nodes
}

fn node_reflectivity_calculator(reflectivity: &[u32; 2]) -> u8 {
    let total = reflectivity[1] as f32;
    let sum = reflectivity[0] as f32;
    (sum / total).round() as u8
}