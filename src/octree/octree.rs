#![allow(dead_code)]
use std::collections::HashMap;
use crate::prelude::{Point3f, Vector3f, Matrix3f, LaserPoint, distance};


#[derive(Debug)]
#[derive(PartialEq)]
#[derive(Clone)]
pub enum Occupancy {
    Free,
    Occupied,
}

#[derive(Debug, Clone)]
pub enum OctreeNode {
    Internal {
        bounds: [Point3f; 2],
        center: Point3f,
        depth: u32,
        children: [Box<OctreeNode>; 8],
    },
    Leaf {
        bounds: [Point3f; 2],
        center: Point3f,
        depth: u32,
        occupancy: Occupancy,
        reflectivity: [u32; 2],
        laser_points: Vec<LaserPoint>,
        mu: Option<Vector3f>,   // mean
        sigma: Option<Matrix3f>,    // covariance
    }
}

pub struct Octree {
    root: OctreeNode,
}

impl Octree {
    pub fn new(
        bounds: [Point3f; 2],
    ) -> Self {
        Octree {
            root: OctreeNode::Leaf {
                bounds,
                center: Point3f::new(0.0, 0.0, 0.0),
                depth: 0,
                occupancy: Occupancy::Free,
                reflectivity: [0, 0],
                laser_points: Vec::new(),
                mu: None,
                sigma: None,
            }
        }
    }

    /// Expose the root node for visualization
    pub fn get_root_mut(&mut self) -> &mut OctreeNode {
        &mut self.root
    }

    pub fn insert(
        &mut self,
        point: LaserPoint,
        max_depth: u32,
        boundary: f32,
    ) -> Result<(), String> {
        let coordinate = point.coordinate;
        let epsilon = 0.00001;
        let mut flag = true;
        for i in 0..3 {
            if coordinate[i] < -boundary - epsilon || coordinate[i] > boundary + epsilon {
                flag = false;
                break;
            }
        }
        if flag {
            Self::insert_internal(&mut self.root, point, 0, max_depth)
        } else {
            return Ok(());
        }
    }

    fn insert_internal(
        node: &mut OctreeNode,
        point: LaserPoint,
        current_depth: u32,
        max_depth: u32,
    ) -> Result<(), String> {
        // point out-of-bounds check
        let coordinate = point.coordinate;
        let point_reflectivity = point.reflectivity as u32;

        match node {
            OctreeNode::Internal { center, children, .. } => {
                let index = Self::get_index(center, coordinate);
                let child = &mut children[index];
                Self::insert_internal(child, point, current_depth + 1, max_depth)
            }
            #[allow(unused_variables)]
            OctreeNode::Leaf { depth, occupancy, reflectivity, laser_points, .. } => {
                if *occupancy == Occupancy::Occupied {
                    //log::trace!("Point {:?} already occupied at depth {}", point, depth);
                    reflectivity[0] += point_reflectivity as u32;
                    reflectivity[1] += 1;
                    laser_points.push(point);
                    return Ok(());
                }
                else {
                    if current_depth < max_depth {
                        //log::debug!("Splitting leaf node at depth {}", current_depth);
                        Self::split(node, current_depth)?;
                        Self::insert_internal(node, point, current_depth, max_depth)
                    }
                    else {
                        //log::trace!("Marking node occupied at depth {}", current_depth);
                        *occupancy = Occupancy::Occupied;
                        reflectivity[0] += point_reflectivity as u32;
                        reflectivity[1] += 1;
                        laser_points.push(point);
                        Ok(())
                    }
                }

            }
        }
    }

    fn get_index(
        center: &Point3f,
        point_coordinate: Point3f
    ) -> usize {
        let mut index = 0;
        for i in 0..3 {
            if point_coordinate[i] > center[i] {
                index |= 1 << i;
            }
        }
        index
    }

    fn split(
        node: &mut OctreeNode,
        current_depth: u32
    ) -> Result<(), String> {
        let center = match *node {
            OctreeNode::Leaf { center, .. } => center,
            _ => return Err("Cannot split non-leaf node".to_string()),
        };
        let bounds = match *node {
            OctreeNode::Leaf { bounds, .. } => bounds,
            _ => return Err("Cannot split non-leaf node".to_string()),
        };
        let laser_points = match node {
            OctreeNode::Leaf { laser_points, .. } => laser_points.clone(),
            _ => return Err("Cannot split non-leaf node".to_string()),
        };
        let children = match node {
            OctreeNode::Leaf { .. } => Self::create_children(
                &bounds,
                &center,
                current_depth,
                laser_points
            ),
            _ => return Err("Cannot split non-leaf node".to_string()),
        };

        *node = OctreeNode::Internal {
            bounds: bounds,
            center: center,
            depth: current_depth,
            children,
        };

        Ok(())
    }

    fn create_children(
        parent_bounds: &[Point3f; 2],
        parent_center: &Point3f,
        parent_depth: u32,
        parent_laser_points: Vec<LaserPoint>,
    ) -> [Box<OctreeNode>; 8] {
        let new_bounds = [Point3f::new(0.0, 0.0, 0.0), Point3f::new(0.0, 0.0, 0.0)];
        let new_center = Point3f::new(0.0, 0.0, 0.0);
        let mut children: [Box<OctreeNode>; 8] = [
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
            Box::new(OctreeNode::Leaf { bounds: new_bounds, center: new_center, occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0], laser_points: Vec::new(), mu: None, sigma: None }),
        ];

        let mut children_laser_points: Vec<Vec<LaserPoint>> = vec![Vec::new(); 8];
        for point in parent_laser_points {
            let index = Self::get_index(parent_center, point.coordinate);
            children_laser_points[index].push(point);
        }
        
        for i in 0..8 {
            let child_bounds = Self::calculate_child_bounds(&parent_bounds, &parent_center, i);
            children[i] = Box::new(OctreeNode::Leaf {
                bounds: child_bounds,
                center: Point3f::new(
                    (child_bounds[0][0] + child_bounds[1][0]) / 2.0,
                    (child_bounds[0][1] + child_bounds[1][1]) / 2.0,
                    (child_bounds[0][2] + child_bounds[1][2]) / 2.0,
                ),
                depth: parent_depth + 1,
                reflectivity: [0, 0],
                laser_points: children_laser_points[i].clone(),
                occupancy: Occupancy::Free,
                mu: None,
                sigma: None,
            });
        }

        children
    }
    
    fn calculate_child_bounds(
        parent_bounds: &[Point3f; 2],
        parent_center: &Point3f,
        index: usize,
    ) -> [Point3f; 2] {
        let x_sign = if (index & 1) != 0 { 1.0 } else { -1.0 };
        let y_sign = if (index & 2) != 0 { 1.0 } else { -1.0 };
        let z_sign = if (index & 4) != 0 { 1.0 } else { -1.0 };
        let epsilon = 0.00001;

        let child_size = (parent_bounds[1][0] - parent_bounds[0][0]) / 2.0;
        let offset = [
            x_sign * child_size / 2.0,
            y_sign * child_size / 2.0,
            z_sign * child_size / 2.0,
        ];

        let child_min: Point3f = Point3f::new(
            parent_center[0] + offset[0] - child_size / 2.0 - epsilon,
            parent_center[1] + offset[1] - child_size / 2.0 - epsilon,
            parent_center[2] + offset[2] - child_size / 2.0 - epsilon,
        );
        let child_max: Point3f = Point3f::new(
            parent_center[0] + offset[0] + child_size / 2.0 + epsilon,
            parent_center[1] + offset[1] + child_size / 2.0 + epsilon,
            parent_center[2] + offset[2] + child_size / 2.0 + epsilon,
        );
        [child_min, child_max]
    }

    /// Return a map of depth to a list of leaf nodes at that depth
    pub fn octree_to_map(&self) -> HashMap<u32, Vec<OctreeNode>> {
        let mut meshes = HashMap::new();
        Self::octree_to_map_internal(&self.root, &mut meshes);
        meshes
    }

    fn octree_to_map_internal(node: &OctreeNode, meshes: &mut HashMap<u32, Vec<OctreeNode>>) {
        match node {
            OctreeNode::Internal { children, .. } => {
                for child in children.iter() {
                    Self::octree_to_map_internal(child, meshes);
                }
            }
            OctreeNode::Leaf {  depth, occupancy, .. } => {
                if *occupancy == Occupancy::Occupied {
                    // let center = [
                    //     (bounds[0][0] + bounds[1][0]) / 2.0,
                    //     (bounds[0][1] + bounds[1][1]) / 2.0,
                    //     (bounds[0][2] + bounds[1][2]) / 2.0,
                    // ];
                    // let reflectivity = Self::node_reflectivity_calculator(reflectivity);
                    // let data = LaserPoint::new(center[0], center[1], center[2], reflectivity);
                    meshes.entry(*depth).or_insert_with(Vec::new).push(node.clone());
                }
            }
        }
    }

    // fn node_reflectivity_calculator(reflectivity: &[u32; 2]) -> u8 {
    //     let total = reflectivity[1] as f32;
    //     let sum = reflectivity[0] as f32;
    //     (sum / total).round() as u8
    // }

    /// key stands for distance, value stands for laser points
    pub fn get_laser_points(&self) -> HashMap<u32, Vec<(f32, LaserPoint)>> {
        let mut points = HashMap::new();
        Self::get_laser_points_internal(&self.root, &mut points);
        points
    }


    fn get_laser_points_internal(node: &OctreeNode, points: &mut HashMap<u32, Vec<(f32, LaserPoint)>>) {
        match node {
            OctreeNode::Internal { children, .. } => {
                for child in children.iter() {
                    Self::get_laser_points_internal(child, points);
                }
            }
            OctreeNode::Leaf { center, laser_points, .. } => {
                if !laser_points.is_empty() {
                    let distance: f32 = distance(center, &Point3f::new(0.0, 0.0, 0.0));
                    let floored_distance = distance.floor() as u32;
                    points.entry(floored_distance).or_insert_with(Vec::new).extend(laser_points.iter().map(|p| (distance, p.clone())));
                }
            }
        }
    }

    pub fn refresh(&mut self) {
        let old_root = self.root.clone();
        let mut new_root = match old_root {
            OctreeNode::Internal { bounds, center, .. } => {
                OctreeNode::Leaf {
                    bounds,
                    center,
                    depth: 0,
                    occupancy: Occupancy::Free,
                    reflectivity: [0, 0],
                    laser_points: Vec::new(),
                    mu: None,
                    sigma: None,
                }
            }

            OctreeNode::Leaf { bounds, center, .. } => {
                OctreeNode::Leaf {
                    bounds,
                    center,
                    depth: 0,
                    occupancy: Occupancy::Free,
                    reflectivity: [0, 0],
                    laser_points: Vec::new(),
                    mu: None,
                    sigma: None,
                }
            }
        };
        std::mem::swap(&mut self.root, &mut new_root);
        self.root = new_root;
    }

    //TODO: Implement ray casting
    // pub fn cast_ray(&self, origin: [f32; 3], direction: [f32; 3], max_distance: f32) -> Option<f32> {
    //     self.root.cast_ray(origin, direction, max_distance, &mut None)
    // }

    pub fn optimize(&mut self) {
        Self::optimize_recursive_internal(&mut self.root);
        self.compute_statistics();
    }

    fn optimize_recursive_internal(node: &mut OctreeNode) {
        if let OctreeNode::Internal { children, .. } = node {
            for child in children.iter_mut() {
                Self::optimize_recursive_internal(child);
            }
        }
        Self::try_merge_node(node);
    }

    fn try_merge_node(node: &mut OctreeNode) {
        // if let OctreeNode::Internal { bounds, center, depth, children } = node {
        //     let all_free = children.iter().all(|c| Self::is_fully_free(c));
        //     let all_occupied = children.iter().all(|c| Self::is_fully_occupied(c));

        //     if all_free || all_occupied {
        //         let reflectivity = Self::merge_reflectivity(children);
        //         let laser_points = Self::merge_laser_points(children);
        //         *node = OctreeNode::Leaf {
        //             bounds: *bounds,
        //             center: *center,
        //             depth: *depth,
        //             occupancy: if all_free { Occupancy::Free } else { Occupancy::Occupied },
        //             reflectivity,
        //             laser_points,
        //             mu: None,
        //             sigma: None,
        //         };
        //     }
        // }
        // if let OctreeNode::Leaf { laser_points, mu, sigma, .. } = node {
        //     if !laser_points.is_empty() {
        //         let n = laser_points.len() as f32;
        //         let sum = laser_points.iter().fold(Vector3f::zeros(), |acc, p| {
        //             acc + Vector3f::new(p.x, p.y, p.z)
        //         });
        //         let mean = sum / n;
        //         let sum_sq = laser_points.iter().fold(Matrix3f::zeros(), |acc, p| {
        //             let diff = Vector3f::new(p.x, p.y, p.z) - mean;
        //             acc + diff * diff.transpose()
        //         });
        //         let cov = sum_sq / n;
        //         *mu = Some(mean);
        //         *sigma = Some(cov);
        //     }
        // }
        if let OctreeNode::Internal { bounds, center, depth, children } = node {
            let all_free = children.iter().all(|c| Self::is_fully_free(c));
            let all_occupied = children.iter().all(|c| Self::is_fully_occupied(c));
    
            if all_free || all_occupied {
                let reflectivity = Self::merge_reflectivity(children);
                let laser_points = Self::merge_laser_points(children);
                
                // 直接计算 mu 和 sigma
                let (mu, sigma) = if laser_points.is_empty() {
                    (None, None)
                } else {
                    let n = laser_points.len() as f32;
                    let sum = laser_points.iter().fold(Vector3f::zeros(), |acc, p| {
                        acc + Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z)
                    });
                    let mean = sum / n;
                    let sum_sq = laser_points.iter().fold(Matrix3f::zeros(), |acc, p| {
                        let diff = Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z) - mean;
                        acc + diff * diff.transpose()
                    });
                    let cov = sum_sq / n;
                    (Some(mean), Some(cov))
                };
    
                *node = OctreeNode::Leaf {
                    bounds: *bounds,
                    center: *center,
                    depth: *depth,
                    occupancy: if all_free { Occupancy::Free } else { Occupancy::Occupied },
                    reflectivity,
                    laser_points,
                    mu,
                    sigma,
                };
            }
        }
    }

    /// Recursively merges reflectivity values from child nodes
    /// 
    /// # Arguments
    /// * `children` - Slice of boxed OctreeNode children
    /// 
    /// # Returns
    /// An array containing the sum of reflectivity values [sum_reflectivity, sum_point_number]
    fn merge_reflectivity(children: &[Box<OctreeNode>]) -> [u32; 2] {
        children.iter().fold([0, 0], |acc, child| {
            match child.as_ref() {
                OctreeNode::Leaf { reflectivity, .. } => {
                    [acc[0] + reflectivity[0], acc[1] + reflectivity[1]]
                },
                OctreeNode::Internal { children, .. } => {
                    let child_reflectivity = Self::merge_reflectivity(children);
                    [acc[0] + child_reflectivity[0], acc[1] + child_reflectivity[1]]
                }
            }
        })
    }

    fn merge_laser_points(children: &[Box<OctreeNode>]) -> Vec<LaserPoint> {
        children.iter().fold(Vec::new(), |mut acc, child| {
            match child.as_ref() {
                OctreeNode::Leaf { laser_points, .. } => {
                    acc.extend(laser_points.iter().cloned());
                    acc
                },
                OctreeNode::Internal { children, .. } => {
                    let child_laser_points = Self::merge_laser_points(children);
                    acc.extend(child_laser_points.iter().cloned());
                    acc
                }
            }
        })
    }

    fn is_fully_occupied(node: &OctreeNode) -> bool {
        match node {
            OctreeNode::Leaf { occupancy, .. } => *occupancy == Occupancy::Occupied,
            OctreeNode::Internal { children, .. } => children.iter().all(|c| Self::is_fully_occupied(c)),
        }
    }

    fn is_fully_free(node: &OctreeNode) -> bool {
        match node {
            OctreeNode::Leaf { occupancy, .. } => *occupancy == Occupancy::Free,
            OctreeNode::Internal { children, .. } => children.iter().all(|c| Self::is_fully_free(c)),
        }
    }

    pub fn compute_statistics(&mut self) {
        Self::compute_statistics_internal(&mut self.root);
    }

    fn compute_statistics_internal(node: &mut OctreeNode) {
        match node {
            OctreeNode::Leaf { laser_points, mu, sigma, .. } => {
                if laser_points.is_empty() {
                    *mu = None;
                    *sigma = None;
                    return;
                }
                let n = laser_points.len() as f32;
                let sum = laser_points.iter().fold(Vector3f::zeros(), |acc, p| {
                    acc + Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z)
                });
                let mean = sum / n;
                let sum_sq = laser_points.iter().fold(Matrix3f::zeros(), |acc, p| {
                    let diff = Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z) - mean;
                    acc + diff * diff.transpose()
                });
                let cov = sum_sq / n;
                *mu = Some(mean);
                *sigma = Some(cov);
            }
            OctreeNode::Internal { children, .. } => {
                for i in 0..8 {
                    Self::compute_statistics_internal(&mut children[i]);
                }
            }
        }
    }

    pub fn clone(&self) -> Self {
        Octree {
            root: self.root.clone(),
        }
    }
}