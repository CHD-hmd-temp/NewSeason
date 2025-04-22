#![allow(unused)]
use super::point_filter_2d::LaserFrame2D;
use crate::prelude::*;
use std::f32::consts::PI;
use nalgebra::{Point2, Matrix2, Vector2};

#[derive(Debug, Clone, Copy)]
struct Pose2D {
    x: f32,
    y: f32,
    theta: f32, // 弧度制，范围[-π, π]
}

struct Map2D {
    points: Vec<Point2f>,    // 全局地图点云
    pose_graph: Vec<Pose2D>,     // 位姿图
    current_pose: Pose2D,        // 当前估计位姿
    last_matched_points: usize,  // 最近成功匹配点数
}

impl Pose2D {
    fn new(x: f32, y: f32, theta: f32) -> Self {
        Self { x, y, theta: theta.rem_euclid(2.0*PI) }
    }
    
    fn identity() -> Self {
        Self { x: 0.0, y: 0.0, theta: 0.0 }
    }
    
    // 转换为齐次变换矩阵
    fn to_matrix(&self) -> [[f32; 3]; 3] {
        let cos = self.theta.cos();
        let sin = self.theta.sin();
        [
            [cos, -sin, self.x],
            [sin,  cos, self.y],
            [0.0, 0.0, 1.0]
        ]
    }
}

impl Map2D {
    fn new() -> Self {
        Self {
            points: Vec::new(),
            pose_graph: Vec::new(),
            current_pose: Pose2D::identity(),
            last_matched_points: 0,
        }
    }
}

impl Map2D {
    // 主处理流程
    pub fn process_frame(&mut self, frame: &LaserFrame2D) {
        if self.points.is_empty() {
            self.initialize_map(frame);
            return;
        }

        let mut current_pose = self.current_pose;
        let mut best_error = f32::MAX;
        let mut best_pose = current_pose;
        
        // 多分辨率ICP
        for grid_size in [0.2, 0.1, 0.05] { // 逐步提高精度
            let (pose, error) = self.icp_iteration(frame, current_pose, grid_size);
            if error < best_error {
                best_error = error;
                best_pose = pose;
            }
            current_pose = pose;
        }

        self.current_pose = best_pose;
        self.update_map(frame);
    }

    // ICP迭代核心
    fn icp_iteration(&self, 
                    frame: &LaserFrame2D,
                    initial_pose: Pose2D,
                    grid_size: f32) -> (Pose2D, f32) {
        let mut current_pose = initial_pose;
        let mut prev_error = f32::MAX;
        
        for _ in 0..20 { // 最大迭代次数
            let transformed = self.transform_points(frame, current_pose);
            let (matches, error) = self.find_correspondences(&transformed, grid_size);
            
            if matches < 10 || (prev_error - error).abs() < 1e-5 {
                break;
            }
            
            if let Some(delta) = self.calculate_transform(&transformed, grid_size) {
                current_pose = self.apply_delta(current_pose, delta);
                prev_error = error;
            }
        }
        
        (current_pose, prev_error)
    }

    // Find correspondences between transformed points and map
    fn find_correspondences(&self, transformed_points: &[Point2<f32>], grid_size: f32) -> (usize, f32) {
        let mut total_error = 0.0;
        let mut matches = 0;
        
        // Create spatial lookup grid for efficient nearest neighbor search
        let mut grid = std::collections::HashMap::new();
        for (idx, point) in self.points.iter().enumerate() {
            let key = (
                (point.x / grid_size).floor() as i32,
                (point.y / grid_size).floor() as i32
            );
            grid.entry(key).or_insert_with(Vec::new).push((idx, point));
        }
        
        // For each transformed point, find closest map point
        for point in transformed_points {
            let grid_x = (point.x / grid_size).floor() as i32;
            let grid_y = (point.y / grid_size).floor() as i32;
            
            let mut min_dist = f32::MAX;
            let mut found_match = false;
            
            // Check neighboring grid cells
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(candidates) = grid.get(&(grid_x + dx, grid_y + dy)) {
                        for (_, map_point) in candidates {
                            let dist = (point.x - map_point.x).powi(2) + 
                                       (point.y - map_point.y).powi(2);
                            if dist < min_dist {
                                min_dist = dist;
                                found_match = true;
                            }
                        }
                    }
                }
            }
            
            // Add to total error if match found
            if found_match && min_dist < grid_size.powi(2) * 4.0 {
                total_error += min_dist;
                matches += 1;
            }
        }
        
        // Return number of matches and average error
        let avg_error = if matches > 0 { total_error / matches as f32 } else { f32::MAX };
        (matches, avg_error)
    }

    // Apply delta transformation to current pose
    fn apply_delta(&self, pose: Pose2D, delta: (f32, f32, f32)) -> Pose2D {
        let (dx, dy, dtheta) = delta;
        
        // Apply rotation first in the local frame
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();
        
        // Rotate delta in global frame
        let rotated_dx = dx * cos_theta - dy * sin_theta;
        let rotated_dy = dx * sin_theta + dy * cos_theta;
        
        // Apply translation and rotation
        Pose2D::new(
            pose.x + rotated_dx,
            pose.y + rotated_dy,
            pose.theta + dtheta
        )
    }

    // Sample corresponding points for transform calculation
    fn sample_correspondences(&self, 
                             transformed_points: &[Point2<f32>], 
                             grid_size: f32) -> (Vec<Point2<f32>>, Vec<Point2<f32>>) {
        let mut source_points = Vec::new();
        let mut target_points = Vec::new();
        
        // Create spatial lookup grid for efficient nearest neighbor search
        let mut grid = std::collections::HashMap::new();
        for (idx, point) in self.points.iter().enumerate() {
            let key = (
                (point.x / grid_size).floor() as i32,
                (point.y / grid_size).floor() as i32
            );
            grid.entry(key).or_insert_with(Vec::new).push((idx, point));
        }
        
        // For each transformed point, find closest map point
        for (i, point) in transformed_points.iter().enumerate() {
            let grid_x = (point.x / grid_size).floor() as i32;
            let grid_y = (point.y / grid_size).floor() as i32;
            
            let mut min_dist = f32::MAX;
            let mut closest_point = None;
            
            // Check neighboring grid cells
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(candidates) = grid.get(&(grid_x + dx, grid_y + dy)) {
                        for (_, map_point) in candidates {
                            let dist = (point.x - map_point.x).powi(2) + 
                                      (point.y - map_point.y).powi(2);
                            if dist < min_dist && dist < grid_size.powi(2) * 4.0 {
                                min_dist = dist;
                                closest_point = Some(**map_point);
                            }
                        }
                    }
                }
            }
            
            // Add correspondence if found
            if let Some(target) = closest_point {
                source_points.push(*point);
                target_points.push(target);
            }
        }
        
        (source_points, target_points)
    }

    // 坐标变换
    fn transform_points(&self, frame: &LaserFrame2D, pose: Pose2D) -> Vec<Point2<f32>> {
        frame.points.iter().map(|p| {
            let rot = pose.theta;
            Point2::new(
                p.x * rot.cos() - p.y * rot.sin() + pose.x,
                p.x * rot.sin() + p.y * rot.cos() + pose.y
            )
        }).collect()
    }

    // SVD计算变换
    #[allow(non_snake_case)]
    fn calculate_transform(&self,
                         points: &[Point2<f32>],
                         grid_size: f32) -> Option<(f32, f32, f32)> {
        let (src, tgt) = self.sample_correspondences(points, grid_size);
        if src.is_empty() || src.len() != tgt.len() {
            return None;
        }

        // 计算中心
        let src_center = src.iter()
            .fold(Vector2::zeros(), |acc, p| acc + p.coords)
            / src.len() as f32;
        
        let tgt_center = tgt.iter()
            .fold(Vector2::zeros(), |acc, p| acc + p.coords)
            / tgt.len() as f32;

        // 计算协方差矩阵
        let mut H = Matrix2::zeros();
        for (s, t) in src.iter().zip(tgt) {
            H += (s.coords - src_center) * (t.coords - tgt_center).transpose();
        }

        // SVD分解
        let svd = H.svd(true, true);
        let U = svd.u.unwrap();
        let V = svd.v_t.unwrap().transpose();

        // 计算旋转
        let mut R = V * U.transpose();
        if R.determinant() < 0.0 {
            let mut col = R.column_mut(1);
            for i in 0..col.len() {
                col[i] = -col[i];
            }
        }

        // 计算平移
        let t = tgt_center - R * src_center;

        // 转换为位姿变化量
        let delta_theta = R[(1, 0)].atan2(R[(0, 0)]);
        Some((t.x, t.y, delta_theta))
    }
}

impl Map2D {
    // 初始化地图（首帧处理）
    fn initialize_map(&mut self, frame: &LaserFrame2D) {
        let points = frame.points.iter()
            .map(|p| Point2::new(p.x, p.y))
            .collect();
        
        self.points = self.downsample(points, 0.05);
        self.pose_graph.push(Pose2D::identity());
    }

    // 更新全局地图
    fn update_map(&mut self, frame: &LaserFrame2D) {
        let transformed = self.transform_points(frame, self.current_pose);
        let mut new_points = self.downsample(transformed, 0.05);
        
        // 空间哈希去重
        let mut hash_map = std::collections::HashMap::new();
        let grid_size = 0.05;
        
        for p in &self.points {
            let key = ((p.x / grid_size).round() as i32, 
                      (p.y / grid_size).round() as i32);
            hash_map.insert(key, *p);
        }
        
        for p in new_points.drain(..) {
            let key = ((p.x / grid_size).round() as i32,
                      (p.y / grid_size).round() as i32);
            if !hash_map.contains_key(&key) {
                self.points.push(p);
                hash_map.insert(key, p);
            }
        }
        
        self.pose_graph.push(self.current_pose);
    }

    // 降采样
    fn downsample(&self, points: Vec<Point2<f32>>, resolution: f32) -> Vec<Point2<f32>> {
        let mut grid = std::collections::HashMap::new();
        for p in points {
            let key = (
                (p.x / resolution).round() as i32,
                (p.y / resolution).round() as i32
            );
            grid.entry(key).or_insert(p);
        }
        grid.values().cloned().collect()
    }
}