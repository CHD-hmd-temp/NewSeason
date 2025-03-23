#![allow(dead_code)]
use crate::prelude::*;
use bevy::ecs::system::Resource;
use nalgebra::{Isometry3, Rotation3};
use rand::seq::index::sample;
#[allow(unused_imports)]
use rand::Rng;
use kdtree::KdTree;

/// ICP 激光雷达里程计结构体
#[derive(Debug, Resource)]
pub struct ICPOdometry {
    pub prev_points: Option<Vec<LaserPoint>>, // 上一帧点云
    pub global_pose: Isometry3<f32>,     // 全局位姿
    pub config: ICPConfig,               // 配置参数
}

/// ICP 配置参数
#[derive(Debug, Clone, Copy, Resource)]
pub struct ICPConfig {
    pub num_samples: usize,          // 随机采样点数 (建议 500~1000)
    pub max_iterations: u32,         // ICP 最大迭代次数 (建议 20)
    pub tolerance: f32,              // 收敛阈值 (建议 1e-5)
    pub max_correspondence_dist: f32,// 最大对应点距离 (建议 1.0~2.0)
}

impl ICPOdometry {
    pub fn new(config: ICPConfig) -> Self {
        Self {
            prev_points: None,
            global_pose: Isometry3::identity(),
            config,
        }
    }

    pub fn process_frame(&mut self, current_frame: &Vec<LaserPoint>) -> Isometry3<f32> {
        let Some(prev_points) = &self.prev_points else {
            self.prev_points = Some(current_frame.clone());
            return self.global_pose;
        };

        // 1. 随机采样
        let sampled_current = random_sample(current_frame, self.config.num_samples);
        let sampled_prev = random_sample(prev_points, self.config.num_samples);

        // 2. ICP 迭代
        let (delta_rotation, delta_translation) = icp_step(
            &sampled_current,
            &sampled_prev,
            self.config
        );

        // 如果 ICP 无法收敛，返回上一帧位姿
        if delta_rotation == Rotation3::identity() && delta_translation == Vector3f::zeros() {
            return self.global_pose;
        }

        // 检查旋转矩阵是否有效
        // if (delta_rotation - Rotation3::identity()).norm() > 1e-3 {
        //     return self.global_pose;
        // }

        // 3. 更新全局位姿
        let delta_iso = Isometry3::from_parts(
            delta_translation.into(),
            delta_rotation.into(),
        );
        self.global_pose = self.global_pose * delta_iso;

        // 更新上一帧点云
        self.prev_points = Some(current_frame.clone());

        self.global_pose
    }
}

fn random_sample(points: &Vec<LaserPoint>, num_samples: usize) -> Vec<LaserPoint> {
    let mut rng = rand::rng();
    let n = points.len();
    if n <= num_samples {
        return points.clone();
    }
    let num_samples = num_samples.min(n);
    let indices = sample(&mut rng, num_samples, num_samples);
    indices.into_iter()
        .map(|i| points[i].clone())
        .collect()
}

fn icp_step(
    source: &Vec<LaserPoint>,
    target: &Vec<LaserPoint>,
    config: ICPConfig,
) -> (Rotation3<f32>, Vector3f) {
    let mut target_kdtree: KdTree<f32, usize, [f32; 3]> = KdTree::new(3);
    for (i, p) in target.iter().enumerate() {
        let coordinate = p.coordinate;
        target_kdtree.add([coordinate.x, coordinate.y, coordinate.z], i).unwrap();
    }
    // for (i, p) in target.iter().enumerate() {
    //     target_kdtree.add([p.coordinate.x, p.coordinate.y, p.coordinate.z], i).unwrap();
    // }
    // target.retain(|p| {
    //     let squared_euclidean = kdtree::distance::squared_euclidean;
    //     let neighbors = target_kdtree.nearest(&[p.coordinate.x, p.coordinate.y, p.coordinate.z], 10, ).unwrap();
    //     let dist_mean = neighbors.iter().map(|(d,_)| d.sqrt()).mean();
    //     dist_mean < 0.1 // 保留静态点
    // });

    let mut source_transformed = source.clone();
    let mut r = Matrix3f::identity();
    let mut t = Vector3f::zeros();

    let mut has_valid_step = false; // 标记是否有至少一次有效迭代

    for _ in 0..config.max_iterations {
        let mut correspondences = Vec::new();
        for s in &source_transformed {
            let nearest = target_kdtree.nearest(&[s.coordinate.x, s.coordinate.y, s.coordinate.z], 1, &kdtree::distance::squared_euclidean)
                .unwrap();
            let (dist_sq, idx) = nearest[0];
            if dist_sq.sqrt() <= config.max_correspondence_dist {
                correspondences.push(target[*idx]);
            }
        }

        if correspondences.is_empty() {
            return (Rotation3::identity(), Vector3f::zeros());
        }

        // 计算质心
        let centroid_s = source_transformed.iter()
            .map(|p| Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z))
            .sum::<Vector3f>() / source_transformed.len() as f32;
        let centroid_t = correspondences.iter()
            .map(|p| Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z))
            .sum::<Vector3f>() / correspondences.len() as f32;

        // 中心化
        let mut src_centered = Vec::new();
        let mut tgt_centered = Vec::new();
        for (s, t) in source_transformed.iter().zip(correspondences.iter()) {
            src_centered.push(Vector3f::new(s.coordinate.x, s.coordinate.y, s.coordinate.z) - centroid_s);
            tgt_centered.push(Vector3f::new(t.coordinate.x, t.coordinate.y, t.coordinate.z) - centroid_t);
        }

        // 计算旋转矩阵
        let h_matrix: Matrix3f = src_centered.iter()
            .zip(tgt_centered.iter())
            .map(|(s, t)| s * t.transpose())
            .sum();
        let svd = h_matrix.svd(true, true);
        if svd.u.is_none() || svd.v_t.is_none() {
            continue;
        }
        let u = svd.u.unwrap();
        let v_t = svd.v_t.unwrap();
        let r_step = v_t.transpose() * u.transpose();

        // 检查旋转矩阵是否有效（行列式接近1）
        if (r_step.determinant() - 1.0).abs() > 1e-3 {
            continue; // 无效旋转，跳过
        }

        // 计算平移矩阵
        let t_step = centroid_t - r_step * centroid_s;

        // 更新变换
        r = r_step * r;
        t = r_step * t + t_step;

        // 更新点云
        source_transformed = source.iter()
            .map(|p| {
                let vec = r_step * Vector3f::new(p.coordinate.x, p.coordinate.y, p.coordinate.z) + t_step;
                let new_coordinate = Point3f::new(vec.x, vec.y, vec.z);
                let new_reflectivity = p.reflectivity;
                LaserPoint::new(new_coordinate, new_reflectivity)
            })
            .collect();

        // 标记有效迭代
        has_valid_step = true;

        // 判断收敛
        if (r_step - Matrix3f::identity()).norm() < config.tolerance {
            break;
        }
    }

    if !has_valid_step {
        (Rotation3::identity(), Vector3f::zeros())
    } else {
        (Rotation3::from_matrix(&r), t)
    }

}