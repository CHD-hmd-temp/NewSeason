#![allow(dead_code)]
use bevy::prelude::*;
use bevy_flycam::prelude::*;
use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, DiagnosticsStore};
use crate::calculator::point_divider;
use crate::data_reader::structor::Point3;
use crate::data_reader::udp_reader::{self, ImuData};
use crate::visualization::color_calculator;
use crate::octree::creat_octree;
use crate::calculator::coordinate_switch::mid360_to_bevy;
use crate::calculator::apf;
use crate::calculator::apf::ApfConfig;
use crate::data_reader::io;
use std::net::UdpSocket;

#[derive(Component)]
struct Ground;

#[derive(Component)]
struct FpsText;

#[derive(Component)]
struct OctreeEntity;

#[derive(Component)]
struct IMUEntityAcc;

#[derive(Component)]
struct IMUEntityGyro;

#[derive(Resource)]
struct FrameIntegrationTime(pub u64);

#[derive(Resource)]
pub struct VelocityVector(pub Vec3);

#[derive(Resource)]
pub struct Path(pub Vec<Point3>);

#[derive(Resource)]
pub struct OctreeConfig {
    boundary: f32,
    max_depth: u32,
    voxel_size: f32,
    frame_integration_time: u32,
}

pub fn run_bevy() {
    let boundary: f32 = io::read_with_default("boundary:", 10.0, None);
    //let boundary: f32 = 10.0;
    let max_depth: u32 = io::read_with_default("max_depth:", 7, None);
    let voxel_size: f32 = io::read_with_default("voxel_size:", 0.08, None);
    let frame_integration_time: u32 = io::read_with_default("frame_integration_time:", 100, None);
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "WorldWithoutAnime".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins( FrameTimeDiagnosticsPlugin)
        .add_plugins(NoCameraPlayerPlugin)
        .insert_resource(MovementSettings {
            sensitivity: 0.00009,
            speed: 3.0,
        })
        .insert_resource(OctreeConfig {
            boundary,
            max_depth,
            voxel_size,
            frame_integration_time,
        })
        .insert_resource(ApfConfig {
            k_att: 2.5,
            k_rep: 2.5,
            d0: 0.7,
            epsilon: 0.1,
            max_steps: 500,
            step_size: 0.1,
        })
        .insert_resource(ImuData {
            version: 0,
            length: 0,
            time_interval: 0,
            dot_num: 0,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: 0,
            time_type: 0,
            reserved: Vec::new(),
            crc32: 0,
            timestamp: 0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            acc_x: 0.0,
            acc_y: 0.0,
            acc_z: 0.0,
        })
        .insert_resource(VelocityVector(Vec3::ZERO))
        .insert_resource(Path(Vec::new()))
        .add_systems(Startup,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            octree_config: Res<OctreeConfig>
            | {
            setup_bevy(
                commands,
                meshes,
                materials,
                octree_config,
            );
        })
        .add_systems(Update, text_update_system)
        .add_systems(Update, update_imu)
        .add_systems(Update,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            velocity: ResMut<VelocityVector>,
            path: ResMut<Path>,
            octree_config: Res<OctreeConfig>,
            apf_config: Res<ApfConfig>,
            query: Query<'_, '_, Entity, With<OctreeEntity>>|
            octree_update_system(
                commands,
                meshes,
                materials,
                velocity,
                path,
                octree_config,
                apf_config,
                query
            ))
        .add_systems(Update, draw_gizmos)
        .run();
}

fn setup_bevy(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    octree_config: Res<OctreeConfig>,
) {
    let boundary = octree_config.boundary;
    let max_depth = octree_config.max_depth;
    let voxel_size = octree_config.voxel_size;
    let frame_integration_time = octree_config.frame_integration_time;
    let mut octree = creat_octree::creat_octree_from_udp(boundary, max_depth, voxel_size, frame_integration_time);
    octree.optimize();
    let leaves = octree.get_laser_points();

    for (depth, group) in leaves {
        let grouped_pixel_points = point_divider::divide_points(group);
        let cube_mesh = meshes.add(Mesh::from(
            Cuboid::new(
                get_size(boundary, depth),
                get_size(boundary, depth),
                get_size(boundary, depth)
            )
        ));

        for (reflectivity, group) in &grouped_pixel_points {
            let material = materials.add(StandardMaterial {
                emissive: color_calculator::reflectivity_to_color(*reflectivity).into(),
                ..default()
            });

            for point in group {
                let (x, y, z) = mid360_to_bevy(point.x, point.y, point.z);
                commands.spawn((
                    //LaserPoint,
                    Mesh3d(cube_mesh.clone()), // Reuse the same mesh
                    MeshMaterial3d(material.clone()), // Reuse the same material
                    Transform::from_translation(Vec3::new(x, y, z)),
                    OctreeEntity,
                ));
            }
        }
    };

    // Add a camera at [0, 0, 2] and look at front
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0., 1.5, 4.).looking_at(Vec3::ZERO, Vec3::Y),
        //FlyCam,
    ));

    // Add a red sphere to represent the drone at [0, 0, 0]
    let sphere_mesh = meshes.add(Sphere::new(0.1));
    let material = materials.add(StandardMaterial {
        emissive: Color::srgb_u8(255, 0, 0).into(),
        ..default()
    });
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(material),
        Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
    )); 

    // Text with multiple sections
    commands
        .spawn((
            Text::new("FPS: "),
            Node {
                position_type: PositionType::Absolute,
                bottom:Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextColor(GOLD.into()),
            FpsText,
        ));
    
    // Text on the top left corner
    commands
        .spawn((
            Text::new("v1.3-Saki"),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
        ));

    // text shows IMU data
    commands
        .spawn((
            Text::new("IMU: "),
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(48.0),
                left: Val::Px(12.0),
                ..default()
            },
        ))
        .with_children(
            |parent| {
                parent.spawn((
                    Text::new("Gyro: "),
                    IMUEntityGyro,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: Val::Px(52.0),
                        left: Val::Px(0.0),
                        width: Val::Px(500.0),
                        ..default()
                    },
                    //TextColor(GOLD.into()),
                ));
                parent.spawn((
                    Text::new("Acc: "),
                    IMUEntityAcc,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: Val::Px(0.0),
                        left: Val::Px(0.0),
                        width: Val::Px(500.0),
                        ..default()
                    },
                    //TextColor(GOLD.into()),
                ));
            }
        );
}

fn text_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<&mut TextSpan, With<FpsText>>,
) {
    for mut span in &mut query {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                // Update the value of the second section
                **span = format!("{value:.2}");
            }
        }
    }
}

fn octree_update_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut velocity: ResMut<VelocityVector>,
    mut path: ResMut<Path>,
    octree_config: Res<OctreeConfig>,
    apf_config: Res<ApfConfig>,
    query: Query<Entity, With<OctreeEntity>>,
) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }
    let boundary = octree_config.boundary;
    let max_depth = octree_config.max_depth;
    let voxel_size = octree_config.voxel_size;
    let frame_integration_time = octree_config.frame_integration_time;
    let mut octree = creat_octree::creat_octree_from_udp(boundary, max_depth, voxel_size, frame_integration_time);

    octree.optimize();

    let leaves = octree.octree_to_map();
    for (depth, group) in leaves {
        let cuboid_size = get_size(boundary, depth);
        let grouped_pixel_points = point_divider::divide_nodes(group);
        let cube_mesh = meshes.add(Mesh::from(
            Cuboid::new(
                cuboid_size,
                cuboid_size,
                cuboid_size
            )
        ));

        for (reflectivity, group) in &grouped_pixel_points {
            let material = materials.add(StandardMaterial {
                emissive: color_calculator::reflectivity_to_color(*reflectivity).into(),
                ..default()
            });

            for point in group {
                let (x, y, z) = mid360_to_bevy(point.x, point.y, point.z);
                commands.spawn((
                    Mesh3d(cube_mesh.clone()), // Reuse the same mesh
                    MeshMaterial3d(material.clone()), // Reuse the same material
                    Transform::from_translation(Vec3::new(x, y, z)),
                    OctreeEntity,
                ));
            }
        }
    };

    // APF palnning
    let start = Point3::new(0.0, 0.0, 0.0);
    let goal_mid360 = (5.0, 0.0, 0.0);
    let goal = Point3::new(goal_mid360.0, goal_mid360.1, goal_mid360.2);

    let config = ApfConfig {
        k_att: apf_config.k_att,
        k_rep: apf_config.k_rep,
        d0: apf_config.d0,
        epsilon: apf_config.epsilon,
        max_steps: apf_config.max_steps,
        step_size: apf_config.step_size,
    };

    let _warn_trigger_distance = apf_config.d0;

    let apf_path = apf::apf_plan(start, goal, &octree, config);
    let vec = match apf_path {
        Ok(path) => {
            path
        }
        Err(e) => {
            println!("Error: {:?}", e);
            Vec::new()
        }
    };
    path.0 = vec;

    velocity.0 = Vec3::ZERO;
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    velocity: Res<VelocityVector>,
    path: Res<Path>,
) {
    use std::f32::consts::PI;
    gizmos.line(
        Vec3::ZERO,
        velocity.0,
        Color::srgb_u8(255, 0, 0),
    );
    gizmos.grid(
        Quat::from_rotation_x(PI / 2.),
        UVec2::splat(20),
        Vec2::new(2., 2.),
        // Light gray
        LinearRgba::gray(0.35),
    );
    if path.0.len() > 1 {
        for i in 0..path.0.len() - 1 {
            let (x, y, z) = mid360_to_bevy(path.0[i].x, path.0[i].y, path.0[i].z);
            let (x1, y1, z1) = mid360_to_bevy(path.0[i + 1].x, path.0[i + 1].y, path.0[i + 1].z);
            gizmos.line(
                Vec3::new(x, y, z),
                Vec3::new(x1, y1, z1),
                Color::srgb_u8(0, 255, 0),
            );
        }
    }
}

fn get_size(boundary: f32, max_depth: u32) -> f32 {
    let mut size = boundary * 2.0;
    for _ in 0..max_depth {
        size /= 2.0;
    }
    size
}

fn update_imu(
    mut param_set: ParamSet<(
        Query<&mut Text, With<IMUEntityGyro>>,
        Query<&mut Text, With<IMUEntityAcc>>,
    )>,
) {
    let socket_imu = UdpSocket::bind("0.0.0.0:56401").expect("Port bind failed");
    let imu_data = udp_reader::read_imu(&socket_imu).unwrap();

    for mut text in param_set.p0().iter_mut() {
        **text = format!("Gyro: Rad/s\nx:{:6.2}, y:{:6.2}, Z:{:6.2}", imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
    }

    for mut text in param_set.p1().iter_mut() {
        **text = format!("Acc: m/s^2\nx:{:6.2}, y:{:6.2}, z:{:6.2}", imu_data.acc_x * 9.8, imu_data.acc_y * 9.8, imu_data.acc_z * 9.8);
    }
}