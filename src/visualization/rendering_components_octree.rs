#![allow(dead_code)]
use bevy::prelude::*;
use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, DiagnosticsStore};
use crate::calculator::icp::{ICPConfig, ICPOdometry};
use crate::calculator::voxel_grid::voxel_grid_filter;
use crate::calculator::{coordinate_switch, crash_detector, imu, point_divider};
use crate::data_reader::udp_reader;
use crate::visualization::color_calculator;
use crate::octree::creat_octree::creat_octree_from_vec;
use crate::calculator::coordinate_switch::mid360_to_bevy;
use crate::calculator::apf;
use crate::data_reader::io;
use crate::prelude::*;
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

#[derive(Component)]
struct ICPEntityTransfer;

#[derive(Component)]
struct ICPEntityRotation;

#[derive(Resource)]
struct FrameIntegrationTime(pub u64);

#[derive(Resource)]
pub struct VelocityVector(pub Vec3);

#[derive(Resource)]
pub struct Path(pub Vec<Point3f>);

#[derive(Resource)]
pub struct OctreeConfig {
    boundary: f32,
    max_depth: u32,
    voxel_size: f32,
    frame_integration_time: u32,
}

pub fn run_bevy() {
    println!("IMU initialization...");
    let imu_bias = imu::imu_init();
    let boundary: f32 = io::read_with_default("boundary:", 10.0, None);
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
        .insert_resource(ICPOdometry::new(ICPConfig {
            num_samples: 600,
            max_iterations: 20,
            tolerance: 1e-5,
            max_correspondence_dist: 1.0,
        }))
        .insert_resource(imu_bias)
        .insert_resource(VelocityVector(Vec3::ZERO))
        .insert_resource(Path(Vec::new()))
        .add_systems(Startup,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            | {
            setup_bevy(
                commands,
                meshes,
                materials,
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
            icp_odometry: ResMut<ICPOdometry>,
            octree_config: Res<OctreeConfig>,
            apf_config: Res<ApfConfig>,
            query: Query<'_, '_, Entity, With<OctreeEntity>>|
            octree_update_system(
                commands,
                meshes,
                materials,
                velocity,
                path,
                icp_odometry,
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
) {
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
            Text::new("v1.4-Soyo"),
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
            Text::new("Lio: "),
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
                    Text::new("Odom_rotation:"),
                    ICPEntityRotation,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: Val::Px(156.0),
                        left: Val::Px(0.0),
                        width: Val::Px(500.0),
                        ..default()
                    },
                ));
                parent.spawn((
                    Text::new("Odom_transfer:"),
                    ICPEntityTransfer,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: Val::Px(104.0),
                        left: Val::Px(0.0),
                        width: Val::Px(500.0),
                        ..default()
                    },
                ));
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
    mut icp_odometry: ResMut<ICPOdometry>,
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
    let socket_laserpoint = UdpSocket::bind("0.0.0.0:56301").expect("Port bind failed");
    let points = udp_reader::read_laserpoint(
        &socket_laserpoint,
        frame_integration_time
    ).unwrap();

    let voxeled_points = voxel_grid_filter(&points, voxel_size);
    let mut octree = creat_octree_from_vec(boundary, max_depth, voxel_size, voxeled_points);

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
                let point = point.coordinate;
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

    // ICP
    let pose = icp_odometry.process_frame(&points);

    // APF palnning
    let start = Point3f::new(0.0, 0.0, 0.0);
    let goal_mid360 = (5.0, 0.0, 0.0);
    let goal = Point3f::new(goal_mid360.0, goal_mid360.1, goal_mid360.2);

    let config = ApfConfig {
        k_att: apf_config.k_att,
        k_rep: apf_config.k_rep,
        d0: apf_config.d0,
        epsilon: apf_config.epsilon,
        max_steps: apf_config.max_steps,
        step_size: apf_config.step_size,
    };

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

    let warn_trigger_distance = apf_config.d0;
    let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
    let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
    velocity.0 = match mavlink_message.type_mask {
        0b0000001000000000 => {
            let (x, y, z) = coordinate_switch::frd_to_bevy(mavlink_message.vx, mavlink_message.vy, mavlink_message.vz);
            Vec3::new(x, y, z)
        }
        _ => Vec3::ZERO,
    };
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
        Query<&mut Text, With<ICPEntityRotation>>,
        Query<&mut Text, With<ICPEntityTransfer>>,
    )>,
    imu_bias: Res<ImuBias>,
    icp_odometry: ResMut<ICPOdometry>
) {
    let socket_imu = UdpSocket::bind("0.0.0.0:56401").expect("Port bind failed");
    let imu_data = udp_reader::read_imu(&socket_imu).unwrap();

    for mut text in param_set.p0().iter_mut() {
        **text = format!("Gyro: Rad/s\nx:{:6.2}, y:{:6.2}, Z:{:6.2}",
            imu_data.gyro_x - imu_bias.gyro_x,
            imu_data.gyro_y - imu_bias.gyro_y,
            imu_data.gyro_z - imu_bias.gyro_z
        );
    }

    for mut text in param_set.p1().iter_mut() {
        **text = format!("Acc: m/s^2\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
            (imu_data.acc_x - imu_bias.acc_x) * 9.8,
            (imu_data.acc_y - imu_bias.acc_y) * 9.8,
            (imu_data.acc_z - imu_bias.acc_z) * 9.8
        );
    }

    for mut text in param_set.p2().iter_mut() {
        **text = format!("Odom_rotation:\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
            icp_odometry.global_pose.rotation.euler_angles().0,
            icp_odometry.global_pose.rotation.euler_angles().1,
            icp_odometry.global_pose.rotation.euler_angles().2
        );
    }

    for mut text in param_set.p3().iter_mut() {
        **text = format!("Odom_transfer:\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
            icp_odometry.global_pose.translation.x,
            icp_odometry.global_pose.translation.y,
            icp_odometry.global_pose.translation.z
        );
    }
}