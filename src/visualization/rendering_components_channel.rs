#![allow(dead_code)]
use bevy::prelude::*;
use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, DiagnosticsStore};
use crate::calculator::icp::{ICPConfig, ICPOdometry};
use crate::calculator::voxel_grid::voxel_grid_filter;
use crate::calculator::{self, coordinate_switch, crash_detector, imu, point_divider};
use crate::data_reader::udp_reader;
use crate::visualization::color_calculator;
use crate::octree::creat_octree::creat_octree_from_vec;
use crate::calculator::coordinate_switch::mid360_to_bevy;
use crate::calculator::apf;
use crate::data_reader::io;
use crate::octree::octree::Octree;
use crate::prelude::*;
use std::time::Instant;
use std::net::UdpSocket;
use crossbeam_channel::{unbounded, Receiver};

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
pub struct VelocityVector(pub Vec3);

#[derive(Resource)]
pub struct Path(pub Vec<Point3f>);

#[derive(Event)]
struct ImuDataEvent {
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Resource)]
struct ImuReceiver(Receiver<ImuDataEvent>);

#[derive(Resource)]
struct OctreeReceiver(Receiver<Octree>);

pub fn run_bevy() {
    println!("IMU initialization...");
    let imu_bias = imu::imu_init(5);
    let imu_socket = UdpSocket::bind("0.0.0.0:56401").expect("Port bind failed");
    let mut imu_kalman = calculator::kalman_filter::imu_kalman_filter_init(imu_socket, 0.01, 0.01);
    let mut imu_integrator = imu::ImuIntegrator::new(imu_bias);
    let boundary: f32 = io::read_with_default("boundary:", 10.0, None);
    let max_depth: u32 = io::read_with_default("max_depth:", 7, None);
    let voxel_size: f32 = io::read_with_default("voxel_size:", 0.08, None);
    let frame_integration_time: u32 = io::read_with_default("frame_integration_time:", 100, None);

    let (imu_tx, imu_rx) = unbounded();

    std::thread::spawn(move || {
        let imu_socket = UdpSocket::bind("0.0.0.0:56401").expect("Imu Port bind failed");
        loop {
            let dt = Instant::now();
            let imu_data = udp_reader::read_imu(&imu_socket).unwrap();
            imu_integrator.update_with_kalman_filter(imu_data, dt.elapsed().as_secs_f32(), &mut imu_kalman);

            let imu_data_event = ImuDataEvent {
                acc_x: imu_integrator.acc_x,
                acc_y: imu_integrator.acc_y,
                acc_z: imu_integrator.acc_z,
                vx: imu_integrator.vx,
                vy: imu_integrator.vy,
                vz: imu_integrator.vz,
                roll: imu_integrator.roll,
                pitch: imu_integrator.pitch,
                yaw: imu_integrator.yaw,
                x: imu_integrator.x,
                y: imu_integrator.y,
                z: imu_integrator.z,
            };
            imu_tx.send(imu_data_event).unwrap();
        }
    });

    let (lidar_tx, lidar_rx) = unbounded();

    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind("0.0.0.0:56301").expect("Lidar Port bind failed");
        loop {
            let points = udp_reader::read_laserpoint(
                &lidar_socket,
                frame_integration_time
            ).unwrap();

            let voxeled_points = voxel_grid_filter(&points, voxel_size);
            let mut octree = creat_octree_from_vec(boundary, max_depth, voxeled_points);

            octree.optimize();
            lidar_tx.send(octree).unwrap();
        }
    });

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "WorldWithoutAnime".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins( FrameTimeDiagnosticsPlugin)
        .add_event::<ImuDataEvent>()
        .add_event::<Octree>()
        .insert_resource(ImuReceiver(imu_rx))
        .insert_resource(OctreeReceiver(lidar_rx))
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
        .add_systems(Update, imu_event_system)
        .add_systems(Update, octree_event_system)
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
            query: Query<'_, '_, Entity, With<OctreeEntity>>,
            octree_events: EventReader<Octree>|
            octree_update_system(
                commands,
                meshes,
                materials,
                velocity,
                path,
                octree_config,
                apf_config,
                query,
                octree_events,
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
                    Text::new(""),
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
                    Text::new(""),
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
                    Text::new(""),
                    IMUEntityGyro,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: Val::Px(52.0),
                        left: Val::Px(0.0),
                        width: Val::Px(500.0),
                        ..default()
                    },
                ));
                parent.spawn((
                    Text::new(""),
                    IMUEntityAcc,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: Val::Px(0.0),
                        left: Val::Px(0.0),
                        width: Val::Px(500.0),
                        ..default()
                    },
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
    mut octree_events: EventReader<Octree>,
) {
    if !octree_events.is_empty() {
        for entity in query.iter() {
            commands.entity(entity).despawn();
        }
        if let Some(received_octree) = octree_events.read().last() {
            let leaves = received_octree.octree_to_map();
            for (depth, group) in leaves {
                let cuboid_size = get_size(octree_config.boundary, depth);
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
            //let _pose = icp_odometry.process_frame(&points);
    
            // APF palnning
            let start = Point3f::new(0.0, 0.0, 0.0);
            let goal_mid360 = (8.0, 0.0, 0.0);
            let goal = Point3f::new(goal_mid360.0, goal_mid360.1, goal_mid360.2);
    
            let config = ApfConfig {
                k_att: apf_config.k_att,
                k_rep: apf_config.k_rep,
                d0: apf_config.d0,
                epsilon: apf_config.epsilon,
                max_steps: apf_config.max_steps,
                step_size: apf_config.step_size,
            };
    
            let apf_path = apf::apf_plan(start, goal, &received_octree, config);
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
            let tup_obstacle_result = crash_detector::crash_warn_for_octree(&received_octree, warn_trigger_distance);
            let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
            velocity.0 = match mavlink_message.type_mask {
                0b0000001000000000 => {
                    let (x, y, z) = coordinate_switch::frd_to_bevy(mavlink_message.vx, mavlink_message.vy, mavlink_message.vz);
                    Vec3::new(x, y, z)
                }
                _ => Vec3::ZERO,
            };
            
        }
    }
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
    mut imu_events: EventReader<ImuDataEvent>,
) {
    if let Some(last_imu) = imu_events.read().last() {
        let (vx, vy, vz) = coordinate_switch::frd_to_bevy(last_imu.vx, last_imu.vy, last_imu.vz);
        let (roll, pitch, yaw) = coordinate_switch::mid360_to_bevy(last_imu.roll, last_imu.pitch, last_imu.yaw);
        let (x, y, z) = coordinate_switch::mid360_to_bevy(last_imu.x, last_imu.y, last_imu.z);
        let (acc_x, acc_y, acc_z) = coordinate_switch::frd_to_bevy(last_imu.acc_x, last_imu.acc_y, last_imu.acc_z);
        for mut text in param_set.p0().iter_mut() {
            **text = format!("Rotation: Rad\nroll:{:6.2}, pitch:{:6.2}, yaw:{:6.2}",
                roll,
                pitch,
                yaw
            );
        }

        for mut text in param_set.p1().iter_mut() {
            **text = format!("Transition: m\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
                x,
                y,
                z
            );
        }

        for mut text in param_set.p2().iter_mut() {
            **text = format!("Speed: m/s\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
                vx,
                vy,
                vz
            );
        }
        
        for mut text in param_set.p3().iter_mut() {
            **text = format!("Acc: m/s^2\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
                acc_x,
                acc_y,
                acc_z
            );
        }
    }

    // for mut text in param_set.p3().iter_mut() {
    //     **text = format!("Odom_transfer:\nx:{:6.2}, y:{:6.2}, z:{:6.2}",
    //         icp_odometry.global_pose.translation.x,
    //         icp_odometry.global_pose.translation.y,
    //         icp_odometry.global_pose.translation.z
    //     );
    // }
}

fn imu_event_system(mut events: EventWriter<ImuDataEvent>, imu_receiver: Res<ImuReceiver>) {
    while let Ok(data) = imu_receiver.0.try_recv() {
        events.send(data);
    }
}

fn octree_event_system(mut events: EventWriter<Octree>, octree_receiver: Res<OctreeReceiver>) {
    while let Ok(data) = octree_receiver.0.try_recv() {
        events.send(data);
    }
}