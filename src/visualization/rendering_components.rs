#![allow(dead_code)]
use bevy::prelude::*;
use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, DiagnosticsStore};
use crate::calculator::voxel_grid::voxel_grid_filter;
use crate::calculator::{self, coordinate_switch, crash_detector, imu, point_divider};
use crate::data_reader::udp_reader;
use crate::visualization::color_calculator;
use crate::octree::creat_octree::creat_octree_from_vec;
use crate::calculator::coordinate_switch::mid360_to_bevy;
use crate::calculator::apf;
use crate::calculator::imu::ImuIntegrator;
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

#[derive(Event)]
struct Msgs {
    apf_path: Vec<Point3f>,
    velocity: Vec3,
}

#[derive(Resource)]
struct LatestMsgs(Msgs);

#[derive(Resource)]
struct OctreeConfig {
    boundary: f32,
    max_depth: u32,
    voxel_size: f32,
}

#[derive(Resource)]
struct ImuReceiver(Receiver<ImuIntegrator>);

#[derive(Resource)]
struct OctreeReceiver(Receiver<Octree>);

#[derive(Resource)]
struct MsgsReceiver(Receiver<Msgs>);

pub fn run_bevy() {
    println!("IMU initialization...");
    let imu_bias = imu::imu_init(5);
    let imu_socket = UdpSocket::bind("0.0.0.0:56401").expect("Port bind failed");
    let mut imu_kalman = calculator::kalman_filter::imu_kalman_filter_init(imu_socket, 0.01, 0.01);
    let mut imu_integrator = imu::ImuIntegrator::new(imu_bias);
    let boundary: f32 = io::read_with_default(
        "boundary:",
        10.0,
        None
    );
    let max_depth: u32 = io::read_with_default(
        "max_depth:",
        7,
        None
    );
    let voxel_size: f32 = io::read_with_default(
        "voxel_size:",
        0.08,
        None
    );
    let frame_integration_time: u32 = io::read_with_default(
        "frame_integration_time:",
        100,
        None
    );

    let (imu_tx, imu_rx) = unbounded();
    std::thread::spawn(move || {
        let imu_socket = UdpSocket::bind("0.0.0.0:56401").expect("Imu Port bind failed");
        loop {
            let dt = Instant::now();
            let imu_data = udp_reader::read_imu(&imu_socket).unwrap();
            imu_integrator.update_with_kalman_filter(imu_data, dt.elapsed().as_secs_f32(), &mut imu_kalman);
            imu_tx.send(imu_integrator).unwrap();
        }
    });

    let (lidar_tx, lidar_rx) = unbounded();
    let (msg_tx, msg_rx) = unbounded();
    std::thread::spawn(move || {
        let lidar_socket = UdpSocket::bind("0.0.0.0:56301").expect("Lidar Port bind failed");
        let apf_goal = Point3f::new(5.0, 0.0, 0.0);
        let apf_config = ApfConfig {
            k_att: 2.5,
            k_rep: 2.5,
            d0: 0.7,
            epsilon: 0.1,
            max_steps: 500,
            step_size: 0.1,
        };
        loop {
            let points = udp_reader::read_laserpoint(
                &lidar_socket,
                frame_integration_time
            ).unwrap();

            let voxeled_points = voxel_grid_filter(&points, voxel_size);
            let mut octree = creat_octree_from_vec(boundary, max_depth, voxeled_points);

            octree.optimize();

            let apf_path = apf::apf_plan(
                Point3f::new(0.0, 0.0, 0.0),
                apf_goal,
                &octree,
                &apf_config,
            );

            let vec = match apf_path {
                Ok(path) => {
                    path
                }
                Err(e) => {
                    println!("Error: {:?}", e);
                    Vec::new()
                }
            };

            let warn_trigger_distance = apf_config.d0;
            let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
            let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
            let velocity = match mavlink_message.type_mask {
                0b0000001000000000 => {
                    let (x, y, z) = coordinate_switch::frd_to_bevy(mavlink_message.vx, mavlink_message.vy, mavlink_message.vz);
                    Vec3::new(x, y, z)
                }
                _ => Vec3::ZERO,
            };

            let msg = Msgs {
                apf_path: vec,
                velocity,
            };

            msg_tx.send(msg).unwrap();
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
        .add_event::<ImuIntegrator>()
        .add_event::<Octree>()
        .add_event::<Msgs>()
        .insert_resource(ImuReceiver(imu_rx))
        .insert_resource(MsgsReceiver(msg_rx))
        .insert_resource(OctreeReceiver(lidar_rx))
        .insert_resource(LatestMsgs(Msgs {
            apf_path: Vec::new(),
            velocity: Vec3::ZERO,
        }))
        .insert_resource(OctreeConfig {
            boundary,
            max_depth,
            voxel_size,
        })
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
        .add_systems(Update, fps_update_system)
        .add_systems(Update, msgs_event_system)
        .add_systems(Update, update_imu)
        .add_systems(Update,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            octree_config: Res<OctreeConfig>,
            query: Query<'_, '_, Entity, With<OctreeEntity>>,
            octree_events: EventReader<Octree>|
            octree_update_system(
                commands,
                meshes,
                materials,
                octree_config,
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

fn fps_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<&mut TextSpan, With<FpsText>>,
) {
    for mut span in &mut query {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                **span = format!("{value:.2}");
            }
        }
    }
}

fn octree_update_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    octree_config: Res<OctreeConfig>,
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
        }
    }
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    latest_msgs: Res<LatestMsgs>,
) {
    use std::f32::consts::PI;
    gizmos.grid(
        Quat::from_rotation_x(PI / 2.),
        UVec2::splat(20),
        Vec2::new(2., 2.),
        // Light gray
        LinearRgba::gray(0.35),
    );
    let (velocity, apf_path) = (&latest_msgs.0.velocity, &latest_msgs.0.apf_path);
    gizmos.line(
        Vec3::ZERO,
        Vec3::new(velocity.x, velocity.y, velocity.z),
        Color::srgb_u8(255, 0, 0),
    );
    gizmos.grid(
        Quat::from_rotation_x(PI / 2.),
        UVec2::splat(20),
        Vec2::new(2., 2.),
        // Light gray
        LinearRgba::gray(0.35),
    );
    if apf_path.len() > 1 {
        for i in 0..apf_path.len() - 1 {
            let (x, y, z) = mid360_to_bevy(apf_path[i].x, apf_path[i].y, apf_path[i].z);
            let (x1, y1, z1) = mid360_to_bevy(apf_path[i + 1].x, apf_path[i + 1].y, apf_path[i + 1].z);
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
    mut imu_events: EventReader<ImuIntegrator>,
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
}

fn imu_event_system(mut events: EventWriter<ImuIntegrator>, imu_receiver: Res<ImuReceiver>) {
    while let Ok(data) = imu_receiver.0.try_recv() {
        events.send(data);
    }
}

fn octree_event_system(mut events: EventWriter<Octree>, octree_receiver: Res<OctreeReceiver>) {
    while let Ok(data) = octree_receiver.0.try_recv() {
        events.send(data);
    }
}

fn msgs_event_system(msgs_receiver: Res<MsgsReceiver>, mut latest_msgs: ResMut<LatestMsgs>) {
    while let Ok(data) = msgs_receiver.0.try_recv() {
        latest_msgs.0 = data;
    }
}