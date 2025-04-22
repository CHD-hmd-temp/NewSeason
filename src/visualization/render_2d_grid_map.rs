#![allow(unused)]
use bevy::prelude::*;
use crossbeam_channel::Receiver;
use crate::map::occupancy_map::OccupancyGrid;

// Marker component for grid cells
#[derive(Component)]
struct GridCell;

// Resource holding channel receiver
#[derive(Resource)]
struct GridReceiver {
    rx: Receiver<OccupancyGrid>,
}

// Resource holding the latest grid
#[derive(Default, Resource)]
struct CurrentGrid(Option<OccupancyGrid>);

pub fn run_bevy_2d(rx: Receiver<OccupancyGrid>) {
    // Setup channel and spawn a thread to simulate sending grids
    // let (tx, rx) = unbounded::<OccupancyGrid>();
    // std::thread::spawn(move || {
    //     // TODO: send OccupancyGrid instances periodically
    //     // e.g., tx.send(your_grid).unwrap();

    // });

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(GridReceiver { rx })
        .insert_resource(CurrentGrid::default())
        .add_systems(Startup, setup_camera)
        .add_systems(Update, receive_grid)
        .add_systems(Update, draw_grid)
        .run();
}

#[allow(deprecated)]
fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

// System to poll the channel for new grids
fn receive_grid(
    mut grid_res: ResMut<CurrentGrid>,
    grid_rx: Res<GridReceiver>,
) {
    if let Ok(grid) = grid_rx.rx.try_recv() {
        grid_res.0 = Some(grid);
    }
}

// System to draw/update the cells whenever grid changes
#[allow(deprecated)]
fn draw_grid(
    mut commands: Commands,
    mut grid_res: ResMut<CurrentGrid>,
    existing: Query<Entity, With<GridCell>>,
) {
    if let Some(grid) = grid_res.0.take() {
        // Remove old cells
        for ent in existing.iter() {
            commands.entity(ent).despawn();
        }
        // Spawn new cells
        for y in 0..grid.height {
            for x in 0..grid.width {
                let idx = y * grid.width + x;
                let lo = grid.log_odds[idx];
                // threshold at 0.0
                let occupied = lo > 0.0;
                let color = if occupied { Color::BLACK } else { Color::rgb(0.8, 0.8, 0.8) };

                let world_x = grid.origin.x + (x as f32 + 0.5) * grid.res;
                let world_y = grid.origin.y + (y as f32 + 0.5) * grid.res;

                commands.spawn(
                    SpriteBundle {
                        sprite: Sprite {
                            color,
                            custom_size: Some(Vec2::splat(grid.res)),
                            ..default()
                        },
                        transform: Transform::from_translation(Vec3::new(world_x, world_y, 0.0)),
                        ..default()
                    }
                ).insert(GridCell);
            }
        }
    }
}
