#![allow(unused)]
pub fn mid360_to_bevy(
    x: f32,
    y: f32,
    z: f32,
) -> (f32, f32, f32) {
    return (-y, z, -x)
}

pub fn bevy_to_mid360(
    x: f32,
    y: f32,
    z: f32,
) -> (f32, f32, f32) {
    return (-z, -x, y)
}

pub fn mid360_to_frd(
    x: f32,
    y: f32,
    z: f32,
) -> (f32, f32, f32) {
    return (x, -y, -z)
}

pub fn frd_to_bevy(
    x: f32,
    y: f32,
    z: f32,
) -> (f32, f32, f32) {
    return (y, -z, -x)
}

pub fn frd_to_mid360(
    x: f32,
    y: f32,
    z: f32,
) -> (f32, f32, f32) {
    return (x, -y, -z)
}