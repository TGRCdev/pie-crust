#![warn(unused_extern_crates)]
pub mod tool;
use glam::{ Vec3, vec3 };

pub use glam;

mod mesh;
pub use mesh::*;

mod marching_cubes;

/// The corners of a unit cube in Z-index order.
pub const CUBE_CORNERS: [Vec3; 8] = [
    vec3(0.0,0.0,0.0),
    vec3(1.0,0.0,0.0),
    vec3(0.0,1.0,0.0),
    vec3(1.0,1.0,0.0),
    vec3(0.0,0.0,1.0),
    vec3(1.0,0.0,1.0),
    vec3(0.0,1.0,1.0),
    vec3(1.0,1.0,1.0),
];

pub mod naive_octree;

pub mod utils;