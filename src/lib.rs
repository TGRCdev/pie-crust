pub mod tool;
use glam::{ Vec3, vec3 };
use rayon::Scope;
use tool::{ Tool, Action, AABB };

use generational_arena::{ Index, Arena };
use parking_lot::{ RwLock, Mutex };
use lerp::Lerp;

pub use glam;

mod mesh;
pub use mesh::*;

mod marching_cubes;

pub const CUBE_CORNERS: [Vec3; 8] = [
    Vec3::ZERO,
    vec3(1.0,0.0,0.0),
    vec3(0.0,1.0,0.0),
    vec3(1.0,1.0,0.0),
    vec3(0.0,0.0,1.0),
    vec3(1.0,0.0,1.0),
    vec3(0.0,1.0,0.0),
    Vec3::ONE,
];

mod naive_octree;
pub use naive_octree::*;

pub mod utils;