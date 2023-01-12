mod sphere;
pub use sphere::*;

mod aabb;
pub use aabb::*;

mod action;
pub use action::*;

use glam::Vec3;

pub trait Tool {

    /// Get the relative isovalue of the point, assuming the point
    /// is a corner of a grid cube with extents of `scale` length
    fn value(&self, pos: Vec3, scale: f32) -> f32;

    fn aabb(&self) -> AABB;
}