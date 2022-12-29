mod sphere;
pub use sphere::*;

mod aabb;
pub use aabb::*;

mod action;
pub use action::*;

use glam::Vec3;

pub trait Tool {
    fn value(&self, pos: Vec3) -> f32 {
        self.value_unclamped(pos).clamp(-1.0,1.0)
    }
    fn value_local(&self, local_pos: Vec3) -> f32 {
        self.value_local_unclamped(local_pos).clamp(-1.0,1.0)
    }

    fn value_unclamped(&self, pos: Vec3) -> f32;
    fn value_local_unclamped(&self, local_pos: Vec3) -> f32;

    fn aabb(&self) -> AABB;
}