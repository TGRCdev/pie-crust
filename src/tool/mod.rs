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

    /// Returns the Tool AABB, representing a rough
    /// estimated area of space that might produce values
    /// greater than 0.0
    fn tool_aabb(&self) -> AABB;

    /// The Area-Of-Effect AABB, representing a rough
    /// estimated area of space that might produce values
    /// greater than -1.0
    fn aoe_aabb(&self) -> AABB;

    /// Returns true if the given Tool is [convex](https://en.wikipedia.org/wiki/Convex_polygon).
    fn is_concave(&self) -> bool;

    /// Returns true if the given Tool is [concave](https://en.wikipedia.org/wiki/Concave_polygon).
    #[inline(always)]
    fn is_convex(&self) -> bool {
        !self.is_concave()
    }
}