use glam::Vec3;

use crate::tool::{ ToolFunc, AABB };

/// A ToolFunc that represents a Sphere of radius 1.0.
/// For Spheres of different radiuses, use [Tool](super::Tool) with
/// a scaled Transform.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sphere;

impl ToolFunc for Sphere {
    fn value(&self, pos: Vec3) -> f32 {
        (1.0 - pos.length()).clamp(-1.0,1.0)
    }

    fn tool_aabb(&self) -> AABB {
        AABB::from_radius(Vec3::ZERO, 1.0) 
    }

    fn aoe_aabb(&self) -> AABB {
        AABB::from_radius(Vec3::ZERO, 2.0)
    }

    #[inline(always)]
    fn is_concave(&self) -> bool {
        false
    }
}