use glam::Vec3;

use crate::tool::{ ToolFunc, AABB };

#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    pub radius: f32,
}

impl ToolFunc for Sphere {
    fn value(&self, pos: Vec3, scale: f32) -> f32 {
        ((self.radius - pos.length()) / scale).clamp(-1.0,1.0)
    }

    fn tool_aabb(&self) -> AABB {
        AABB::from_radius(Vec3::ZERO, self.radius)
    }

    fn aoe_aabb(&self) -> AABB {
        AABB::from_radius(Vec3::ZERO, self.radius + 1.0)
    }

    #[inline(always)]
    fn is_concave(&self) -> bool {
        false
    }
}