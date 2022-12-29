use glam::Vec3;

use crate::tool::{ Tool, AABB };

pub struct Sphere {
    pub origin: Vec3,
    pub radius: f32,
}

impl Tool for Sphere {
    fn value_unclamped(&self, pos: Vec3) -> f32 {
        self.value_local(pos - self.origin)
    }

    fn value_local_unclamped(&self, local_pos: Vec3) -> f32 {
        self.radius - local_pos.length()
    }

    fn aabb(&self) -> AABB {
        AABB::from_radius(self.origin, self.radius)
    }
}