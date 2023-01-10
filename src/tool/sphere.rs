use glam::Vec3;

use crate::tool::{ Tool, AABB };

#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    pub origin: Vec3,
    pub radius: f32,
}

impl Sphere {
    pub fn new(origin: Vec3, radius: f32) -> Self {
        Self {
            origin, radius
        }
    }
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