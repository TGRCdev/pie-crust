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
    fn value(&self, pos: Vec3, scale: f32) -> f32 {
        ((self.radius - (pos - self.origin).length()) / scale).clamp(-1.0,1.0)
    }

    fn aabb(&self) -> AABB {
        
        AABB::from_radius(self.origin, self.radius)
    }
}