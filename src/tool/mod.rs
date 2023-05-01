mod sphere;
pub use sphere::*;

mod aabb;
pub use aabb::*;

mod action;
pub use action::*;

use glam::{ Vec3, Affine3A, Quat, Vec3A };

/// A ToolFunc represents a function that can return a density value for a given
/// point. i.e. a [Sphere] will produce positive values within the Sphere's surface,
/// and negative values outside of it.
pub trait ToolFunc {
    /// Get the isovalue of `pos` in the ToolFunc.
    fn value(&self, pos: Vec3) -> f32;

    /// Returns the ToolFunc AABB, representing a rough
    /// estimated area of space that might produce values
    /// greater than 0.0
    fn tool_aabb(&self) -> AABB;

    /// Returns the Area-Of-Effect AABB, representing a rough
    /// estimated area of space that might produce values
    /// greater than -1.0
    fn aoe_aabb(&self) -> AABB;

    /// Returns true if the given ToolFunc is [convex](https://en.wikipedia.org/wiki/Convex_polygon).
    fn is_concave(&self) -> bool;

    /// Returns true if the given ToolFunc is [concave](https://en.wikipedia.org/wiki/Concave_polygon).
    #[inline(always)]
    fn is_convex(&self) -> bool {
        !self.is_concave()
    }
}

/// A wrapper for ToolFunc that gives it a Transform.
pub struct Tool<F> {
    pub func: F,
    transform: Affine3A,
    _inverse: Affine3A,
}

impl<F: Clone> Clone for Tool<F> {
    fn clone(&self) -> Self {
        Self {
            func: self.func.clone(),
            transform: self.transform.clone(),
            _inverse: self._inverse.clone(),
        }
    }
}

impl<F: Copy> Copy for Tool<F> {}

impl<F> Tool<F> {
    pub fn new(func: F) -> Self {
        Self {
            func,
            transform: Affine3A::IDENTITY,
            _inverse: Affine3A::IDENTITY,
        }
    }

    pub fn translated(mut self, translation: Vec3A) -> Self {
        self.transform.translation += translation;
        self._inverse = self.transform.inverse();
        self
    }

    pub fn rotated(self, rotation: Quat) -> Self {
        self.transformed(Affine3A::from_quat(rotation))
    }

    pub fn scaled(self, scale: Vec3) -> Self {
        self.transformed(Affine3A::from_scale(scale))
    }

    pub fn transformed(mut self, transform: Affine3A) -> Self {
        self.transform = transform * self.transform;
        self._inverse = self.transform.inverse();
        self
    }

    pub fn set_transform(&mut self, trns: Affine3A) {
        self.transform = trns;
        self._inverse = trns.inverse();
    }

    pub fn transform(&self) -> &Affine3A {
        &&self.transform
    }

    pub fn inverse_transform(&self) -> &Affine3A {
        &self._inverse
    }

    pub fn value(&self, pos: Vec3) -> f32 where F: ToolFunc {
        let inverse = self.inverse_transform();
        let local_pos = inverse.transform_point3(pos);
        self.func.value(local_pos)
    }

    pub fn tool_aabb(&self) -> AABB where F: ToolFunc {
        let mut local_aabb = self.func.tool_aabb();
        local_aabb.transform_with(self.transform);
        local_aabb
    }

    pub fn aoe_aabb(&self) -> AABB where F: ToolFunc {
        let mut local_aabb = self.func.aoe_aabb();
        local_aabb.transform_with(self.transform);
        local_aabb
    }

    #[inline(always)]
    pub fn is_concave(&self) -> bool where F: ToolFunc {
        self.func.is_concave()
    }

    #[inline(always)]
    pub fn is_convex(&self) -> bool where F: ToolFunc {
        self.func.is_convex()
    }
}

#[test]
fn tool_aabb_test() {
    use aabb::AABB;

    let mut tool = Tool::new(Sphere).scaled(Vec3::splat(5.0)).translated(Vec3A::splat(3.0));
    assert_eq!(tool.tool_aabb(), AABB { start: Vec3::splat(-2.0), size: Vec3::splat(10.0) });
    tool = tool.scaled(Vec3::splat(0.5));
    println!("{:?}", tool.tool_aabb());
}

#[test]
fn tool_test() {
    use glam::{ vec3, vec3a };

    let mut tool = Tool::new(Sphere).scaled(Vec3::splat(5.0));
    let pos = vec3(4.5,0.0,0.0);
    println!("tool({}) = {}", pos, tool.value(pos));
    tool = tool.translated(vec3a(1.0,0.0,0.0));
    println!("tool({}) = {}", pos, tool.value(pos));
}