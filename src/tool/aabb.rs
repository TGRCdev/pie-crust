use glam::{ Vec3, vec3, Affine3A };
use arrayvec::ArrayVec;
use crate::CUBE_CORNERS;

/// Axis-Aligned Bounding Box
#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct AABB {
    pub start: Vec3,
    pub size: Vec3,
}

/// Describes how an AABB intersects with another AABB.
#[derive(Debug, PartialEq)]
pub enum IntersectType {
    /// The two AABBs do not intersect
    DoesNotIntersect,
    /// The two AABBs intersect, and the intersecting space is provided
    Intersects(AABB),
    /// This AABB encloses the other AABB
    Contains,
    /// This AABB is enclosed by the other AABB
    ContainedBy,
}

impl AABB {
    /// An identity AABB that extends from (0,0,0) to (1,1,1)
    pub const ONE_CUBIC_METER: Self = Self {
        start: Vec3::ZERO,
        size: Vec3::ONE,
    };

    /// Create a new AABB that encloses all of the points provided by the
    /// iterator.
    pub fn containing(points: impl IntoIterator<Item = Vec3>) -> Self {
        let mut points = points.into_iter();
        let start = points.next();
        if let Some(start) = start {
            let mut aabb = AABB { start, size: Vec3::ZERO };
            points.for_each(|p| aabb.expand(p));
            aabb
        }
        else {
            AABB { start: Vec3::ZERO, size: Vec3::ZERO }
        }
    }

    /// Returns true if `point` lies within the AABB.
    pub fn contains(&self, point: Vec3) -> bool
    {
        return point.to_array().into_iter()
            .zip(self.start.to_array().into_iter()
            .zip(self.size.to_array().into_iter()))
            .all(|(point, (start, size))|
            {
                point >= start && point <= start + size
            })
    }

    /// Expands the AABB to contain `point`.
    pub fn expand(&mut self, point: Vec3) {
        point.to_array().into_iter()
            .zip(self.start.as_mut().iter_mut())
            .zip(self.size.as_mut().iter_mut())
            .for_each(|((p, start), size)| {
                *start = start.min(p);
                *size = size.max(p - *start);
            });
    }

    /// Create an AABB centered on `pos`, using `extents` as the length
    /// of the box's edges.
    pub fn from_extents(pos: Vec3, extents: Vec3) -> Self {
        let half_extents = extents / 2.0;
        return Self {
            start: pos - half_extents,
            size: extents,
        };
    }

    /// Create an AABB centered on `pos`, using `radius * 2` as the length
    /// of the box's edges.
    /// 
    /// eg. `AABB::from_radius(Vec3::ZERO, 1.0)` would produce an AABB from
    /// (-1,-1,-1) to (1,1,1).
    pub fn from_radius(pos: Vec3, radius: f32) -> Self {
        Self {
            start: pos - radius,
            size: Vec3::splat(radius*2.0),
        }
    }
 
    /// Get the positions of the AABB's corners in Z-index order.
    pub fn calculate_corners(&self) -> [Vec3; 8] {
        assert!(self.size.is_negative_bitmask() == 0);
        let corners = CUBE_CORNERS.map(|offset| {
            self.start + (self.size * offset)
        });

        return corners;
    }

    /// Calculate the intersection between two AABBs and return the result.
    pub fn intersect(&self, other: AABB) -> IntersectType {
        #[derive(Debug)]
        enum AxisIntersectType {
            DoesNotIntersect,
            Intersects(f32, f32),
            Contains,
            ContainedBy,
        }

        // Intersect one axis at a time
        let axis_intersects: ArrayVec<AxisIntersectType, 3> = self
            .start.to_array().into_iter()
            .zip(self.size.to_array().into_iter())
            .zip(
                other.start.to_array().into_iter()
                .zip(other.size.to_array().into_iter())
            )
            .map(|(this, other)| {
                // Change (start, size) to (start, end)
                (
                    (this.0, this.0 + this.1),
                    (other.0, other.0 + other.1),
                )
            })
            .map(|(this, other)| {
                // Perform axis intersection
                if this.0 >= other.1 || this.1 <= other.0 {
                    return AxisIntersectType::DoesNotIntersect;
                }
                else if this.0 <= other.0 && this.1 >= other.1 {
                    return AxisIntersectType::Contains
                }
                else if(this.0 >= other.0 && this.1 < other.1) || (this.0 > other.0 && this.1 <= other.1) {
                    return AxisIntersectType::ContainedBy
                }
                else {
                    return AxisIntersectType::Intersects(
                        this.0.max(other.0), this.1.min(other.1)
                    );
                }
            }).collect();
        
        use AxisIntersectType::*;
        
        match axis_intersects.into_inner().unwrap() {
            [Contains, Contains, Contains] => return IntersectType::Contains,
            [ContainedBy, ContainedBy, ContainedBy] => return IntersectType::ContainedBy,
            [DoesNotIntersect, _, _] |
            [_, DoesNotIntersect, _] |
            [_, _, DoesNotIntersect] => return IntersectType::DoesNotIntersect,
            [x,y,z] => {
                let x = match x {
                    Intersects(start, end) => (start, end - start),
                    Contains => (other.start.x, other.size.x),
                    ContainedBy => (self.start.x, self.size.x),
                    DoesNotIntersect => unreachable!(), // We've already proved that none of the axes are DoesNotIntersect
                };
                let y = match y {
                    Intersects(start, end) => (start, end - start),
                    Contains => (other.start.y, other.size.y),
                    ContainedBy => (self.start.y, self.size.y),
                    DoesNotIntersect => unreachable!(),
                };
                let z = match z {
                    Intersects(start, end) => (start, end - start),
                    Contains => (other.start.z, other.size.z),
                    ContainedBy => (self.start.z, self.size.z),
                    DoesNotIntersect => unreachable!(),
                };
                return IntersectType::Intersects(
                    AABB {
                        start: vec3(x.0, y.0, z.0),
                        size: vec3(x.1, y.1, z.1),
                    }
                )
            }
        }
    }

    /// Calculate the intersection between two AABBs and return the
    /// resulting intersection AABB, if it exists.
    pub fn get_intersect_aabb(&self, other: AABB) -> Option<AABB> {
        let intersect = self.intersect(other);
        match intersect {
            IntersectType::DoesNotIntersect => None,
            IntersectType::Intersects(new_aabb) => Some(new_aabb),
            IntersectType::ContainedBy => Some(*self),
            IntersectType::Contains => Some(other),
        }
    }

    /// Subdivide the AABB into 8 equally-sized AABBs. The resulting
    /// array is in Z-index order.
    pub fn octree_subdivide(&self) -> [AABB; 8] {
        let half_size = self.size / 2.0;
        let mut cells: ArrayVec<AABB, 8> = ArrayVec::new();
        
        CUBE_CORNERS.into_iter().for_each(|point| {
            let start = self.start + (half_size * point);
            cells.push(
        AABB {
                    start,
                    size: half_size,
                }
            );
        });
        assert!(cells.windows(2).all(|a| {a[0].size == a[1].size}));
        cells.into_inner().unwrap()
    }

    /// Calculate the AABB of octant child `index`.
    /// 
    /// See also: [`octree_subdivide`](Self::octree_subdivide)
    pub fn octree_child(&self, index: u8) -> AABB {
        assert!(index < 8);
        let half_size = self.size / 2.0;
        
        let corner = CUBE_CORNERS[index as usize];
        AABB {
            start: self.start + (half_size * corner),
            size: half_size,
        }
    }

    /// Returns an AABB that contains the corners of the AABB
    /// after they have been transformed by `transform`.
    pub fn transformed(self, transform: Affine3A) -> Self {
        let corners = self.calculate_corners().map(|p| transform.transform_point3(p));
        Self::containing(corners)
    }

    /// Transforms the AABB's corners by `transform`, and expands to
    /// contain the resulting corner positions.
    /// 
    /// See also: [`transformed`](Self::transformed)
    pub fn transform_with(&mut self, transform: Affine3A) {
        *self = self.transformed(transform);
    }
}

#[test]
fn intersect_test() {
    use IntersectType::*;

    let aabb_1 = AABB {
        start: vec3(1.0, 2.0, 3.0),
        size: vec3(4.0, 5.0, 6.0),
    };
    let aabb_2 = AABB {
        start: vec3(3.0, 4.0, 5.0),
        size: vec3(2.0, 1.0, 0.5),
    };
    let aabb_3 = AABB {
        start: Vec3::ZERO,
        size: vec3(2.0, 2.0, 2.0),
    };
    let aabb_4 = AABB {
        start: vec3(4.0, 6.0, 8.0),
        size: vec3(10.0,15.0,7.0),
    };

    
    assert_eq!(aabb_1.intersect(aabb_2), Contains);
    assert_eq!(aabb_2.intersect(aabb_1), ContainedBy);

    assert_eq!(aabb_1.intersect(aabb_1), Contains);

    assert_eq!(aabb_1.intersect(aabb_3), DoesNotIntersect);
    assert_eq!(aabb_3.intersect(aabb_1), DoesNotIntersect);

    assert_eq!(aabb_2.intersect(aabb_3), DoesNotIntersect);
    assert_eq!(aabb_3.intersect(aabb_2), DoesNotIntersect);

    assert_eq!(aabb_1.intersect(aabb_4), Intersects(AABB { start: vec3(4.0, 6.0, 8.0), size: Vec3::ONE }));
    assert_eq!(aabb_4.intersect(aabb_1), Intersects(AABB { start: vec3(4.0, 6.0, 8.0), size: Vec3::ONE }));
}

#[test]
fn octree_subdivide_test() {
    let aabb = AABB::ONE_CUBIC_METER;

    let subdiv = aabb.octree_subdivide();
    assert_eq!(subdiv[5], AABB { start: vec3(0.5,0.0,0.5), size: Vec3::splat(0.5) });
    let subdiv = subdiv[5].octree_subdivide();
    assert_eq!(subdiv[6], AABB { start: vec3(0.5,0.25,0.75), size: Vec3::splat(0.25) });
    let subdiv = subdiv[6].octree_subdivide();
    assert_eq!(subdiv[3], AABB { start: vec3(0.625,0.375,0.75), size: Vec3::splat(0.125) });
}