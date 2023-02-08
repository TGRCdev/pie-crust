use glam::{ Vec3, vec3, Affine3A };
use arrayvec::ArrayVec;
use crate::CUBE_CORNERS;

#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct AABB {
    pub start: Vec3,
    pub size: Vec3,
}

#[derive(Debug, PartialEq)]
pub enum IntersectType {
    DoesNotIntersect, // The two AABBs do not intersect
    Intersects(AABB), // Contains the AABB of the intersection
    Contains, // This AABB contains the entirety of the other AABB
    ContainedBy, // The other AABB contains the entirety of this AABB
}

impl AABB {

    pub const ONE_CUBIC_METER: Self = Self {
        start: Vec3::ZERO,
        size: Vec3::ONE,
    };

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

    pub fn from_extents(pos: Vec3, extents: Vec3) -> Self {
        let half_extents = extents / 2.0;
        return Self {
            start: pos - half_extents,
            size: extents,
        };
    }

    pub fn from_radius(pos: Vec3, radius: f32) -> Self {
        Self {
            start: pos - radius,
            size: Vec3::splat(radius*2.0),
        }
    }

    pub fn calculate_corners(&self) -> [Vec3; 8] {
        assert!(self.size.is_negative_bitmask() == 0);
        let corners = CUBE_CORNERS.map(|offset| {
            self.start + (self.size * offset)
        });

        return corners;
    }

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

    pub fn get_intersect_aabb(&self, other: AABB) -> Option<AABB> {
        let intersect = self.intersect(other);
        match intersect {
            IntersectType::DoesNotIntersect => None,
            IntersectType::Intersects(new_aabb) => Some(new_aabb),
            IntersectType::ContainedBy => Some(*self),
            IntersectType::Contains => Some(other),
        }
    }

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

    pub fn octree_child(&self, index: u8) -> AABB {
        assert!(index < 8);
        let half_size = self.size / 2.0;
        
        let corner = CUBE_CORNERS[index as usize];
        AABB {
            start: self.start + (half_size * corner),
            size: half_size,
        }
    }

    pub fn transform_with(&mut self, transform: Affine3A) {
        let point1 = transform.transform_point3(self.start);
        let point2 = transform.transform_point3(self.start + self.size);
        
        self.start = point1.min(point2);
        let end = point1.max(point2);
        self.size = end - self.start;
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