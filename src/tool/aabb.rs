use glam::{ Vec3, vec3 };
use tinyvec::ArrayVec;

#[derive(Debug, Copy, Clone, Default)]
pub struct AABB {
    pub start: Vec3,
    pub end: Vec3,
}

pub enum IntersectType {
    DoesNotIntersect,
    Intersects(AABB), // Contains the AABB of the intersection
    Contains,
}

#[derive(PartialEq, PartialOrd)]
enum AxisIntersectType {
    DoesNotIntersect,
    Intersects(f32, f32),
    Contains,
}

impl AABB {
    pub fn expand(&mut self, vec: Vec3)
    {
        self.start = self.start.min(vec);
        self.end = self.end.max(vec);
    }

    pub const ONE_CUBIC_METER: Self = Self {
        start: Vec3::ZERO,
        end: Vec3::ONE,
    };

    pub fn contains(&self, point: Vec3) -> bool
    {
        return point.to_array().iter()
                .zip(self.start.to_array().iter()
                .zip(self.end.to_array().iter()))
                .all(|(point, (start, end))|
                {
                    point >= start && point <= end
                })
    }

    pub fn from_extents(pos: Vec3, extents: Vec3) -> Self {
        let half_extents = extents / 2.0;
        return Self {
            start: pos - half_extents,
            end: pos + half_extents,
        };
    }

    pub fn from_radius(pos: Vec3, radius: f32) -> Self {
        let radius_vec = vec3(radius, radius, radius);
        Self {
            start: pos - radius_vec,
            end: pos + radius_vec,
        }
    }

    pub fn calculate_corners(&self) -> [Vec3; 8] {
        let corners = [
            vec3(0.,0.,0.),
            vec3(1.,0.,0.),
            vec3(0.,1.,0.),
            vec3(1.,1.,0.),
            vec3(0.,0.,1.),
            vec3(1.,0.,1.),
            vec3(0.,1.,0.),
            vec3(1.,1.,1.)
        ].map(|offset| {
            self.start + (self.end * offset)
        });

        return corners;
    }

    #[inline]
    pub fn size(&self) -> Vec3 {
        self.end - self.start
    }

    fn intersect_axis(range1: (f32, f32), range2: (f32, f32)) -> AxisIntersectType
    {
        if range1.0 <= range2.0
        {
            if range1.1 >= range2.1
            {
                return AxisIntersectType::Contains;
            }
            else
            {
                return AxisIntersectType::Intersects(range2.0, range1.1);
            }
        }
        else if range1.0 < range2.1
        {
            return AxisIntersectType::Intersects(range1.0, range1.1.min(range2.1));
        }
        else {
            return AxisIntersectType::DoesNotIntersect;
        }
    }

    pub fn intersect(&self, other: AABB) -> IntersectType {
        let axes_intersects = [
            Self::intersect_axis(
                (self.start.x, self.end.x),
                (other.start.x, other.end.x)
            ),
            Self::intersect_axis(
                (self.start.y, self.end.y),
                (other.start.y, other.end.y)
            ),
            Self::intersect_axis(
                (self.start.z, self.end.z),
                (other.start.z, other.end.z)
            ),
        ];

        if axes_intersects.iter().any(|x| *x == AxisIntersectType::DoesNotIntersect) {
            return IntersectType::DoesNotIntersect;
        }
        else if axes_intersects.iter().all(|x| *x == AxisIntersectType::Contains) {
            return IntersectType::Contains;
        }
        else {
            let mut startnums: [f32;3] = [0.0;3];
            let mut endnums: [f32;3] = [0.0;3];
            for i in 0..3
            {
                match axes_intersects[i]
                {
                    AxisIntersectType::Contains => {
                        startnums[i] = other.start.x;
                        endnums[i] = other.end.x;
                    },
                    AxisIntersectType::Intersects(start, end) => {
                        startnums[i] = start;
                        endnums[i] = end;
                    },
                    AxisIntersectType::DoesNotIntersect => {
                        panic!("We shouldn't have reached this");
                    }
                }
            }

            return IntersectType::Intersects(AABB {
                start: startnums.into(),
                end: endnums.into(),
            });
        }
    }

    pub fn octree_subdivide(&self) -> [AABB; 8] {
        let half_size = self.size() / 2.0;
        let cells: ArrayVec<[AABB; 8]> = crate::CUBE_CORNERS.into_iter().map(|idx| {
            let start = self.start + (half_size * idx);
            let end = start + half_size;
            AABB {
                start,
                end
            }
        }).collect();
        cells.into_inner()
    }
}

#[test]
fn octree_subdivide_test() {
    let aabb = AABB{
        start: Vec3::ZERO,
        end: Vec3::ONE,
    };

    let subdiv = aabb.octree_subdivide();
    println!("{subdiv:?}");
}