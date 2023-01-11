use crate::{
    tool::{ Tool, Action, AABB, IntersectType },
    utils,
};
use glam::Vec3;
use crate::Mesh;

#[derive(Debug)]
pub struct NaiveOctreeCell {
    pub values: [f32; 8],
    pub children: Option<Box<[NaiveOctreeCell; 8]>>,
    depth: u8,
}

impl Default for NaiveOctreeCell {
    fn default() -> Self {
        Self {
            values: [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0],
            children: None,
            depth: 0,
        }
    }
}

impl NaiveOctreeCell {
    pub fn subdivide_cell(&mut self) {
        if self.children.is_some() {
            return;
        }

        // Subdivide 8 points into 8 cells
        let points = utils::subdivide_values(self.values);

        // Create new cells
        // We have constructed all the corners needed for our 8 new cells.
        let make_cell = |cell: usize| -> NaiveOctreeCell {
                NaiveOctreeCell {
                values: points[cell],
                    children: None,
                    depth: self.depth + 1,
                }
        };

        let new_cells = Box::new([
            make_cell(0),
            make_cell(1),
            make_cell(2),
            make_cell(3),
            make_cell(4),
            make_cell(5),
            make_cell(6),
            make_cell(7),
        ]);

        self.children = Some(new_cells);
    }

    pub fn collapse_cell(&mut self) {
        self.children = None;
    }

    pub fn is_leaf(&self) -> bool {
        self.children.is_none()
    }

    pub fn has_children(&self) -> bool {
        self.children.is_some()
    }

    pub fn apply_tool<T: Tool + Copy>(&mut self, tool: T, action: Action, cell_aabb: AABB, max_depth: u8) {
        cell_aabb.calculate_corners().into_iter().zip(self.values.iter_mut()).for_each(|(pos, value)| {
            let newval = tool.value(pos);
            action.apply_value(value, newval);
        });

        // Check if subdivision is needed
        if self.depth < max_depth{
            if let IntersectType::Intersects(_) = tool.aabb().intersect(cell_aabb) {
                // Tool intersects but does not contain, the cell intersects the isosurface
                // subdivide for more detail
                self.subdivide_cell();
                let child_aabbs = cell_aabb.octree_subdivide();
                // Recursive apply to each child cell
                self.children.as_mut()
                    .unwrap()
                    .iter_mut()
                    .zip(child_aabbs.into_iter())
                    .for_each(|(child, aabb)| child.apply_tool(tool, action, aabb, max_depth));
            }
        }
    }

    pub fn generate_mesh(&self, vertices: &mut Vec<Vec3>, max_depth: u8, cell_aabb: AABB) {
        use crate::marching_cubes::{ EDGE_TABLE, TRI_TABLE, vert_interp };

        if self.depth < max_depth {
            if let Some(children) = self.children.as_ref() {
                let child_aabbs = cell_aabb.octree_subdivide();
                children.iter()
                .zip(child_aabbs.into_iter())
                .for_each(|(child, aabb)| child.generate_mesh(vertices, max_depth, aabb));
                return;
            }
        }

        let mut cubeindex = 0;
        if self.values[0] > 0.0 { cubeindex |= 1;   }
        if self.values[1] > 0.0 { cubeindex |= 2;   }
        if self.values[2] > 0.0 { cubeindex |= 4;   }
        if self.values[3] > 0.0 { cubeindex |= 8;   }
        if self.values[4] > 0.0 { cubeindex |= 16;  }
        if self.values[5] > 0.0 { cubeindex |= 32;  }
        if self.values[6] > 0.0 { cubeindex |= 64;  }
        if self.values[7] > 0.0 { cubeindex |= 128; }

        let corners = cell_aabb.calculate_corners();
        let interp = |index1, index2| -> Vec3 {
            vert_interp(
                (corners[index1], self.values[index1]),
                (corners[index2], self.values[index2])
            )
        };

        if EDGE_TABLE[cubeindex] != 0 {
            let mut edge_verts = [None; 12];

            if (EDGE_TABLE[cubeindex] & 1   ) != 0 { edge_verts[0 ] = Some(interp(0, 1)) }
            if (EDGE_TABLE[cubeindex] & 2   ) != 0 { edge_verts[1 ] = Some(interp(0, 4)) }
            if (EDGE_TABLE[cubeindex] & 4   ) != 0 { edge_verts[2 ] = Some(interp(4, 5)) }
            if (EDGE_TABLE[cubeindex] & 8   ) != 0 { edge_verts[3 ] = Some(interp(5, 1)) }

            if (EDGE_TABLE[cubeindex] & 16  ) != 0 { edge_verts[4 ] = Some(interp(2, 3)) }
            if (EDGE_TABLE[cubeindex] & 32  ) != 0 { edge_verts[5 ] = Some(interp(2, 6)) }
            if (EDGE_TABLE[cubeindex] & 64  ) != 0 { edge_verts[6 ] = Some(interp(6, 7)) }
            if (EDGE_TABLE[cubeindex] & 128 ) != 0 { edge_verts[7 ] = Some(interp(7, 3)) }

            if (EDGE_TABLE[cubeindex] & 256 ) != 0 { edge_verts[8 ] = Some(interp(0, 2)) }
            if (EDGE_TABLE[cubeindex] & 512 ) != 0 { edge_verts[9 ] = Some(interp(4, 6)) }
            if (EDGE_TABLE[cubeindex] & 1024) != 0 { edge_verts[10] = Some(interp(5, 7)) }
            if (EDGE_TABLE[cubeindex] & 2048) != 0 { edge_verts[11] = Some(interp(1, 3)) }

            TRI_TABLE[cubeindex].into_iter().copied().for_each(|tri_idx| {
                vertices.push(edge_verts[tri_idx as usize].expect("Tried to use invalid edge vertex!"));
            });
        }
    }
}

#[derive(Debug)]
pub struct NaiveOctree {
    root: NaiveOctreeCell,
}

impl NaiveOctree {
    pub fn new() -> Self {
        Self {
            root: Default::default(),
        }
    }

    pub fn apply_tool<T: Tool + Copy>(&mut self, tool: T, action: Action, max_depth: u8) {
        self.root.apply_tool(tool, action, AABB { start: Vec3::ZERO, end: Vec3::ONE, }, max_depth);
    }

    pub fn generate_mesh(&self, max_depth: u8) -> Mesh {
        let mut verts = Vec::new();
        self.root.generate_mesh(&mut verts, max_depth, AABB { start: Vec3::ZERO, end: Vec3::ONE });
        return Mesh {
            vertices: verts,
            indices: None,
            normals: None,
        }
    }
}

#[test]
fn terrain_test() {
    let mut terrain = NaiveOctree::new();
    let tool = crate::tool::Sphere::new(
        Vec3::ZERO,
        0.3
    );
    let action = Action::Place;
    let start = std::time::Instant::now();
    terrain.apply_tool(tool, action, 5);
    let end = std::time::Instant::now();
    let duration = end - start;

    //println!("Terrain: {terrain:#?}");
    println!("Terrain Tool Duration: {} micros ({} calls per second)", duration.as_micros(), 1.0f64 / duration.as_secs_f64());

    let start = std::time::Instant::now();
    let mut mesh = terrain.generate_mesh(8);
    let end = std::time::Instant::now();
    let duration = end - start;
    println!("Terrain Mesh Duration: {} micros ({} calls per second)", duration.as_micros(), 1.0f64 / duration.as_secs_f64());
    println!("{mesh:?}");

    mesh.write_obj_to_file(&"naive_octree.obj");
}