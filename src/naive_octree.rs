use crate::{
    tool::{ Tool, Action, AABB, IntersectType },
    utils,
};
use glam::Vec3;
use crate::Mesh;
use std::sync::Mutex;
use arrayvec::ArrayVec;

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
        let points = utils::subdivide_cell(self.values);

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

    pub fn intersects_surface(&self) -> bool {
        self.values.windows(2).any(|vals| vals[0].signum() != vals[1].signum())
    }

    pub fn apply_tool<T: Tool + ?Sized>(&mut self, tool: &T, action: Action, cell_aabb: AABB, max_depth: u8) {
        // Store the results of tool application
        //
        // We need to compute these before subdivision to decide if we need
        // to subdivide, but we need to apply them after subdivision so it
        // doesn't muddy up the interpolation
        let mut newvals = self.values;
        let cube_scale = cell_aabb.size.x;
        cell_aabb.calculate_corners().into_iter().zip(newvals.iter_mut()).for_each(|(pos, value)| {
            let newval = tool.value(pos, cube_scale);
            action.apply_value(value, newval);
        });

        // TODO: Rewrite all these conditions for performance (if needed)
        let diff_signs = newvals.windows(2).any(|vals| vals[0].signum() != vals[1].signum());

        let tool_aabb = match action {
            Action::Remove => tool.aoe_aabb(),
            Action::Place => tool.tool_aabb(),
        };
        
        use IntersectType::*;
        // Check if subdivision is needed
        if self.children.is_none() && self.depth < max_depth {
            if (tool.is_convex() && (diff_signs || matches!(tool_aabb.intersect(cell_aabb), ContainedBy))) ||
                (tool.is_concave() && !matches!(tool.aoe_aabb().intersect(cell_aabb), DoesNotIntersect))
            {
                // Tool intersects but does not contain, the cell intersects the isosurface
                // subdivide for more detail
                self.subdivide_cell();
            }
        }

        self.values = newvals;

        if let Some(children) = self.children.as_mut() {
            let child_aabbs = cell_aabb.octree_subdivide();
            // Recursive apply to each child cell
            children.iter_mut()
                .zip(child_aabbs.into_iter())
                .for_each(|(child, aabb)| child.apply_tool(tool, action, aabb, max_depth));
            
            // Check if collapse is needed
            if children.iter().all(|child| child.is_leaf() && !child.intersects_surface()) {
                self.collapse_cell();
            }
        }
    }

    pub fn generate_mc_tris(&self, corners: [Vec3; 8]) -> ArrayVec<Vec3, 15> {
        use crate::marching_cubes::{ EDGE_TABLE, TRI_TABLE, vert_interp };

        let mut tris = ArrayVec::new();
        
        let mut cubeindex = 0;
        if self.values[0] > 0.0 { cubeindex |= 1;   }
        if self.values[1] > 0.0 { cubeindex |= 2;   }
        if self.values[2] > 0.0 { cubeindex |= 4;   }
        if self.values[3] > 0.0 { cubeindex |= 8;   }
        if self.values[4] > 0.0 { cubeindex |= 16;  }
        if self.values[5] > 0.0 { cubeindex |= 32;  }
        if self.values[6] > 0.0 { cubeindex |= 64;  }
        if self.values[7] > 0.0 { cubeindex |= 128; }

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
                tris.push(edge_verts[tri_idx as usize].expect("Tried to use invalid edge vertex!"));
            });
        }

        tris
    }

    pub fn generate_mesh(&self, vertices: &mut Vec<Vec3>, max_depth: u8, cell_aabb: AABB) {
        if self.depth < max_depth {
            if let Some(children) = self.children.as_ref() {
                let child_aabbs = cell_aabb.octree_subdivide();
                children.iter()
                .zip(child_aabbs.into_iter())
                .for_each(|(child, aabb)| child.generate_mesh(vertices, max_depth, aabb));
                return;
            }
        }

        vertices.extend(self.generate_mc_tris(cell_aabb.calculate_corners()));
    }

    pub fn par_generate_mesh(&self, vertices: &Mutex<Vec<Vec3>>, max_depth: u8, cell_aabb: AABB) {
        use rayon::prelude::*;

        if self.depth < max_depth {
            if let Some(children) = self.children.as_ref() {
                let child_aabbs = cell_aabb.octree_subdivide();
                children.par_iter()
                .zip(child_aabbs.into_par_iter())
                .for_each(|(child, aabb)| {
                    child.par_generate_mesh(vertices, max_depth, aabb)
                });
                return;
            }
        }
        
        let tris = self.generate_mc_tris(cell_aabb.calculate_corners());

        vertices.lock()
            .unwrap()
            .extend(tris);
    }

    pub fn generate_octree_frame_mesh(&self, vertices: &mut Vec<Vec3>, max_depth: u8, cell_aabb: AABB) {
        use utils::{ line_vertices, LineDir };
        
        if let Some(children) = self.children.as_ref() {
            let child_aabbs = cell_aabb.octree_subdivide();
            children.iter().zip(child_aabbs.into_iter()).for_each(|(child, aabb)| {
                child.generate_octree_frame_mesh(vertices, max_depth, aabb);
            })
        }
        else {
            let cube_scale = cell_aabb.size.x;
            let cube_corners = cell_aabb.calculate_corners();
            let cell_size = cell_aabb.size;
            let line_scale = cube_scale * 0.01;
            vertices.extend(line_vertices(cube_corners[0], cell_size.x, line_scale, LineDir::Right));
            vertices.extend(line_vertices(cube_corners[0], cell_size.y, line_scale, LineDir::Up));
            vertices.extend(line_vertices(cube_corners[0], cell_size.z, line_scale, LineDir::Forward));
        }
    }
}

#[derive(Debug)]
pub struct NaiveOctree {
    root: NaiveOctreeCell,
    pub scale: f32,
}

impl NaiveOctree {
    pub fn new(scale: f32) -> Self {
        Self {
            root: Default::default(),
            scale,
        }
    }

    pub fn apply_tool<T: Tool + Copy + ?Sized>(&mut self, tool: &T, action: Action, max_depth: u8) {
        self.root.apply_tool(tool, action, AABB{ start: Vec3::ZERO, size: Vec3::splat(self.scale) }, max_depth);
    }

    pub fn generate_mesh(&self, max_depth: u8) -> Mesh {
        let mut verts = Vec::new();
        self.root.generate_mesh(&mut verts, max_depth, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) });
        return Mesh {
            vertices: verts,
            indices: None,
            normals: None,
        }
    }

    pub fn par_generate_mesh(&self, max_depth: u8) -> Mesh {
        let verts = Mutex::new(Vec::new());
        rayon::in_place_scope(|_| {
            self.root.par_generate_mesh(&verts, max_depth, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) });
        });

        Mesh {
            vertices: verts.into_inner().unwrap(),
            indices: None,
            normals: None,
        }
    }

    pub fn generate_octree_frame_mesh(&self, max_depth: u8) -> Mesh {
        let mut verts = Vec::new();
        self.root.generate_octree_frame_mesh(&mut verts, max_depth, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) });
        return Mesh {
            vertices: verts,
            indices: None,
            normals: None,
        }
    }
}

#[test]
#[ignore]
fn terrain_test() {
    use std::time::Instant;
    use crate::tool::Sphere;

    let mut terrain = NaiveOctree::new(100.0);
    let mut tool = Sphere::new(
        Vec3::splat(50.0),
        30.0,
    );
    
    let start = Instant::now();
    terrain.apply_tool(&tool, Action::Place, 8);
    let duration = Instant::now() - start;
    println!("Terrain Tool Duration: {} micros ({} calls per second)", duration.as_micros(), 1.0f64 / duration.as_secs_f64());

    tool.radius = 20.0;
    tool.origin.y = 70.0;
    terrain.apply_tool(&tool, Action::Remove, 8);

    let start = Instant::now();
    let mut mesh = terrain.generate_mesh(255);
    let duration = Instant::now() - start;
    println!("Terrain Mesh Duration: {} micros ({} calls per second)", duration.as_micros(), 1.0f64 / duration.as_secs_f64());

    mesh.write_obj_to_file(&"naive_octree.obj");
}

#[test]
#[ignore]
fn par_terrain_test() {
    use std::time::Instant;
    use crate::tool::Sphere;

    let mut terrain = NaiveOctree::new(100.0);
    let mut tool = Sphere::new(
        Vec3::splat(50.0),
        30.0,
    );
    
    let start = Instant::now();
    terrain.apply_tool(&tool, Action::Place, 8);
    let duration = Instant::now() - start;
    println!("Terrain Tool Duration: {} micros ({} calls per second)", duration.as_micros(), 1.0f64 / duration.as_secs_f64());

    tool.radius = 20.0;
    tool.origin.y = 70.0;
    terrain.apply_tool(&tool, Action::Remove, 8);

    let start = Instant::now();
    let mut mesh = terrain.par_generate_mesh(255);
    let duration = Instant::now() - start;
    println!("Terrain Mesh Duration: {} micros ({} calls per second)", duration.as_micros(), 1.0f64 / duration.as_secs_f64());

    mesh.write_obj_to_file(&"par_naive_octree.obj");
}

#[test]
fn cell_mesh_test() {
    use crate::tool::Sphere;

    let mut cell = NaiveOctreeCell::default();
    let tool = Sphere {
        origin: Vec3::ZERO,
        radius: 0.3,
    };

    cell.apply_tool(&tool, Action::Place, AABB::ONE_CUBIC_METER, 0);

    let mut verts = Vec::new();
    cell.generate_mesh(&mut verts, 0, AABB::ONE_CUBIC_METER);

    let mesh = Mesh {
        vertices: verts,
        indices: None,
        normals: None,
    };
    mesh.write_obj_to_file(&"cell_mesh_test.obj");
}