use crate::{
    tool::{ Tool, Action, AABB, IntersectType },
    utils,
};
use glam::Vec3;
use crate::{ UnindexedMesh, marching_cubes::march_cube };

#[cfg(feature = "multi-thread")]
use lockfree::stack::Stack;
#[cfg(feature = "multi-thread")]
use rayon::prelude::*;

#[derive(Debug)]
pub struct NaiveOctreeCell {
    pub values: [f32; 8],
    pub children: Option<Box<[NaiveOctreeCell; 8]>>
}

impl Default for NaiveOctreeCell {
    fn default() -> Self {
        Self {
            values: [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0],
            children: None
        }
    }
}

impl NaiveOctreeCell {
    pub fn subdivide_cell(&mut self) {
        if self.children.is_some() {
            return;
        }

        // Subdivide 8 points into 8 cells
        let points = utils::subdivide_cell(&self.values);

        // Create new cells
        // We have constructed all the corners needed for our 8 new cells.
        let make_cell = |cell: usize| -> NaiveOctreeCell {
                NaiveOctreeCell {
                values: points[cell],
                    children: None,
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

    /// Handles applying to the current Cell and determining if children need subdivision.
    /// This is split from apply_tool and par_apply_tool to deduplicate code.
    fn apply_tool_impl<T: Tool + ?Sized>(
        &mut self,
        tool: &T,
        tool_aabb: AABB,
        aoe_aabb: AABB,
        action: Action,
        cell_aabb: AABB,
        current_depth: u8,
        max_depth: u8
    ) {
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

        let check_aabb = match action {
            Action::Remove => aoe_aabb,
            Action::Place => tool_aabb,
        };
        
        use IntersectType::*;
        // Check if subdivision is needed
        if self.children.is_none() && current_depth < max_depth {
            if (tool.is_convex() && (diff_signs || matches!(check_aabb.intersect(cell_aabb), ContainedBy))) ||
                (tool.is_concave() && !matches!(aoe_aabb.intersect(cell_aabb), DoesNotIntersect))
            {
                // Tool intersects but does not contain, the cell intersects the isosurface
                // subdivide for more detail
                self.subdivide_cell();
            }
        }

        self.values = newvals;
    }

    pub fn apply_tool<T: Tool + ?Sized>(
        &mut self,
        tool: &T,
        tool_aabb: AABB,
        aoe_aabb: AABB,
        action: Action,
        cell_aabb: AABB,
        current_depth: u8,
        max_depth: u8
    ) {
        self.apply_tool_impl(tool, tool_aabb, aoe_aabb, action, cell_aabb, current_depth, max_depth);

        if let Some(children) = self.children.as_mut() {
            let child_aabbs = cell_aabb.octree_subdivide();
            // Recursive apply to each child cell
            children.iter_mut()
                .zip(child_aabbs.into_iter())
                .for_each(|(child, aabb)| child.apply_tool(tool, tool_aabb, aoe_aabb, action, aabb, current_depth+1, max_depth));
            
            // Check if collapse is needed
            if children.iter().all(|child| child.is_leaf() && !child.intersects_surface()) {
                self.collapse_cell();
            }
        }
    }

    #[cfg(feature = "multi-thread")]
    pub fn par_apply_tool<T: Tool + Sync + ?Sized>(
        &mut self,
        tool: &T,
        tool_aabb: AABB,
        aoe_aabb: AABB,
        action: Action,
        cell_aabb: AABB,
        current_depth: u8,
        max_depth: u8
    ) {
        self.apply_tool_impl(tool, tool_aabb, aoe_aabb, action, cell_aabb, current_depth, max_depth);

        if let Some(children) = self.children.as_mut() {
            let child_aabbs = cell_aabb.octree_subdivide();
            // Recursive apply to each child cell
            children.par_iter_mut()
                .zip(child_aabbs.into_par_iter())
                .for_each(|(child, aabb)| child.par_apply_tool(tool, tool_aabb, aoe_aabb, action, aabb, current_depth+1, max_depth));
            
            // Check if collapse is needed
            if children.iter().all(|child| child.is_leaf() && !child.intersects_surface()) {
                self.collapse_cell();
            }
        }
    }

    pub fn generate_mesh(&self, faces: &mut Vec<[Vec3; 3]>, current_depth: u8, max_depth: u8, cell_aabb: AABB) {
        if current_depth < max_depth {
            if let Some(children) = self.children.as_ref() {
                let child_aabbs = cell_aabb.octree_subdivide();
                children.iter()
                .zip(child_aabbs.into_iter())
                .for_each(|(child, aabb)| child.generate_mesh(faces, current_depth+1, max_depth, aabb));
                return;
            }
        }

        let corners = cell_aabb.calculate_corners();
        faces.extend(march_cube(&corners, &self.values));
    }

    #[cfg(feature = "multi-thread")]
    pub fn par_generate_mesh(&self, vertices: &Stack<[Vec3; 3]>, current_depth: u8, max_depth: u8, cell_aabb: AABB) {
        use rayon::prelude::*;

        if current_depth < max_depth {
            if let Some(children) = self.children.as_ref() {
                let child_aabbs = cell_aabb.octree_subdivide();
                children.par_iter()
                .zip(child_aabbs.into_par_iter())
                .for_each(|(child, aabb)| {
                    child.par_generate_mesh(vertices, current_depth, max_depth, aabb)
                });
                return;
            }
        }
        
        let tris = march_cube(&cell_aabb.calculate_corners(), &self.values);

        vertices.extend(tris);
    }

    fn generate_octree_frame_mesh(&self, faces: &mut Vec<[Vec3; 3]>, max_depth: u8, cell_aabb: AABB) {
        use utils::{ line_vertices, LineDir };
        
        if let Some(children) = self.children.as_ref() {
            let child_aabbs = cell_aabb.octree_subdivide();
            children.iter().zip(child_aabbs.into_iter()).for_each(|(child, aabb)| {
                child.generate_octree_frame_mesh(faces, max_depth, aabb);
            })
        }
        else {
            let cube_scale = cell_aabb.size.x;
            let cube_corners = cell_aabb.calculate_corners();
            let cell_size = cell_aabb.size;
            let line_scale = cube_scale * 0.01;
            faces.extend(line_vertices(cube_corners[0], cell_size.x, line_scale, LineDir::Right));
            faces.extend(line_vertices(cube_corners[0], cell_size.y, line_scale, LineDir::Up));
            faces.extend(line_vertices(cube_corners[0], cell_size.z, line_scale, LineDir::Forward));
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
        let mut tool_aabb = tool.tool_aabb();
        let mut aoe_aabb = tool.aoe_aabb();

        let terrain_aabb = AABB{ start: Vec3::ZERO, size: Vec3::splat(self.scale) };
        
        // Intersect the tool AABBs to fit inside the terrain
        match terrain_aabb.intersect(aoe_aabb) {
            IntersectType::DoesNotIntersect => return,
            IntersectType::Intersects(new_aabb) => aoe_aabb = new_aabb,
            IntersectType::ContainedBy => aoe_aabb = terrain_aabb,
            IntersectType::Contains => (),
        }
        match terrain_aabb.intersect(tool_aabb) {
            IntersectType::DoesNotIntersect => if matches!(action, Action::Place) { return }, 
            IntersectType::Intersects(new_aabb) => tool_aabb = new_aabb,
            IntersectType::ContainedBy => tool_aabb = terrain_aabb,
            IntersectType::Contains => (),
        }

        self.root.apply_tool(tool, tool_aabb, aoe_aabb, action, terrain_aabb, 0, max_depth);
    }

    #[cfg(feature = "multi-thread")]
    pub fn par_apply_tool<T: Tool + ?Sized + Sync>(&mut self, tool: &T, action: Action, max_depth: u8) {
        let mut tool_aabb = tool.tool_aabb();
        let mut aoe_aabb = tool.aoe_aabb();

        let terrain_aabb = AABB{ start: Vec3::ZERO, size: Vec3::splat(self.scale) };
        
        // Intersect the tool AABBs to fit inside the terrain
        match terrain_aabb.intersect(aoe_aabb) {
            IntersectType::DoesNotIntersect => return,
            IntersectType::Intersects(new_aabb) => aoe_aabb = new_aabb,
            IntersectType::ContainedBy => aoe_aabb = terrain_aabb,
            IntersectType::Contains => (),
        }
        match terrain_aabb.intersect(tool_aabb) {
            IntersectType::DoesNotIntersect => if matches!(action, Action::Place) { return }, 
            IntersectType::Intersects(new_aabb) => tool_aabb = new_aabb,
            IntersectType::ContainedBy => tool_aabb = terrain_aabb,
            IntersectType::Contains => (),
        }

        rayon::in_place_scope(|_| {
            self.root.par_apply_tool(tool, tool_aabb, aoe_aabb, action, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) }, 0, max_depth);
        });
    }

    pub fn generate_mesh(&self, max_depth: u8) -> UnindexedMesh {
        let mut faces = Vec::new();
        self.root.generate_mesh(&mut faces, 0, max_depth, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) });
        return UnindexedMesh {
            faces,
            normals: None,
        }
    }

    #[cfg(feature = "multi-thread")]
    pub fn par_generate_mesh(&self, max_depth: u8) -> UnindexedMesh {
        let faces = Stack::new();
        rayon::in_place_scope(|_| {
            self.root.par_generate_mesh(&faces, 0, max_depth, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) });
        });

        UnindexedMesh {
            faces: faces.collect(),
            normals: None,
        }
    }

    pub fn generate_octree_frame_mesh(&self, max_depth: u8) -> UnindexedMesh {
        let mut faces = Vec::new();
        self.root.generate_octree_frame_mesh(&mut faces, max_depth, AABB { start: Vec3::ZERO, size: Vec3::splat(self.scale) });
        return UnindexedMesh {
            faces,
            normals: None,
        }
    }
}

#[test]
#[ignore]
fn terrain_test() {
    use crate::tool::Sphere;
    use utils::time_test;

    let mut terrain = NaiveOctree::new(100.0);
    let mut tool = Sphere::new(
        Vec3::splat(50.0),
        30.0,
    );
    
    time_test!(terrain.apply_tool(&tool, Action::Place, 8), "NaiveOctree Apply Tool");
    
    tool.radius = 20.0;
    tool.origin.y = 70.0;
    time_test!(terrain.apply_tool(&tool, Action::Remove, 8), "NaiveOctree Remove Tool");

    let mesh = time_test!(terrain.generate_mesh(255), "NaiveOctree Generate UnindexedMesh");

    time_test!(mesh.write_obj_to_file("naive_octree_unindexed.obj"), "NaiveOctree UnindexedMesh To File");

    let mesh = time_test!(mesh.index(), "NaiveOctree Mesh Indexing");
    
    time_test!(mesh.write_obj_to_file("naive_octree_indexed.obj"), "NaiveOctree IndexedMesh To File");
}

#[test]
#[ignore]
fn edge_tool_test() {
    use crate::tool::Sphere;
    use utils::time_test;
    use glam::vec3;

    let mut terrain = NaiveOctree::new(100.0);
    let tool = Sphere::new(
        vec3(0.0,50.0,50.0),
        24.583,
    );

    time_test!(terrain.apply_tool(&tool, Action::Place, 8), "Edge Tool Place");

    let mesh = time_test!(terrain.generate_mesh(255), "Edge Tool Generate Mesh");
    let mesh = time_test!(mesh.index(), "Edge Tool Index Mesh");

    mesh.write_obj_to_file("edge_tool.obj");
}

#[test]
#[ignore]
#[cfg(feature = "multi-thread")]
fn par_terrain_test() {
    use crate::tool::Sphere;
    use utils::time_test;

    let mut terrain = NaiveOctree::new(100.0);
    let mut tool = Sphere::new(
        Vec3::splat(50.0),
        30.0,
    );
    
    time_test!(terrain.par_apply_tool(&tool, Action::Place, 8), "NaiveOctree Apply Tool");
    
    tool.radius = 20.0;
    tool.origin.y = 70.0;
    time_test!(terrain.par_apply_tool(&tool, Action::Remove, 8), "NaiveOctree Remove Tool");

    let mesh = time_test!(terrain.par_generate_mesh(255), "NaiveOctree Generate UnindexedMesh");

    time_test!(mesh.write_obj_to_file("par_naive_octree_unindexed.obj"), "NaiveOctree UnindexedMesh To File");

    let mesh = time_test!(mesh.index(), "NaiveOctree Mesh Indexing");
    
    time_test!(mesh.write_obj_to_file("par_naive_octree_indexed.obj"), "NaiveOctree IndexedMesh To File");
}

#[test]
fn cell_mesh_test() {
    use crate::tool::Sphere;

    let mut cell = NaiveOctreeCell::default();
    let tool = Sphere {
        origin: Vec3::ZERO,
        radius: 0.3,
    };

    cell.apply_tool(&tool, tool.tool_aabb(), tool.aoe_aabb(), Action::Place, AABB::ONE_CUBIC_METER, 0, 0);

    let mut faces = Vec::new();
    cell.generate_mesh(&mut faces, 0, 0, AABB::ONE_CUBIC_METER);

    let mesh = UnindexedMesh {
        faces,
        normals: None,
    };
    mesh.write_obj_to_file("cell_mesh_test.obj");
}