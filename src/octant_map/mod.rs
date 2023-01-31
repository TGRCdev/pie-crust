use ahash::{ AHashMap, AHashSet };
use crate::{
    tool::{ Tool, Action, IntersectType::*, },
    Mesh,
    marching_cubes::march_cube,
    utils
};
use glam::Vec3;

mod octant_key;
pub use octant_key::*;

#[derive(Debug)]
pub struct OctantMap {
    octants: AHashMap<OctantKey, [f32; 8]>,
    leaves: AHashSet<OctantKey>,
}

impl Default for OctantMap {
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

impl OctantMap {
    pub fn new() -> Self {
        let root_key = OctantKey::default();
        let mut octants = AHashMap::new();
        let mut leaves = AHashSet::new();
        octants.insert(root_key, [-1.0; 8]);
        leaves.insert(root_key);

        Self {
            octants,
            leaves
        }
    }

    fn has_children(&self, mut cell: OctantKey) -> bool {
        cell.push(0);
        self.exists(cell)
    }

    fn exists(&self, cell: OctantKey) -> bool {
        self.octants.contains_key(&cell)
    }

    fn intersects_surface(&self, cell: OctantKey) -> bool {
        let values = self.octants.get(&cell).expect("Non-existant cell key");
        values.windows(2).any(|nums| nums[0].is_sign_negative() != nums[1].is_sign_negative())
    }

    fn is_collapsible(&self, cell: OctantKey) -> bool {
        if cell.at_max_depth() || !self.has_children(cell) { return false; }

        // Cell can't be collapsed if any children have children or intersect the isosurface
        !cell.children().into_iter()
            .any(|child| self.has_children(child) || self.intersects_surface(child))
    }



    fn collapse_cell(&mut self, cell: OctantKey) {
        if !self.is_collapsible(cell) { return; }

        let children = cell.children();
        for child in children {
            self.leaves.remove(&child);
            self.octants.remove(&child);
        }

        self.leaves.insert(cell);
    }

    fn subdivide_cell(&mut self, cell: OctantKey) {
        let values = self.octants.get(&cell).unwrap().clone();
        self.subdivide_with_old_values(cell, &values);
    }

    fn subdivide_with_old_values(&mut self, cell: OctantKey, values: &[f32; 8]) {
        if cell.at_max_depth() || self.has_children(cell) { return; }

        let child_values = utils::subdivide_cell(values);
        cell.children().into_iter()
            .zip(child_values.into_iter())
            .for_each(|(child, values)| {
                self.leaves.insert(child);
                self.octants.insert(child, values);
            });
        self.leaves.remove(&cell);
    }

    pub fn apply_tool_recurse<T: Tool + ?Sized>(&mut self, tool: &T, action: Action, max_depth: u8) {
        // Implementation 1: Recurse
        // This implementation is to compare performance
        // vs. NaiveOctree
        fn apply_tool_at_octant<T: Tool + ?Sized>(this: &mut OctantMap, tool: &T, action: Action, max_depth: u8, octant: OctantKey) {
            let aabb = octant.aabb();
            let scale = octant.scale();
            
            let mut newvals = this.octants.get(&octant).cloned().unwrap();
            newvals.iter_mut()
                .zip(aabb.calculate_corners().into_iter())
                .for_each(|(value, pos)| {
                    action.apply_value(value, tool.value(pos, scale))
                });
            
            // Whether or not the new values intersect the isosurface
            let newvals_intersect = newvals.windows(2).any(|vals| vals[0].is_sign_negative() != vals[1].is_sign_negative());
            
            let tool_aabb = match action {
                Action::Place => tool.tool_aabb(),
                Action::Remove => tool.aoe_aabb(),
            };

            // Check if subdivision is needed
            // (This must happen BEFORE modification to avoid bad data)
            if !octant.at_max_depth() && octant.depth() < max_depth && !this.has_children(octant) &&
                (tool.is_convex() && (newvals_intersect || matches!(tool_aabb.intersect(aabb), ContainedBy))) ||
                (tool.is_concave() && (newvals_intersect || !matches!(tool.aoe_aabb().intersect(aabb), DoesNotIntersect)))
                {
                    this.subdivide_cell(octant)
            }

            this.octants.insert(octant, newvals);

            if this.has_children(octant) {
                let children = octant.children();
                // Recursive apply to each child cell
                // If children already exist, they still get updated regardless
                // of max_depth
                children.into_iter().for_each(|child| apply_tool_at_octant(this, tool, action, max_depth, child));

                // Check if collapse needed
                if this.is_collapsible(octant) {
                    this.collapse_cell(octant);
                }
            }
        }

        apply_tool_at_octant(self, tool, action, max_depth, OctantKey::default());
    }

    pub fn apply_tool_filter<T: Tool + ?Sized>(&mut self, tool: &T, action: Action, max_depth: u8) {
        // Implementation 2: Filter

        fn apply_to_octant<T: Tool + ?Sized>(tool: &T, action: Action, max_depth: u8, octant: &OctantKey, values: &mut [f32; 8], leaves: &AHashSet<OctantKey>, subdivide: &mut Vec<(OctantKey, [f32; 8])>) {
            let aabb = octant.aabb();
            let scale = octant.scale();
            
            let mut newvals = values.clone();
            newvals.iter_mut()
                .zip(aabb.calculate_corners().into_iter())
                .for_each(|(value, pos)| {
                    action.apply_value(value, tool.value(pos, scale))
                });
            
            // Whether or not the new values intersect the isosurface
            let newvals_intersect = newvals.windows(2).any(|vals| vals[0].is_sign_negative() != vals[1].is_sign_negative());
            
            let tool_aabb = match action {
                Action::Place => tool.tool_aabb(),
                Action::Remove => tool.aoe_aabb(),
            };

            // Check if subdivision is needed
            // (This must happen BEFORE modification to avoid bad data)
            if !octant.at_max_depth() && octant.depth() < max_depth && leaves.contains(&octant) &&
                (tool.is_convex() && (newvals_intersect || matches!(tool_aabb.intersect(aabb), ContainedBy))) ||
                (tool.is_concave() && (newvals_intersect || !matches!(tool.aoe_aabb().intersect(aabb), DoesNotIntersect)))
                {
                    subdivide.push((*octant, values.clone()));
            }

            *values = newvals;

            // TODO: Collapse check somehow
        }

        // Iterate through octants, filter by which are below max_depth, apply, and subdivide
        let mut subdivide: Vec<(OctantKey, [f32; 8])> = Vec::new();
        {
            let leaves = &self.leaves;
            let octants = &mut self.octants;
            octants.iter_mut().filter(|(octant, _)| octant.depth() <= max_depth)
                .for_each(|(octant, values)| apply_to_octant(tool, action, max_depth, octant, values, leaves, &mut subdivide));
        }
        
        while !subdivide.is_empty() {
            let (octant, values) = subdivide.pop().unwrap();
            self.subdivide_with_old_values(octant, &values);
            octant.children().into_iter().for_each(|child| {
                let values = self.octants.get_mut(&child).unwrap();
                apply_to_octant(tool, action, max_depth, &child, values, &self.leaves, &mut subdivide)
            })
        }
    }

    pub fn generate_mesh(&self, max_depth: u8) -> Mesh {
        let vertices: Vec<Vec3> = self.octants.iter()
            .filter_map(|(octant, values)| {
                if octant.depth() == max_depth || (octant.depth() < max_depth && self.leaves.contains(octant)) {
                    let aabb = octant.aabb();
                    let corners = aabb.calculate_corners();
                    return Some(march_cube(&corners, values));
                }
                None
            }).flatten().collect();

        Mesh {
            vertices,
            indices: None,
            normals: None,
        }
    }
}

#[test]
#[ignore]
fn octant_map_test() {
    use glam::Vec3;
    use crate::tool::{ Sphere, Action };
    use utils::time_test;

    let mut map = OctantMap::new();
    let tool = Sphere::new(Vec3::ZERO, 0.3291);
    time_test!(map.apply_tool_filter(&tool, Action::Place, 8), "OctantMap Apply Tool");

    for leaf in map.leaves.iter().cloned() {
        assert!(!map.has_children(leaf))
    }

    let mesh = time_test!(map.generate_mesh(8), "OctantMap Mesh Generate");
    mesh.write_obj_to_file(&"octant_map.obj");
}