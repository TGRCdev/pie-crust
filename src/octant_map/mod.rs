use std::{
    collections::{ BTreeMap, BTreeSet },
};
use crate::{
    tool::{ Tool, Action, IntersectType::*, },
    utils
};

mod octant_key;
pub use octant_key::*;

#[derive(Debug)]
pub struct OctantMap {
    octants: BTreeMap<OctantKey, [f32; 8]>,
    leaves: BTreeSet<OctantKey>,
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
        let mut octants = BTreeMap::new();
        let mut leaves = BTreeSet::new();
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
        if cell.at_max_depth() || self.has_children(cell) { return; }

        let cell_values = self.octants.get(&cell).unwrap();

        let child_values = utils::subdivide_cell(cell_values);
        cell.children().into_iter()
            .zip(child_values.into_iter())
            .for_each(|(child, values)| {
                self.leaves.insert(child);
                self.octants.insert(child, values);
            });
        self.leaves.remove(&cell);
    }

    pub fn apply_tool<T: Tool + ?Sized>(&mut self, tool: &T, action: Action, max_depth: u8) {
        // Implementation 1: Recurse
        // This implementation is to compare performance
        // vs. NaiveOctree
        fn apply_tool_at_octant<T: Tool + ?Sized>(this: &mut OctantMap, tool: &T, action: Action, max_depth: u8, octant: OctantKey) {
            let depth = octant.depth();
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
}

#[test]
#[ignore]
fn octant_map_test() {
    use glam::Vec3;
    use crate::tool::{ Sphere, Action };
    let mut map = OctantMap::new();
    let tool = Sphere::new(Vec3::ZERO, 0.3291);
    map.apply_tool(&tool, Action::Place, 3);

    for leaf in map.leaves.iter().cloned() {
        assert!(!map.has_children(leaf))
    }

    println!("{map:?}");
}