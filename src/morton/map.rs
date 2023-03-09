use super::key::{ MortonKeyU32, MortonOctantKey };
use ahash::HashMap;
use crate::{
    tool::{ AABB, Action, Tool, ToolFunc, IntersectType },
    utils::subdivide_cell_into_grid,
};

pub struct MortonOctree {
    points: HashMap<MortonKeyU32, f32>,
}

impl MortonOctree {
    pub fn apply_tool<F: ToolFunc>(&mut self, tool: &Tool<F>, action: Action, max_depth: u8) {
        let mut stack = Vec::with_capacity(10);
        let (tool_aabb, aoe_aabb) = (tool.tool_aabb(), tool.aoe_aabb());
        
        let mut current_octant = Some((MortonOctantKey::root(), AABB::ONE_CUBIC_METER));
        while let Some((octant, aabb)) = current_octant {
            use IntersectType::*;
            // Determine if subdivision is appropriate
            if octant.depth() < max_depth.min(MortonOctantKey::MAX_DEPTH) && 
                matches!(tool_aabb.intersect(aabb), Intersects(_) | ContainedBy) ||
                (matches!(action, Action::Remove) &&
                    matches!(aoe_aabb.intersect(aabb), Intersects(_) | ContainedBy))
            {
                // Subdivide
                let corners = octant.corners().map(|key| self.points.get(&key).copied().unwrap());
                let new_points = subdivide_cell_into_grid(&corners);
                let children = octant.children();

                
            }

            current_octant = stack.pop();
        }
    }
}