pub mod tool;
use glam::{ Vec3, vec3 };
use rayon::Scope;
use tool::{ Tool, Action, AABB };

use generational_arena::{ Index, Arena };
use parking_lot::{ RwLock, Mutex };
use lerp::Lerp;

pub use glam;

mod mesh;
pub use mesh::*;

mod marching_cubes;

pub const CUBE_CORNERS: [Vec3; 8] = [
    Vec3::ZERO,
    vec3(1.0,0.0,0.0),
    vec3(0.0,1.0,0.0),
    vec3(1.0,1.0,0.0),
    vec3(0.0,0.0,1.0),
    vec3(1.0,0.0,1.0),
    vec3(0.0,1.0,0.0),
    Vec3::ONE,
];

pub struct Cell {
    pub values: [f32; 8],
    pub children: Option<[Index; 8]>,
    depth: u8,
}

impl Default for Cell {
    fn default() -> Self {
        Self {
            values: [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0],
            children: None,
            depth: 0,
        }
    }
}

impl Cell {
    pub fn depth(&self) -> u8 {
        self.depth
    }
}

pub struct Terrain {
    pub cell_arena: RwLock<Arena<Mutex<Cell>>>,
    pub root: Index,
}

impl Terrain {
    pub fn new() -> Self {
        let mut cell_arena = Arena::new();
        let root = cell_arena.insert(Mutex::new(Cell::default()));
        let cell_arena = RwLock::new(cell_arena);
        Terrain {
            cell_arena,
            root,
        }
    }

    pub fn with_cell_capacity(capacity: usize) -> Self {
        let mut cell_arena = Arena::with_capacity(capacity);
        let root = cell_arena.insert(Mutex::new(Cell::default()));
        let cell_arena = RwLock::new(cell_arena);
        Terrain {
            cell_arena,
            root,
        }
    }

    fn subdivide_cell(&self, index: Index) -> [Index; 8] {
        let values;
        let new_depth;

        {
            let cell_arena = self.cell_arena.read();
            let cell = cell_arena.get(index).unwrap().lock();
            if let Some(children) = cell.children {
                return children;
            }

            new_depth = cell.depth + 1;
            values = cell.values;
        }

        let points = crate::utils::subdivide_values(values);

        // We have constructed all the corners needed for our 8 new cells.
        macro_rules! make_cell {
            // Macro to take an index to represent corner 0
            // and fill the rest of the corners accordingly
            ($first_corner:expr) => {
                Mutex::new(Cell {
                    values: [
                        points[$first_corner  ],
                        points[$first_corner+1],
                        points[$first_corner+3],
                        points[$first_corner+4],
                        points[$first_corner+9],
                        points[$first_corner+10],
                        points[$first_corner+12],
                        points[$first_corner+13],
                    ],
                    children: None,
                    depth: new_depth,
                })
            }
        }
        let new_cells = [
            make_cell!(0),
            make_cell!(1),
            make_cell!(3),
            make_cell!(4),
            make_cell!(9),
            make_cell!(10),
            make_cell!(12),
            make_cell!(13),
        ];
        
        // Re-borrow the cell arena
        let mut cell_arena = self.cell_arena.write();

        // Insert the newly created cells into the Arena
        let new_indices = new_cells.map(|new_cell| {
            cell_arena.insert(new_cell)
        });

        // These indices become the children indices of the original cell
        cell_arena.get_mut(index).unwrap().lock().children = Some(new_indices);

        // All done!
        return new_indices;
    }

    fn apply_to_cell<T: Tool + Sync + Send + Clone>(&self, tool: T, action: Action, cell_index: Index, mut cell_aabb: AABB) {
        let corner_signs: Vec<bool> = {
            let corners = cell_aabb.calculate_corners();
            let cell_arena = self.cell_arena.read();
            let mut cell = cell_arena.get(cell_index).unwrap().try_lock().unwrap();

            corners.into_iter().zip(cell.values.iter_mut()).map(|(pos, val)| {
                action.apply_value(val, tool.value_local(pos));
                val.is_sign_negative()
            }).collect()
        };
    }

    pub fn apply<'a, T: Tool + Sync + Send + Clone + 'a>(&mut self, tool: T, action: Action) {
        let root_aabb = AABB {
            start: Vec3::ZERO,
            end: Vec3::ONE,
        };

        let tool_clone = tool.clone();
    }
}

mod naive_octree;
pub use naive_octree::*;

pub mod utils;