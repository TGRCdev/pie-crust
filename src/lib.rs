pub mod tool;
use glam::Vec3;
use rayon::Scope;
use tool::{ Tool, Action, AABB };

use generational_arena::{ Index, Arena };
use parking_lot::{ RwLock, Mutex };
use lerp::Lerp;

pub use glam;

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
        let cell_arena = self.cell_arena.read();
        let cell = cell_arena.get(index).unwrap().lock();
        if let Some(children) = cell.children {
            return children;
        }

        // Construct 19 new points, for a total
        // of 27 points
        // 
        // The points are indexed from the bottom-left-back point to
        // the top-right-front point, counting in order of X, then Y,
        // then Z.
        // 
        // E.G. bottom-left-back is 0, bottom-middle-back is 1, bottom-
        // right-back is 2, middle-left-back is 3, middle-middle-back is 4,
        // etc.
        let mut points = [0.0;27];
        let new_depth = cell.depth + 1;

        // First, copy the base points
        //      24-----------------26
        //      /                 /|
        //     /                 / |
        //    /                 /  |
        //   /                 /   |
        //  /                 /    |
        // 6-----------------8     20
        // |                 |    /
        // |                 |   /
        // |                 |  /
        // |                 | /
        // |                 |/
        // 0-----------------2
        // New points: 8
        // Total points: 8
        points[0] = cell.values[0];
        points[2] = cell.values[1];
        points[6] = cell.values[2];
        points[8] = cell.values[3];
        points[18] = cell.values[4];
        points[20] = cell.values[5];
        points[24] = cell.values[6];
        points[26] = cell.values[7];

        // Drop the parent cell to mutate the arena later
        drop(cell);
        drop(cell_arena);

        // Next, lerp between corners.
        //      24-------25--------26
        //      /                 /|
        //     /                 / |
        //   15                 17 23
        //   /                 /   |
        //  /                 /    |
        // 6--------7--------8     20
        // |                 |    /
        // |                 |   /
        // 3                 5  11
        // |                 | /
        // |                 |/
        // 0--------1--------2
        // New points: 12
        // Total points: 20
        points[1] = points[0].lerp(points[2], 0.5);
        points[3] = points[0].lerp(points[6], 0.5);
        points[5] = points[2].lerp(points[8], 0.5);
        points[7] = points[6].lerp(points[8], 0.5);
        
        points[9] = points[0].lerp(points[18], 0.5);
        points[11] = points[2].lerp(points[20], 0.5);
        points[15] = points[6].lerp(points[24], 0.5);
        points[17] = points[8].lerp(points[26], 0.5);

        points[19] = points[18].lerp(points[20], 0.5);
        points[21] = points[18].lerp(points[24], 0.5);
        points[23] = points[20].lerp(points[26], 0.5);
        points[25] = points[24].lerp(points[26], 0.5);

        // Now, lerp between midpoints.
        // We can either go from back-to-front or
        // left to right, and the values will be
        // roughly the same.
        //      24-------25--------26
        //      /        /        /|
        //     /        /        / |
        //   15--------16-------17 23
        //   /        /        /| /|
        //  /        /        / |/ |
        // 6--------7--------8  14 20
        // |        |        | /| /
        // |        |        |/ |/
        // 3--------4--------5  11
        // |        |        | /
        // |        |        |/
        // 0--------1--------2
        // New points: 6
        // Total points: 26
        points[4] = points[1].lerp(points[7], 0.5);
        points[10] = points[9].lerp(points[11], 0.5);
        points[12] = points[3].lerp(points[21], 0.5);
        points[14] = points[5].lerp(points[23], 0.5);
        points[16] = points[7].lerp(points[25], 0.5);
        points[22] = points[19].lerp(points[25], 0.5);

        // Finally, compute the midpoint.
        // (Diagram unnecessary)
        //
        // New points: 1
        // Total points: 27
        points[13] = points[4].lerp(points[22], 0.5);

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

        self.cell_arena.read().iter_mut().for_each(|cell| {
            
        })
    }
}