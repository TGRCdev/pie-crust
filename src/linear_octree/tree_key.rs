use bitvec::prelude::*;
use crate::{
    tool::AABB,
    CUBE_CORNERS,
};
use glam::Vec3;

/// 64 bit index that describes the exact location of a point
/// in an octree of maximum depth 20.
/// 
/// The intent of TreeKey is to allow consistent access/modify time
/// to any point at any depth, to allow cells to share corner data
/// across depths without reference-counting and other shared-mutability
/// shenanigans, and to utilize a more uniform storage structure
/// to increase cache coherency.
/// 
/// ```text
/// Bits 0-2, 3-5, ... , 57-59 etc.: Cell indices
/// These represent 3-bit unsigned integers for which cell to
/// access at each depth level. All Cell indices after the
/// specified depth level MUST be 000.
/// 
/// Bits 60: Unused/Funny Bit
/// This bit is leftover from calculating the maximum allowable
/// depth using this key format. I've named it "the Funny Bit",
/// because funny things will happen if it isn't 0.
/// 
/// Bits 61-63: Corner index
/// This represents a 3-bit unsigned integer for which corner of
/// the final cell to access.
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TreeKey {
    key: u64,
    // When calculating a cell's corner keys, depth is used
    // to set all cell indices past depth to the corner index
    depth: u8,
}

impl Default for TreeKey {
    fn default() -> Self {
        Self {
            key: 0,
            depth: 0,
        }
    }
}

impl TreeKey {
    pub fn from_iter<I: IntoIterator<Item = u8>>(cell_iter: I, corner: u8) -> Self {
        let mut cell_iter = cell_iter.into_iter();
        let mut key = TreeKey::default();
        while let Some(child_idx) = cell_iter.next() {
            key.push(child_idx);
        }
        key.set_corner(corner);

        key
    }

    pub fn corner_keys(&self) -> [TreeKey; 8] {
        let mut corners = [*self; 8];
        corners.iter_mut().enumerate().for_each(|(idx, corner)| {
            corner.set_corner(idx as u8);
        });
        corners
    }

    /// Iterates over the cell indices up to the current depth
    pub fn iter(&self) -> impl Iterator<Item = u8> + '_ {
        self.key.view_bits::<LocalBits>()
            [0..self.depth as usize*3]
            .chunks_exact(3)
            .into_iter()
            .map(|bits| bits.load())
    }

    pub fn cell_aabb(&self) -> AABB {
        let mut start = Vec3::ZERO;
        let mut scale = 1.0;
        self.iter().for_each(|cell_idx| {
            scale *= 0.5;
            start += CUBE_CORNERS[cell_idx as usize] * scale;
        });

        AABB {
            start,
            size: Vec3::splat(scale),
        }
    }

    fn _sanity_check(&self) {
        let bits = self.key.view_bits::<LocalBits>();

        let corner_bits = &bits[61..64];
        // Check 1: No invalid cell indices
        bits[..60].chunks(3).enumerate().for_each(|(current_depth, bits)| {
            let current_depth = current_depth as u8 + 1;
            // Past current depth, all indices should be equal to the corner index
            if current_depth > self.depth {
                assert!(bits.iter().eq(corner_bits.iter()), "{bits} != {corner_bits}");
            }
        });

        // Check 2: Funny bit
        assert!(!bits[60]);
    }

    #[inline(always)]
    pub fn depth(&self) -> u8 {
        self.depth
    }

    pub fn set_depth(&mut self, new_depth: u8) {
        if self.depth == new_depth {
            return;
        }

        assert!(new_depth <= 20);
        let old_depth = self.depth;
        self.depth = new_depth;

        if old_depth > new_depth {
            let corner = self.corner();
            // Overwrite unused cell indices with corner index
            self.key.view_bits_mut::<LocalBits>()
                [(new_depth as usize)*3 .. (old_depth as usize)*3]
                .chunks_exact_mut(3)
                .for_each(|bits| {
                    bits.store(corner);
                })
        }
    }

    pub fn get_index(&self, depth: u8) -> u8 {
        assert!(depth <= 20);
        if depth == 0 {
            return 0;
        }

        self.key.view_bits::<LocalBits>()
            [((depth-1)*3) as usize..] // Seek to the correct depth
            [0..3] // Get the index bits
            .load()
    }

    pub fn set_index(&mut self, depth: u8, index: u8) {
        assert!(depth <= self.depth);
        assert!(depth > 0);

        self.key.view_bits_mut::<LocalBits>()
            [((depth-1)*3) as usize..] // Seek to correct depth
            [0..3] // Get index bits
            .store(index);
    }

    pub fn set_corner(&mut self, corner_idx: u8) {
        assert!(corner_idx < 8);

        let bits = self.key.view_bits_mut::<LocalBits>();
        
        // Store corner index
        bits[61..64].store(corner_idx);
        
        // Set it for all cell indices past depth
        bits[..60]
            .chunks_exact_mut(3)
            .skip(self.depth as usize)
            .for_each(|bits| {
                bits.store(corner_idx);
            })
    }

    pub fn corner(&self) -> u8 {
        self.key.view_bits::<LocalBits>()
            [61..64]
            .load()
    }

    #[inline(always)]
    pub fn at_max_depth(&self) -> bool {
        self.depth == 20
    }

    pub fn push(&mut self, index: u8) {
        assert!(!self.at_max_depth(), "TreeKey overflow"); // Can't go deeper!

        self.depth += 1;

        let bits = self.key.view_bits_mut::<LocalBits>();
        let start_bit = ((self.depth-1)*3) as usize;
        bits[start_bit..(start_bit+3)].store(index);
    }

    pub fn pop(&mut self) -> u8 {
        assert!(self.depth > 0, "pop on empty TreeKey");
        let index = self.get_index(self.depth);
        self.set_depth(self.depth - 1);
        return index;
    }
}

#[test]
fn tree_key_modify_test() {
    use glam::vec3;

    macro_rules! key_eq {
        ($key:expr, $test_key:expr) => {{
            let key: u64 = $key;
            let test_key: u64 = $test_key;
            assert_eq!(key, test_key, "KEY MISMATCH\n{:#066b}\n{:#066b}", key, test_key);
        }}
    }

    let mut key = TreeKey::from_iter([7,7,7,7,0], 2);
    key._sanity_check();
    assert_eq!(key.depth(), 5);
    key_eq!(key.key, 0b010_0_010010010010010010010010010010010010010010010_000_111_111_111_111);

    key.push(6);
    key._sanity_check();
    assert_eq!(key.depth(), 6);
    assert_eq!(key.get_index(5), 0);
    assert_eq!(key.get_index(6), 6);
    assert_eq!(key.get_index(7), key.corner());
    assert_eq!(key.cell_aabb(), AABB { start: vec3(0.9375,0.953125,0.953125), size: Vec3::splat(0.015625) });
    key_eq!(key.key, 0b010_0_010010010010010010010010010010010010010010_110_000_111_111_111_111);

    key.pop();
    key._sanity_check();
    assert_eq!(key.depth(), 5);
    assert_eq!(key.get_index(5), 0);
    assert_eq!(key.get_index(6), key.corner());
    assert_eq!(key.get_index(7), key.corner());
    assert_eq!(key.cell_aabb(), AABB { start: Vec3::splat(0.9375), size: Vec3::splat(0.03125) });
    key_eq!(key.key, 0b010_0_010010010010010010010010010010010010010010010_000_111_111_111_111);

    key.set_depth(2);
    key._sanity_check();
    assert_eq!(key.get_index(2), 7);
    assert_eq!(key.get_index(3), key.corner());
    assert_eq!(key.cell_aabb(), AABB { start: Vec3::splat(0.75), size: Vec3::splat(0.25)});
    key_eq!(key.key, 0b010_0_010010010010010010010010010010010010010010010010010010_111_111);

    key.set_corner(5);
    key._sanity_check();
    key_eq!(key.key, 0b101_0_101101101101101101101101101101101101101101101101101101_111_111);
}

#[test]
fn tree_key_default() {
    TreeKey::default()._sanity_check();
}