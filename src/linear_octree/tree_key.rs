use bitvec::prelude::*;

/// 64 bit index that describes the exact location of a point
/// in an octree of maximum depth 19.
/// ```text
/// Bit layout
/// 010 00 000 000 ... 000 000 001 101 100 000 00100
///  |   |  19  18      6   5   4   3   2   1   |
///  |   |  |   |       |   |   |   |   |   |   Depth index
///  |   |  |   |       |   |   --------------Cell indices
///  |   |  ------------------(Set to 0 if greater than Depth index)
///  |   Funny bits (Unused)
///  --Corner index
/// 
/// Bits 0-4: Depth index
/// This represents a 5-bit unsigned integer for how deep into
/// the octree we are traversing. It is also the number of Cell
/// indices that are considered valid.
///
/// Bits 5-7, 8-10, etc.: Cell indices
/// These represent 3-bit unsigned integers for which cell to
/// access at each depth level. All Cell indices after the
/// specified depth level MUST be 000.
/// 
/// Bits 60-61: Unused/Funny Bits
/// These bits are leftover from calculating the maximum allowable
/// depth using this key format. I've named them "Funny Bits",
/// because funny things will happen if they aren't 0.
/// 
/// Bits 62-64: Corner index
/// This represents a 3-bit unsigned integer for which corner of
/// the final cell to access.
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TreeKey(u64);

impl Default for TreeKey {
    fn default() -> Self {
        Self(0)
    }
}

impl TreeKey {
    pub fn from_u64(num: u64) -> Option<Self> {
        let key = TreeKey(num);
        Some(key).filter(|key| key._sanity_check())
    }

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

    fn _sanity_check(&self) -> bool {
        let bits = self.0.view_bits::<LocalBits>();

        // Check 1: depth <= 19
        let depth: u64 = bits[0..5].load();
        if depth > 19 {
            return false;
        }

        // Check 2: No invalid cell indices
        let invalid_bits = bits[5..60].chunks(3).enumerate().find_map(|(current_depth, bits)| {
            if bits.any() && current_depth as u64 >= depth {
                return Some(());
            }
            None
        }).is_some();

        if invalid_bits {
            return false;
        }

        // Check 3: Funny bits
        if bits[60..62].any() {
            return false;
        }

        // That works for me!
        true
    }

    pub fn depth(&self) -> u8 {
        self.0.view_bits::<LocalBits>()[0..5].load()
    }

    pub fn set_depth(&mut self, new_depth: u8) {
        assert!(new_depth <= 19);

        let old_depth = self.depth();

        let bits = self.0.view_bits_mut::<LocalBits>();
        bits[0..5].store(new_depth);

        if new_depth < old_depth {
            // Zero out old cell indices
            if new_depth != 0 {
                bits[5..][(3*new_depth) as usize..1+(3*old_depth-1) as usize].fill(false);
            }
        }
    }

    pub fn get_index(&self, depth: u8) -> u8 {
        let key_depth = self.depth();
        assert!(depth <= key_depth);
        // TODO: Should the root be index 0?
        assert!(depth > 0);

        self.0.view_bits::<LocalBits>()
            [5..] // Skip depth bits
            [((depth-1)*3) as usize..] // Seek to the correct depth
            [0..3] // Get the index bits
            .load()
    }

    pub fn set_index(&mut self, depth: u8, index: u8) {
        let key_depth = self.depth();
        assert!(depth <= key_depth);
        assert!(depth > 0);

        self.0.view_bits_mut::<LocalBits>()
            [5..] // Skip depth bits
            [((depth-1)*3) as usize..] // Seek to correct depth
            [0..3] // Get index bits
            .store(index);
    }

    pub fn set_corner(&mut self, corner_idx: u8) {
        assert!(corner_idx < 8);

        self.0.view_bits_mut::<LocalBits>()
            [61..64]
            .store(corner_idx);
    }

    pub fn corner(&self) -> u8 {
        self.0.view_bits::<LocalBits>()
            [61..64]
            .load()
    }

    #[inline(always)]
    pub fn at_max_depth(&self) -> bool {
        self.depth() == 19
    }

    pub fn push(&mut self, index: u8) {
        assert!(!self.at_max_depth(), "TreeKey overflow"); // Can't go deeper!

        let depth = self.depth() + 1;
        self.set_depth(depth);

        let bits = self.0.view_bits_mut::<LocalBits>();
        let start_bit = ((depth-1)*3) as usize;
        bits[5..][start_bit..(start_bit+3)].store(index);
    }

    pub fn pop(&mut self) -> u8 {
        assert!(self.depth() > 0, "pop on empty TreeKey");
        let depth = self.depth();
        let index = self.get_index(self.depth());
        self.set_depth(depth - 1);
        return index;
    }
}

#[test]
fn tree_key_modify_test() {
    let mut key = TreeKey::from_u64(0b010_00000000000000000000000000000000000000000000_111_111_111_111_00100).expect("Valid TreeKey rejected?");
    key.set_depth(1);
    println!("Key: {:b}", key.0);
    assert!(key._sanity_check());
    key.push(4);
    println!("Key: {:b}", key.0);
    assert!(key._sanity_check());
    key.set_index(1, 7);
    println!("Key: {:b}", key.0);
    assert!(key._sanity_check());
    assert_eq!(key.pop(), 4);
    println!("Key: {:b}", key.0);
    assert!(key._sanity_check());
    assert_eq!(key.get_index(1), 7);
}

#[test]
fn tree_key_default() {
    assert!(TreeKey::default()._sanity_check());
}

#[test]
fn tree_key_funny_bits() {
    // Invalid: Funny bits set
    assert!(TreeKey::from_u64(0b010_11_000000000000000000000000000000000000000000_001_101_100_000_00100).is_none());
}

#[test]
fn tree_key_overflow() {
    // Invalid: Too many cell indices
    assert!(TreeKey::from_u64(0b010_00000000000000000000000000000000000000000_111_001_101_100_000_00100).is_none());
}

#[test]
fn tree_key_max_depth() {
    // Invalid: Depth > 19
    assert!(TreeKey::from_u64(0b010_00000000000000000000000000000000000000000000_001_101_100_000_10100).is_none());
}