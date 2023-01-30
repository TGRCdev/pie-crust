use bitvec::prelude::*;
use std::ops::Range;

/// 64 bit index that describes the exact location of an octant
/// in an octree of maximum depth 20 (Including an implied root at depth=0).
/// ```text
/// Bit layout
/// 00101 00 000 000 ... 000 000 001 101 100 000 001
///   |    |  19  18      7   6   5   4   3   2   1
///   |    |  |   |       |   |   |   |   |   |   |
///   |    |  |   |       |   |   -----------------Cell indices
///   |    |  -----------------(Set to 0 if greater than Depth index)
///   |    ---------Funny Bits (Unused, always 0)
///   --Depth index
/// 
/// Bits 0-2, 3-5, ..., 54-56: Cell indices
/// These represent 3-bit unsigned integers for which cell to
/// access at each depth level. All Cell indices after the
/// specified depth level MUST be 000.
/// 
/// Bits 57-58: Unused/Funny Bits
/// These bits are leftover from calculating the maximum allowable
/// depth using this key format. I've named them "Funny Bits",
/// because funny things will happen if they aren't 0.
/// 
/// Bits 59-63: Depth index
/// This represents a 5-bit unsigned integer for how deep into
/// the octree we are traversing. It is also the number of Cell
/// indices that are considered valid.
/// ```
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OctantKey(u64);

#[allow(dead_code)]
impl OctantKey {
    const DEPTH_BITS_RANGE: Range<usize> = 59..64;
    const DEPTH_BITS_SIZE: usize = 5;

    const FUNNY_BITS_RANGE: Range<usize> = 57..59;
    const FUNNY_BITS_SIZE: usize = 2;

    const CELL_INDICES_RANGE: Range<usize> = 0..57;
    const CELL_INDEX_RANGE: Range<usize> = 0..3;
    const CELL_INDEX_SIZE: usize = 3;

    fn _sanity_check(&self) {
        let bits = self.0.view_bits::<LocalBits>();

        // Check 1: depth <= 19
        let depth: u64 = bits[Self::DEPTH_BITS_RANGE].load();
        assert!(depth <= 19);

        // Check 2: No invalid cell indices
        bits[Self::CELL_INDICES_RANGE]
            .chunks(Self::CELL_INDEX_SIZE)
            .enumerate()
            .for_each(|(current_depth, bits)| {
            let current_depth: u64 = current_depth as u64+1;
            assert!(bits.not_any() || current_depth <= depth);
        });

        // Check 3: Funny bits
        assert!(bits[Self::FUNNY_BITS_RANGE].not_any());

        // That works for me!
    }

    pub fn depth(&self) -> u8 {
        self.0.view_bits::<LocalBits>()[Self::DEPTH_BITS_RANGE].load()
    }

    pub fn set_depth(&mut self, new_depth: u8) {
        assert!(new_depth <= 19);

        let old_depth = self.depth();

        let bits = self.0.view_bits_mut::<LocalBits>();
        bits[Self::DEPTH_BITS_RANGE].store(new_depth);

        if new_depth < old_depth {
            // Zero out old cell indices
            if new_depth != 0 {
                bits[Self::CELL_INDICES_RANGE]
                    [Self::CELL_INDEX_SIZE * new_depth as usize..]
                    .fill(false);
            }
        }
    }

    pub fn get_index(&self, depth: u8) -> u8 {
        let key_depth = self.depth();
        assert!(depth <= key_depth);
        // TODO: Should the root be index 0?
        assert!(depth > 0);

        self.0.view_bits::<LocalBits>()
            [Self::CELL_INDICES_RANGE]
            [Self::CELL_INDEX_SIZE * (depth-1) as usize..] // Seek to the correct depth
            [Self::CELL_INDEX_RANGE] // Get the index bits
            .load()
    }

    pub fn set_index(&mut self, depth: u8, index: u8) {
        let key_depth = self.depth();
        assert!(depth <= key_depth);
        assert!(depth > 0);

        self.0.view_bits_mut::<LocalBits>()
            [Self::CELL_INDICES_RANGE]
            [Self::CELL_INDEX_SIZE * (depth-1) as usize..] // Seek to correct depth
            [Self::CELL_INDEX_RANGE] // Get index bits
            .store(index);
    }

    #[inline(always)]
    pub fn at_max_depth(&self) -> bool {
        self.depth() == 19
    }

    pub fn push(&mut self, index: u8) {
        assert!(!self.at_max_depth(), "OctantKey overflow"); // Can't go deeper!

        let depth = self.depth() + 1;
        self.set_depth(depth);

        let bits = self.0.view_bits_mut::<LocalBits>();

        bits[Self::CELL_INDICES_RANGE]
            [Self::CELL_INDEX_SIZE * (depth as usize - 1)..]
            [Self::CELL_INDEX_RANGE].store(index);
    }

    pub fn pop(&mut self) -> u8 {
        assert!(self.depth() > 0, "pop on empty OctantKey");
        let depth = self.depth();
        let index = self.get_index(self.depth());
        self.set_depth(depth - 1);
        return index;
    }
}

#[test]
fn octant_key_from_u64() {
    let mut key = OctantKey::default();

    key.set_depth(1);
    println!("Key: {:#066b}", key.0);
    key._sanity_check();
    assert_eq!(key.0, 0b0000100000000000000000000000000000000000000000000000000000000000);

    key.push(4);
    println!("Key: {:#066b}", key.0);
    key._sanity_check();
    assert_eq!(key.0, 0b0001000000000000000000000000000000000000000000000000000000100000);

    key.set_index(1, 7);
    println!("Key: {:#066b}", key.0);
    key._sanity_check();
    assert_eq!(key.0, 0b0001000000000000000000000000000000000000000000000000000000100111);

    assert_eq!(key.pop(), 4);
    println!("Key: {:#066b}", key.0);
    key._sanity_check();
    assert_eq!(key.get_index(1), 7);
    assert_eq!(key.0, 0b0000100000000000000000000000000000000000000000000000000000000111);
}