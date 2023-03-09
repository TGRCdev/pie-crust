use glam::{ UVec3, uvec3 };
use bitvec::prelude::*;
use std::{
    ops::RangeInclusive,
    hash::Hash
};

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MortonKeyU32(pub(crate) u32);

impl Default for MortonKeyU32 {
    fn default() -> Self {
        Self::root()
    }
}
impl Hash for MortonKeyU32 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u32(self.0)
    }
}

impl MortonKeyU32 {
    const U10_MAX: u16 = 0x3ff;
    pub const MAX_POSITION: UVec3 = UVec3::splat(Self::U10_MAX as u32);

    const X_BITS_START: usize = 0;
    const X_BITS_END: usize = 9;
    const Y_BITS_START: usize = 10;
    const Y_BITS_END: usize = 19;
    const Z_BITS_START: usize = 20;
    const Z_BITS_END: usize = 29;

    const FUNNY_BITS: RangeInclusive<usize> = 30..=31;

    const X_BITS: RangeInclusive<usize> = Self::X_BITS_START..=Self::X_BITS_END;
    const Y_BITS: RangeInclusive<usize> = Self::Y_BITS_START..=Self::Y_BITS_END;
    const Z_BITS: RangeInclusive<usize> = Self::Z_BITS_START..=Self::Z_BITS_END;

    pub fn root() -> Self {
        Self(0)
    }

    pub fn position(&self) -> UVec3 {
        let bits = self.0.view_bits::<LocalBits>();

        uvec3(
            bits[Self::X_BITS].load(),
            bits[Self::Y_BITS].load(),
            bits[Self::Z_BITS].load()
        )
    }
}

#[derive(Debug, Clone)]
pub struct MortonOctantKey {
    key: MortonKeyU32,
    depth: u8,
}

impl Default for MortonOctantKey {
    fn default() -> Self {
        Self::root()
    }
}

impl MortonOctantKey {
    pub const MAX_DEPTH: u8 = 9;

    pub fn root() -> Self {
        Self {
            key: MortonKeyU32::root(),
            depth: 0,
        }
    }

    pub fn child(mut self, corner: u8) -> Self {
        assert!(self.depth < Self::MAX_DEPTH);
        self.depth += 1;
        self = self.sibling(corner);
        self
    }

    pub fn parent(mut self) -> Self {
        assert!(self.depth > 0);
        self = self.sibling(0);
        self.depth -= 1;
        self
    }

    pub fn sibling(mut self, corner: u8) -> Self {
        assert!(self.depth > 0);
        let bits = self.key.0.view_bits_mut::<LocalBits>();
        let corner_bits = corner.view_bits::<LocalBits>();

        bits.set(MortonKeyU32::X_BITS_START + self.depth as usize, corner_bits[0]);
        bits.set(MortonKeyU32::Y_BITS_START + self.depth as usize, corner_bits[1]);
        bits.set(MortonKeyU32::Z_BITS_START + self.depth as usize, corner_bits[2]);

        self
    }

    pub fn corner(&self, corner: u8) -> MortonKeyU32 {
        let mut key = self.key.clone();
        let bits = key.0.view_bits_mut::<LocalBits>();
        let corner_bits = corner.view_bits::<LocalBits>();
        
        bits[MortonKeyU32::X_BITS_START + self.depth as usize .. MortonKeyU32::X_BITS_END]
            .fill(corner_bits[0]);
        bits[MortonKeyU32::Y_BITS_START + self.depth as usize .. MortonKeyU32::Y_BITS_END]
            .fill(corner_bits[1]);
        bits[MortonKeyU32::Z_BITS_START + self.depth as usize .. MortonKeyU32::Z_BITS_END]
            .fill(corner_bits[2]);

        key
    }

    pub fn depth(&self) -> u8 {
        self.depth
    }

    pub fn at_max_depth(&self) -> bool {
        self.depth == Self::MAX_DEPTH
    }

    pub fn at_root(&self) -> bool {
        self.depth == 0
    }

    pub fn children(&self) -> [MortonOctantKey; 8] {
        [
            self.clone().child(0),
            self.clone().child(1),
            self.clone().child(2),
            self.clone().child(3),
            self.clone().child(4),
            self.clone().child(5),
            self.clone().child(6),
            self.clone().child(7),
        ]
    }

    pub fn corners(&self) -> [MortonKeyU32; 8] {
        [
            self.corner(0),
            self.corner(1),
            self.corner(2),
            self.corner(3),
            self.corner(4),
            self.corner(5),
            self.corner(6),
            self.corner(7),
        ]
    }
}