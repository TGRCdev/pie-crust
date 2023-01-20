mod tree_key;
pub use tree_key::*;

use std::collections::BTreeMap;

#[derive(Debug)]
pub struct LinearOctree {
    buf: BTreeMap<TreeKey, f32>,
}

impl LinearOctree {
    pub fn new() -> Self {
        let mut buf = BTreeMap::new();
        let root_keys = TreeKeyBuilder::default().corner_keys();
        root_keys.into_iter().for_each(|key| {
            assert!(buf.insert(key.key(), -1.0).is_none());
        });

        Self {
            buf
        }
    }
}

#[test]
fn linear_octree_test() {
    let tree = LinearOctree::new();
    println!("{tree:#?}");
}