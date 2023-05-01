/// Action represents operations to perform on a Terrain with a given
/// Tool.
#[derive(Clone, Copy, Debug)]
pub enum Action
{
    /// Subtract material from the Terrain
    Remove,
    /// Add material to the Terrain
    Place,
}

impl Action
{
    pub fn apply_value(&self, point: &mut f32, val: f32)
    {
        match self {
            Action::Place => {
                *point = point.max(val);
            },
            Action::Remove => {
                *point = point.min(-val);
            },
        }
    }
}