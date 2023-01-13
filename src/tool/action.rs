#[derive(Clone, Copy, Debug)]
pub enum Action
{
    Remove,
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