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
                if val > 0.0
                {
                    *point = point.min(-val);
                }
                else if val > -1.0
                {
                    if *point > 0.0
                    {
                        *point = point.min(-val);
                    }
                    else
                    {
                        *point = point.min(val);
                    }
                }
            },
        }
    }
}