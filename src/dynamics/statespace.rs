
use na::DVector;
use std::collections::HashMap;
use serde::Serialize;

/// Defines an interface for solving systems of first-order ODEs according to the State Space model
pub trait StateSpaceRepresentation {
    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
}


#[derive(Debug, Clone, PartialEq, Serialize)]
pub enum StatespaceType {

    Empty,
    Position0,
    Position1,
    Position2,
    Velocity0,
    Velocity1,
    Velocity2,
    Attitude0,
    Attitude1,
    Attitude2,
    Attitude3,
    AngularVelocity0,
    AngularVelocity1,
    AngularVelocity2,

}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Statespace {
    definition: HashMap<usize, StatespaceType>,
    size: usize
}

impl Statespace {

    pub fn new(dx: usize) -> Self {

        let mut definition = HashMap::new();
        for i in 0..dx {
            definition.entry(i).or_insert(StatespaceType::Empty);
        }

        Self { definition, size: dx }

    }

    pub fn add_state(&mut self, i: usize, state: StatespaceType) {

        // If i has not been entered already, create entry with Empty, otherwise return current
        // value
        let statespace = self.definition.entry(i).or_insert(StatespaceType::Empty);
        // Modify the Empty or current value with given value
        *statespace = state;
    }

    pub fn get(&self, i: usize) -> Option<&StatespaceType> {

        self.definition.get(&i)

    }

}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_Statespace() {

        let dx = 6;
        let mut statespace = Statespace::new(dx);

        statespace.add_state(0, StatespaceType::Position0);
        statespace.add_state(1, StatespaceType::Position1);
        statespace.add_state(2, StatespaceType::Position2);
        statespace.add_state(3, StatespaceType::Velocity0);
        statespace.add_state(4, StatespaceType::Velocity1);
        statespace.add_state(5, StatespaceType::Velocity2);

        // println!("{:?}", statespace);
        assert_eq!(statespace.get(0).unwrap(), &StatespaceType::Position0);
        assert_eq!(statespace.get(1).unwrap(), &StatespaceType::Position1);
        assert_eq!(statespace.get(2).unwrap(), &StatespaceType::Position2);
        assert_eq!(statespace.get(3).unwrap(), &StatespaceType::Velocity0);
        assert_eq!(statespace.get(4).unwrap(), &StatespaceType::Velocity1);
        assert_eq!(statespace.get(5).unwrap(), &StatespaceType::Velocity2);

    }

}
