
use na::DVector;
use std::collections::HashMap;
use serde::Serialize;

/// Defines an interface for solving systems of first-order ODEs according to the state-space model
///
/// \dot{x}(t) = f(t, x(t), u(t)) \
/// y(t) = h(t, x(t), u(t))
///
pub trait StateSpaceRepresentation {
    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
}


/// Provides typical descriptions of components of a state vector for dynamical system
#[derive(Debug, Clone, PartialEq, Serialize)]
pub enum StateSpaceType {

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


/// Defines a state-space as a mapping of a state vector index to a state-space component name.
/// States are initialized as Empty and must be updated as needed.
///
/// A state-space is formally defined as a Euclidean space in which the variables on the axes
/// are the state variables. The state of a dynamical system can be represented in such a way 
/// where the system's "state" is described by a vector within the state-space.
///
/// For example, the state of a pendulum can be represented in state-space form as a 
/// vector x(t) = [theta(t), theta_dot(t)], where theta is the angle of the pendulum and 
/// theta_dot is the rotational velocity. The Statespace object captures the mapping of vector
/// index to state-space component name.
///
/// # Examples
///
///
/// A state-space of a system with x-y position and velocity, respectively
/// x(t) = [pos_0, pos_1, vel_0, vel_1]
///
/// ```
/// use mads::dynamics::statespace::*;
///
/// let state_size = 4;
/// let mut statespace = StateSpace::new(state_size);
/// statespace.add_state(0, StateSpaceType::Position0);
/// statespace.add_state(1, StateSpaceType::Position1);
/// statespace.add_state(2, StateSpaceType::Velocity0);
/// statespace.add_state(3, StateSpaceType::Velocity1);
/// ```
///
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct StateSpace {
    definition: HashMap<usize, StateSpaceType>,
    size: usize
}

impl StateSpace {

    pub fn new(dx: usize) -> Self {

        let mut definition = HashMap::new();
        for i in 0..dx {
            definition.entry(i).or_insert(StateSpaceType::Empty);
        }

        Self { definition, size: dx }

    }


    /// Updates a state entry for a given index value, i. If i has not been entered already, a new
    /// entry with the new state is created.
    pub fn add_state(&mut self, i: usize, state: StateSpaceType) {

        // If i has not been entered already, create entry with Empty, otherwise return current
        // value
        let statespace = self.definition.entry(i).or_insert(StateSpaceType::Empty);
        // Modify the Empty or current value with given value
        *statespace = state;
    }

    /// Returns the state type for a given index value, i
    pub fn get(&self, i: usize) -> Option<&StateSpaceType> {

        self.definition.get(&i)

    }

}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_Statespace() {

        let dx = 6;
        let mut statespace = StateSpace::new(dx);

        statespace.add_state(0, StateSpaceType::Position0);
        statespace.add_state(1, StateSpaceType::Position1);
        statespace.add_state(2, StateSpaceType::Position2);
        statespace.add_state(3, StateSpaceType::Velocity0);
        statespace.add_state(4, StateSpaceType::Velocity1);
        statespace.add_state(5, StateSpaceType::Velocity2);

        // println!("{:?}", statespace);
        assert_eq!(statespace.get(0).unwrap(), &StateSpaceType::Position0);
        assert_eq!(statespace.get(1).unwrap(), &StateSpaceType::Position1);
        assert_eq!(statespace.get(2).unwrap(), &StateSpaceType::Position2);
        assert_eq!(statespace.get(3).unwrap(), &StateSpaceType::Velocity0);
        assert_eq!(statespace.get(4).unwrap(), &StateSpaceType::Velocity1);
        assert_eq!(statespace.get(5).unwrap(), &StateSpaceType::Velocity2);

    }

}
