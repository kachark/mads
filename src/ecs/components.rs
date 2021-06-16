
use nalgebra::DVector;
use uuid::Uuid;

use crate::controls::models::lqr::LinearQuadraticRegulator;
use crate::dynamics::statespace::StateSpaceRepresentation;

// Define Components
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Velocity {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

/// Assigns a Dynamic Vector (see nalgebra) to an Entity
#[derive(Clone, Debug, PartialEq)]
pub struct FullState(pub DVector<f32>);

/// Assigns a unique identifier and name to an Entity
#[derive(Clone, Debug, PartialEq)]
pub struct SimID {
    pub uuid: Uuid,
    pub name: String
}

/// Flags an Entity as Targetable
/// Targetable entities are tracked within the TargetableSet
/// resource
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Targetable(pub bool);

/// Generic Component for a Dynamics model
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicsModel<T>
where
    T: StateSpaceRepresentation
{
    pub model: T
}

/// Component representing a Linear-Quadratic Regulator
#[derive(Clone, Debug, PartialEq)]
pub struct LQRController {
    pub model: LinearQuadraticRegulator
}



