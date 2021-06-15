
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

#[derive(Clone, Debug, PartialEq)]
pub struct FullState(pub DVector<f32>);

#[derive(Clone, Debug, PartialEq)]
pub struct SimID {
    pub uuid: Uuid,
    pub name: String
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Targetable(pub bool);

// NOTE: generic component for dynamics models
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicsModel<T>
where
    T: StateSpaceRepresentation
{
    pub model: T
}

#[derive(Clone, Debug, PartialEq)]
pub struct LQRController {
    pub model: LinearQuadraticRegulator
}



