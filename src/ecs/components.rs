
use std::fmt;
use nalgebra::DVector;
use uuid::Uuid;
use serde::Serialize;
use crate::controls::models::lqr::LinearQuadraticRegulator;
use crate::dynamics::statespace::{StateSpace, StateSpaceRepresentation};
use crate::dynamics::closed_form::ClosedFormRepresentation;
use crate::math::frames::ReferenceFrame;

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

/// Define reference frame for an entity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EntityFrame {
    pub frame: ReferenceFrame
}

/// Assigns a Dynamic Vector (see nalgebra) to an Entity
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct FullState {
    pub data: DVector<f32>,
    pub statespace: StateSpace
}

/// Assigns a unique identifier and name to an Entity
#[derive(Clone, Debug, Eq, Hash, PartialEq, Serialize)]
pub struct SimID {
    pub uuid: Uuid,
    pub name: String
}

impl fmt::Display for SimID {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "uuid: {}, name: {}", self.uuid, self.name)
    }
}

// IDENTIFIERS

/// Flags an Entity as Targetable
/// Targetable entities are tracked within the TargetableSet
/// resource
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Targetable(pub bool);

/// Flags an Entity as a Waypoint
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint(pub bool);

/// Flags an Entity as Static
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Static(pub bool);


// DYNAMICS MODELS

/// Generic Component for a Dynamics model
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicsModel<T>
where
    T: StateSpaceRepresentation
{
    pub model: T
}

/// Generic Component for a closed-form equation
#[derive(Clone, Debug, PartialEq)]
pub struct ClosedForm<T>
where
    T: ClosedFormRepresentation
{
    pub model: T
}

// CONTROLLERS

/// Component representing a Linear-Quadratic Regulator
#[derive(Clone, Debug, PartialEq)]
pub struct LQRController {
    pub model: LinearQuadraticRegulator
}

