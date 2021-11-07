
use std::fmt;
use nalgebra::DVector;
use uuid::Uuid;
use serde::Serialize;
use crate::controls::models::lqr::LinearQuadraticRegulator;
use crate::dynamics::statespace::{Statespace, StateSpaceRepresentation};
use crate::dynamics::closed_form::ClosedFormRepresentation;
use crate::dynamics::models::*;
use crate::controls::models::*;
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
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct EntityFrame {
    pub frame: ReferenceFrame
}

/// Assigns a Dynamic Vector (see nalgebra) to an Entity
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct FullState {
    pub data: DVector<f32>,
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

/// Flags an Entity as having a dynamics model
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicFlag(pub bool);

/// Flags an Entity as Static
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StaticFlag(pub bool);

/// Flags an Entity as Targetable
/// Targetable entities are tracked within the TargetableSet
/// resource
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TargetableFlag(pub bool);

/// Flags an Entity as a Waypoint
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaypointFlag(pub bool);

/// Flags an Entity as Loggable
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoggableFlag(pub bool);


// DYNAMICS

pub type StatespaceComponent = Statespace;

pub type LinearInvertedPendulumComponent = linear::inverted_pendulum::InvertedPendulum;
pub type DoubleIntegrator1DComponent = linear::double_integrator::DoubleIntegrator1D;
pub type DoubleIntegrator2DComponent = linear::double_integrator::DoubleIntegrator2D;
pub type DoubleIntegrator3DComponent = linear::double_integrator::DoubleIntegrator3D;
pub type NonlinearInvertedPendulumComponent = nonlinear::inverted_pendulum::InvertedPendulum;
pub type DoublePendulumComponent = nonlinear::double_pendulum::DoublePendulum;
pub type ClohessyWiltshireComponent = nonlinear::clohessy_wiltshire::ClohessyWiltshire;

// CONTROLLERS

pub type LQRComponent = lqr::LinearQuadraticRegulator;

