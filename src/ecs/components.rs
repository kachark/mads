
use std::fmt;
use nalgebra::DVector;
use uuid::Uuid;
use serde::Serialize;
use crate::dynamics::statespace::Statespace;
use crate::math::frames::ReferenceFrame;

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

/// Defines the statespace for a dynamically modeled entity
pub type StatespaceComponent = Statespace;

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

pub type LinearInvertedPendulumComponent = crate::dynamics::models::InvertedPendulum;
pub type DoubleIntegrator1DComponent = crate::dynamics::models::DoubleIntegrator1D;
pub type DoubleIntegrator2DComponent = crate::dynamics::models::DoubleIntegrator2D;
pub type DoubleIntegrator3DComponent = crate::dynamics::models::DoubleIntegrator3D;
pub type NonlinearInvertedPendulumComponent = crate::dynamics::models::NonlinearInvertedPendulum;
pub type DoublePendulumComponent = crate::dynamics::models::DoublePendulum;
pub type ClohessyWiltshireComponent = crate::dynamics::models::ClohessyWiltshire;

// CONTROLLERS

pub type LQRComponent = crate::controls::models::LinearQuadraticRegulator;

