
use std::collections::HashMap;
use uuid::Uuid;
use serde::Serialize;
use crate::ecs::components::*;
use crate::math::integrators::IntegratorType;
use crate::math::frames::ReferenceFrame;

// Define Engine resources for Legion Entity-Component-System

/// Current simulation time
#[derive(Default)]
pub struct SimulationTime(pub f32);

/// Simulation time history
#[derive(Default, Debug)]
pub struct SimulationTimeHistory {
    pub data: Vec<f32>
}

/// Max simulation time
pub struct MaxSimulationTime(pub f32);

impl Default for MaxSimulationTime {

    fn default() -> Self {
        Self { 0: 10.0 }
    }

}

/// Simulation engine step size
pub struct EngineStep(pub f32);

impl Default for EngineStep {

    fn default() -> Self {
        Self { 0: 0.1 }
    }

}

/// Integrator step size (ie. dynamics integrations)
pub struct IntegratorStep(pub f32);

impl Default for IntegratorStep {

    fn default() -> Self {
        Self { 0: 0.1 }
    }

}

/// Integrator type to be used in simulation
#[derive(Default)]
pub struct Integrator(pub IntegratorType);

// TODO: should TargetableSet map SimID's instead of Uuid?
/// Tracks a set of entities defined by Uuid
#[derive(Default, Debug)]
pub struct TargetableSet(pub HashMap::<Uuid, FullState>);

/// Storage for Simulation outputs
#[derive(Default, Debug, Clone, Serialize)]
pub struct SimulationResult {
    pub data: HashMap<SimID, Vec<FullState>>
}

/// Define an inertial reference frame for the World
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct WorldFrame {
    pub frame: ReferenceFrame
}


