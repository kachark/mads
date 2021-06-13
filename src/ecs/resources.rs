
use std::collections::HashMap;
use uuid::Uuid;

use crate::ecs::components::*;

// Define Resources for Specs Entity-Component-System

/// Current simulation time
#[derive(Default)]
pub struct SimulationTime(pub f32);

/// Number of Agents
#[derive(Default)]
pub struct NumAgents(pub i32);

/// Max simulation time
#[derive(Default)]
pub struct MaxSimulationTime(pub f32);

/// Simulation engine step size
#[derive(Default)]
pub struct EngineStep(pub f32);

/// Integrator step size (ie. dynamics integrations)
#[derive(Default)]
pub struct IntegratorStep(pub f32);

/// Tracks the targetable set of entities
#[derive(Default)]
pub struct TargetableSet(pub HashMap::<Uuid, FullState>);
