
use std::collections::HashMap;
use uuid::Uuid;
use nalgebra::DVector;

/// Number of Agents
#[derive(Default)]
pub struct NumAgents(pub i32);

/// Number of Targets
#[derive(Default)]
pub struct NumTargets(pub i32);

/// Number of Obstacles
#[derive(Default)]
pub struct NumObstacles(pub i32);

