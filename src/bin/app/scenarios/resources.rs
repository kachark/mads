
use std::collections::HashMap;
use uuid::Uuid;

/// Number of Agents
#[derive(Default)]
pub struct NumAgents(pub u32);

/// Number of Targets
#[derive(Default)]
pub struct NumTargets(pub u32);

/// Number of Obstacles
#[derive(Default)]
pub struct NumObstacles(pub u32);

/// Assignment between agents and targets
#[derive(Default, Debug)]
pub struct Assignments {
    map: HashMap<Uuid, Uuid>
}
