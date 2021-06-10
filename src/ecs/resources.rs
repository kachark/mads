

// Define Resources for Specs Entity-Component-System

/// Current simulation time
#[derive(Default)]
pub struct SimulationTime(pub f32);

/// Max simulation time
#[derive(Default)]
pub struct MaxSimulationTime(pub f32);

/// Simulation engine step size
#[derive(Default)]
pub struct EngineStep(pub f32);

/// Integrator step size (ie. dynamics integrations)
#[derive(Default)]
pub struct IntegratorStep(pub f32);

