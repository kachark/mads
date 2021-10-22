
use crate::math::integrate::IntegratorType;

/// Configuration for the Simulator loop and engine
pub struct EngineConfig {

    pub simulation_time: f32,
    pub max_simulation_time: f32,
    pub engine_step: f32

}

impl EngineConfig {

    pub fn new(simulation_time: f32, max_simulation_time: f32, engine_step: f32) -> Self {

        Self {
            simulation_time,
            max_simulation_time,
            engine_step
        }

    }

}

impl Default for EngineConfig {

    fn default() -> Self {

        Self {
            simulation_time: 0f32,
            max_simulation_time: 10f32,
            engine_step: 0.1
        }

    }

}


/// Configuration for the Simulation and dynamics solvers
pub struct SimulatorConfig {

    pub integrator: IntegratorType,
    pub integrator_step: f32

}

impl SimulatorConfig {

    pub fn new(integrator: IntegratorType, integrator_step: f32) -> Self {

        Self {
            integrator,
            integrator_step
        }

    }

}

impl Default for SimulatorConfig {

    fn default() -> Self {

        Self {
            integrator: IntegratorType::RK45,
            integrator_step: 0.1,
        }

    }

}

