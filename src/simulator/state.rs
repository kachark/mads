
use crate::simulator::configuration::{EngineConfig, SimulatorConfig};
use crate::simulator::ecs::EntityComponentSystem;

#[derive(Debug, PartialEq, Eq)]
pub enum EngineState {

    Active,
    Inactive

}

impl Default for EngineState {
    fn default() -> Self {
        EngineState::Inactive
    }
}

pub struct SimulatorState
{

    pub status: EngineState,
    pub ecs: EntityComponentSystem,
    engine_config: EngineConfig,
    simulation_config: SimulatorConfig,

}

impl SimulatorState
{

    pub fn new(
        engine_config: EngineConfig,
        simulation_config: SimulatorConfig,
        ) -> Self {

        let status = EngineState::Inactive;
        let mut ecs = EntityComponentSystem::new();
        ecs.setup(&engine_config, &simulation_config);

        Self {
            status,
            engine_config,
            simulation_config,
            ecs,
        }

    }

    /// Execute ECS systems and update Simulation state (World, Resources, etc.)
    pub fn update(&mut self) {

        // Check for simulator exit condition and update status accordingly
        if let true = self.check_done() {

            println!("************************** DONE! *******************************");
            self.status = EngineState::Inactive;

        } else {

            // Execute ECS systems
            self.ecs.schedule.execute(&mut self.ecs.world, &mut self.ecs.resources);

        }

    }

    /// Checks for simulation exit condition (time)
    fn check_done(&mut self) -> bool {

        if self.ecs.get_current_time() >= self.engine_config.max_simulation_time {

            true

        } else {

            false

        }

    }

}

