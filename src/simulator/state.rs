
use crate::configuration::{EngineConfig, SimulationConfig};
use crate::setup::setup;
use crate::ecs::systems::simple_systems::*;
use crate::ecs::resources::SimulationTimeHistory;
use legion::{World, Resources, Schedule};

#[derive(Debug, PartialEq, Eq)]
pub enum EngineState {

    Active,
    Inactive

}

/// Maintains ECS World and entities
pub struct SimulationState
{

    pub status: EngineState,

    engine_config: EngineConfig,
    simulation_config: SimulationConfig,

    pub world: World,
    pub resources: Resources,
    pub schedule: Schedule,
    pub time_history: Vec<f32>,

    start_time: f32,
    elapsed_time: f32,

}

impl SimulationState
{

    pub fn new(
        engine_config: EngineConfig,
        simulation_config: SimulationConfig,
        ) -> Self {

        let status = EngineState::Inactive;

        let (world, resources, time_history) = setup(&engine_config, &simulation_config);
        let schedule = Schedule::builder()
            .add_system(increment_time_system())
            .build(); // Simple schedule
        let start_time = time_history[0];
        let elapsed_time = 0f32;

        Self {
            status,
            engine_config,
            simulation_config,
            world,
            resources,
            schedule,
            time_history,
            start_time,
            elapsed_time
        }

    }

    pub fn update(&mut self) {

        // Check for simulator exit condition and update status accordingly
        if let EngineState::Inactive = self.check_done() {

            println!("************************** DONE! *******************************");
            self.status = EngineState::Inactive;

        } else {

            // Execute ECS systems
            self.schedule.execute(&mut self.world, &mut self.resources);

        }

    }

    fn check_done(&mut self) -> EngineState {

        let elapsed_times = self.resources.get::<SimulationTimeHistory>().unwrap();
        let k = elapsed_times.data.len();

        if elapsed_times.data[k-1] >= self.engine_config.max_simulation_time {

            EngineState::Inactive

        } else {

            EngineState::Active

        }

    }

}
