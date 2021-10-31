
use legion::{World, Resources, Schedule};
use uuid::Uuid;
use crate::ecs::systems::simple::increment_time_system;
use crate::ecs::resources::*;
use crate::ecs::components::{EntityFrame, SimID};
use crate::simulator::configuration::{EngineConfig, SimulatorConfig};

/// Defines the ECS for the MADS engine and configures core resources and entities
pub struct EntityComponentSystem {

    pub world: World,
    pub resources: Resources,
    pub schedule: Schedule,

}

impl EntityComponentSystem {

    pub fn new() -> Self {

        let simple_schedule = Schedule::builder()
            .add_system(increment_time_system())
            .build(); // Simple schedule

        Self {
            world: World::default(),
            resources: Resources::default(),
            schedule: simple_schedule
        }

    }

    pub fn setup(&mut self, engine_config: &EngineConfig, sim_config: &SimulatorConfig) {

        self.add_engine_resources(engine_config);
        self.add_simulation_resources(sim_config);
        self.add_simulation_entities();

    }

    pub fn get_current_time(&self) -> f32 {

        let elapsed_times = self.resources.get::<SimulationTimeHistory>().unwrap();
        let k = elapsed_times.data.len();

        elapsed_times.data[k-1]

    }

    /// Inserts Resources to ECS derived from an EngineConfig
    fn add_engine_resources(&mut self, config: &EngineConfig) {

        self.resources.insert(SimulationTime(config.simulation_time));
        self.resources.insert(SimulationTimeHistory{ data: vec![config.simulation_time] });
        self.resources.insert(MaxSimulationTime(config.max_simulation_time));
        self.resources.insert(EngineStep(config.engine_step));

    }

    /// Inserts core Entities for simulation
    fn add_simulation_entities(&mut self) {

        // Add entity for origin
        let inertial_frame = EntityFrame::default();
        let sim_id = SimID { uuid: Uuid::new_v4(), name: "O".to_string() };
        let frame_entity = (inertial_frame, sim_id);
        self.world.extend(vec![frame_entity]);

    }

    /// Inserts Resources to ECS derived from a SimulatorConfig
    fn add_simulation_resources(&mut self, config: &SimulatorConfig) {

        self.resources.insert(IntegratorStep(config.integrator_step));
        self.resources.insert(Integrator(config.integrator));

    }

}

impl Default for EntityComponentSystem {
    fn default() -> Self {

        let simple_schedule = Schedule::builder()
            .build();

        Self {
            world: World::default(),
            resources: Resources::default(),
            schedule: simple_schedule
        }

    }
}
