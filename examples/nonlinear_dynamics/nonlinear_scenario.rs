
use std::collections::HashMap;
use nalgebra::DVector;
use legion::*;
use uuid::Uuid;

use mads::scene::scenario::Scenario;
use mads::ecs::systems::simple::*;
use mads::ecs::systems::simulate::simulate_dynamics_system;
use mads::ecs::components::*;
use mads::ecs::resources::*;
use mads::dynamics::statespace::{State, StateSpace};
use mads::dynamics::models::nonlinear::double_pendulum::DoublePendulum;

use crate::resources::NumAgents;

use std::f32::consts::FRAC_PI_4;

pub struct NonlinearScenario {

    pub num_agents: u32

}

impl NonlinearScenario {

    pub fn new(num_agents: u32) -> Self {


        Self {
            num_agents,
        }

    }

    /// Add 10 pendulums
    pub fn default() -> Self {

        Self {
            num_agents: 10,
        }

    }


    fn setup_agents(&self, world: &mut World, resources: &mut Resources) {

        let mut storage = resources.get_mut::<SimulationResult>().unwrap();

        // Define Components for "Agent" Entity
        let agents: Vec<(FullState, DynamicsModel::<DoublePendulum>, SimID)> = (0..self.num_agents).into_iter()
            .map(| i | -> (FullState, DynamicsModel::<DoublePendulum>, SimID) {

                let name = "Agent".to_string() + &i.to_string();
                let id = Uuid::new_v4();
                let sim_id = SimID { uuid: id, name };

                // Initial conditions
                let state = DVector::<f32>::from_vec(vec![FRAC_PI_4, 3.0, FRAC_PI_4, 3.0]);
                let statespace = StateSpace{
                    position: State::Empty,
                    velocity: State::Empty,
                    attitude: State::Empty,
                    angular_velocity: State::Empty
                };
                let fullstate = FullState { data: state, statespace };

                // Define dynamics model to simulate
                let dynamics = DynamicsModel { model: DoublePendulum::new() };

                (fullstate, dynamics, sim_id)
            })
            .collect();

        // Add agents to storage resource
        for agent in agents.iter() {
            storage.data.entry(agent.2.clone()).or_insert(vec![agent.0.clone()]);
        }

        world.extend(agents);

    }
}

impl Scenario for NonlinearScenario {

    /// Generate Resources for the scenario and insert into Simulator Resource pool
    fn setup(&self,
        world: &mut World,
        resources: &mut Resources,
    )
    {
        // scenario resources
        let num_agents = NumAgents(self.num_agents);
        let storage = SimulationResult{ data: HashMap::new() };
        resources.insert(num_agents);
        resources.insert(storage);

        self.setup_agents(world, resources);

    }

    /// Define systems to be run per loop iteration and return a Schedule to execute
    fn build(&self) -> Schedule {

        let schedule = Schedule::builder()
            .add_system(simulate_dynamics_system::<DoublePendulum>()) // can add any dynamics type here
            .add_system(update_result_system())
            .add_system(print_state_system())
            .add_system(increment_time_system())
            .build();

        schedule

    }

    /// Update and perform logic specific to the scenario
    fn update(&mut self, _world: &mut World, _resources: &mut Resources) {

        ()

    }

}


