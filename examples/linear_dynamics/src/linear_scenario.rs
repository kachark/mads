#![allow(non_snake_case)]

use std::collections::HashMap;
use nalgebra::{DMatrix, DVector};
use legion::*;
use uuid::Uuid;

use formflight::scene::scenario::Scenario;
use formflight::ecs::systems::simple_systems::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::components::*;
use formflight::ecs::resources::*;
use formflight::dynamics::statespace::{State, StateSpace};
use formflight::dynamics::models::linear::inverted_pendulum::InvertedPendulum;
use formflight::controls::models::lqr::LinearQuadraticRegulator;

use crate::resources::NumAgents;

pub struct LinearScenario {

    pub num_agents: u32,

}

impl LinearScenario {

    pub fn new(num_agents: u32) -> Self {

        Self {
            num_agents,
        }

    }

    pub fn default() -> Self {

        Self {
            num_agents: 1,
        }

    }

    fn setup_agents(&self, world: &mut World, resources: &mut Resources) {

        let mut storage = resources.get_mut::<SimulationResult>().unwrap();

        // Define dynamics models and controllers
        let inverted_pendulum = InvertedPendulum::new();
        let A = inverted_pendulum.dynamics.A.clone();
        let B = inverted_pendulum.dynamics.B.clone();
        let Q = DMatrix::<f32>::identity(4, 4);
        let R = DMatrix::<f32>::identity(1, 1);

        // Define Components for "Agent" Entity
        let agents: Vec<(FullState, DynamicsModel::<InvertedPendulum>, LQRController, SimID)> = (0..self.num_agents).into_iter()
            .map(| i | -> (FullState, DynamicsModel::<InvertedPendulum>, LQRController, SimID) {

                let name = "Agent".to_string() + &i.to_string();
                let id = Uuid::new_v4();
                let sim_id = SimID { uuid: id, name };

                // Initial conditions
                let state = DVector::<f32>::from_vec(vec![2.0, -3.0, 5.0, 1.0]);
                let statespace = StateSpace{
                    position: State::Empty,
                    velocity: State::Empty,
                    attitude: State::Empty,
                    angular_velocity: State::Empty
                };
                let fullstate = FullState { data: state, statespace };

                // Define dynamics model component
                let dynamics = DynamicsModel { model: InvertedPendulum::new() };

                // Define controller component
                let controller = LQRController { model: LinearQuadraticRegulator::new(A.clone(), B.clone(), Q.clone(), R.clone()) };

                (fullstate, dynamics, controller, sim_id)
            })
            .collect();

        // Add agents to storage resource
        for agent in agents.iter() {
            storage.data.entry(agent.3.clone()).or_insert(vec![agent.0.clone()]);
        }

        world.extend(agents);

    }
}

impl Scenario for LinearScenario {

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
            .add_system(dynamics_lqr_solver_system::<InvertedPendulum>()) // can add any dynamics type here
            .add_system(update_result_system())
            .add_system(increment_time_system())
            .build();

        schedule

    }

    /// Update and perform logic specific to the scenario
    fn update(&mut self, _world: &mut World, _resources: &mut Resources) {

        ()

    }

}


