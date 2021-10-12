#![allow(non_snake_case)]

use std::collections::HashMap;
use nalgebra::{DMatrix, DVector};
use legion::*;
use uuid::Uuid;

use mads::scene::scenario::Scenario;
use mads::ecs::systems::simple::*;
use mads::ecs::systems::simulate::simulate_lqr_dynamics_system;
use mads::ecs::components::*;
use mads::ecs::resources::*;
use mads::dynamics::statespace::{State, StateSpace};
use mads::dynamics::models::linear::double_integrator::DoubleIntegrator2D;
use mads::controls::models::lqr::LinearQuadraticRegulator;

pub struct MyScenario {

  pub num_entities: u32,

}

impl MyScenario {

  pub fn new() -> Self {

    Self { num_entities: 1 }

  }


  fn setup_entities(&self, world: &mut World, resources: &mut Resources) {

    // SimulationResult maps SimIDs to an entity's history of states
    // This is initialized when SimulatorState is first constructed
    // See src/ecs/resources.rs
    let mut storage = resources.get_mut::<SimulationResult>().unwrap();

    // Define dynamics models and controllers
    let double_integrator = DoubleIntegrator2D::new();
    let A = double_integrator.dynamics.A.clone();
    let B = double_integrator.dynamics.B.clone();
    let Q = DMatrix::<f32>::identity(4, 4);
    let R = DMatrix::<f32>::identity(2, 2);

    // Define each Entity as a tuple of Components and collect into a vector
    let entities: Vec<(FullState, DynamicsModel::<DoubleIntegrator2D>, LQRController, SimID)> = (0..self.num_entities).into_iter()
        .map(| i | -> (FullState, DynamicsModel::<DoubleIntegrator2D>, LQRController, SimID) {

            // Generate an ID for each Entity
            let name = "Entity".to_string() + &i.to_string();
            let id = Uuid::new_v4();
            let sim_id = SimID { uuid: id, name };

            // Initial x,y position and velocity
            let state = DVector::<f32>::from_vec(vec![2.0, -3.0, 5.0, 1.0]);
            let statespace = StateSpace{
                position: State::Empty,
                velocity: State::Empty,
                attitude: State::Empty,
                angular_velocity: State::Empty
            };
            let fullstate = FullState { data: state, statespace };

            // Define dynamics model component
            let dynamics = DynamicsModel { model: DoubleIntegrator2D::new() };

            // Define controller component
            let controller = LQRController { model: LinearQuadraticRegulator::new(A.clone(), B.clone(), Q.clone(), R.clone()) };

            (fullstate, dynamics, controller, sim_id)
        })
        .collect();

    // Create an entry in the SimulatorResult resource for each Entity
    // Each entity state is updated here as the simulation runs
    for entity in entities.iter() {
        storage.data.entry(entity.3.clone()).or_insert(vec![entity.0.clone()]);
    }

    // Add the Entities to the World
    world.extend(entities);

  }

}

impl Scenario for MyScenario {

  fn setup(&self, world: &mut World, resources: &mut Resources) {

    let storage = SimulationResult{ data: HashMap::new() };
    resources.insert(storage);
    self.setup_entities(world, resources);

  }

  // Build a Schedule to execute at the beginning of an engine time step
  fn build(&self) -> Schedule {
    let schedule = Schedule::builder()
        .add_system(simulate_lqr_dynamics_system::<DoubleIntegrator2D>())
        .add_system(update_result_system())
        .add_system(print_state_system())
        .add_system(increment_time_system())
        .build();

    schedule
  }

  // Update and perform logic specific to the scenario at the end of every time step
  fn update(&mut self, _world: &mut World, _resources: &mut Resources) {

      ()

  }

}

