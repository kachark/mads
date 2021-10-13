# MADS: Multi-Agent Dynamics Simulator

This library provides tools to perform large-scale multi-agent simulations using an Entity Component System.

The following are supported:

### Dynamics Models
- 2D/3D Double Integrator
- Linear/Nonlinear Inverted Pendulum
- Nonlinear Double Pendulum
- Clohessy-Whiltshire equations

### Controllers
- Continuous Infinite-Horizon Linear Quadratic Regulator

## Setup

MADS consists of a higher level "Engine", managing the overall runtime and step size, and a lower level "Simulator" which
propogates the dynamics between engine time steps.

### Configuration

To setup MADS, you must first configure these two components into the initial SimulatorState. Upon construction, the state initializes
Legion ECS Resources, Schedule, and World with some of these parameters. Further user configuration and world building is done with Scenarios:

```rust
use mads::simulator::configuration::{EngineConfig, SimulatorConfig};
use mads::simulator::state::SimulatorState;
use mads::simulator::simulator::Simulator;
use mads::math::integrators::IntegratorType;

// Configure engine
let start_time = 0.0;
let max_time = 10.0;
let engine_step = 0.1;
let engine_config = EngineConfig::new(start_time, max_time, engine_step);

// Configure simulator
let integrator = IntegratorType::RK45;
let integrator_step = 0.1;
let sim_config = SimulatorConfig::new(integrator, integrator_step);

// Initial simulator state
let sim_state = SimulatorState::new(engine_config, sim_config);

```

### Scenarios

Scenarios capture the user-defined world to be simulated, including entities, their interactions, and dynamics models to be evaluated.
Using the Legion ECS library, Scenarios provide a structured way to setup Entities, Systems, the World, and other necessary data strucures
for the simulation.

A detailed description of Entity Component Systems (ECS) and Legion can be read [here](https://en.wikipedia.org/wiki/Entity_component_system)
and [here](https://docs.rs/legion/0.4.0/legion/).

#### Example

See below for an example user-defined Scenario of a single Entity, with 2D Double Integrator dynamics, and driven by an LQR controller:

```rust

use std::collections::HashMap;
use legion::*;
use nalgebra::{DMatrix, DVector};
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


```

### Running a simulation

A simulator can be constructed using the initial simulator state and scenario generated earlier.
Once built, the simulator can be ran to perform the Scenario with the given configuration:

```rust

// Construct user defined scenario
let scenario = MyScenario::new();

// Run simulation
let mut simulator = Simulator::new(sim_state, scenario);
simulator.build();
simulator.run();

```

### Logging results

To serialze simulation results to csv:

```rust
use mads::log::logger::{SimpleLogger, Logger};

let logger = SimpleLogger;
if let Err(err) = logger.to_csv(&simulator.state, "./my_scenario_results.csv") {
  println!("csv write error, {}", err);
};

```

The time-series data stored in the csv is organized by the dynamic state of each Entity,
but each Entity is stored unordered.

See [examples](https://github.com/kachark/mads/tree/main/examples) for more details on how to
setup and run simulations and scenarios.

## Libraries Used

MADS would not be possible without the great work put into these projects:

- [Legion](https://github.com/amethyst/legion): High performance Entity Component system library
- [nalgebra](https://nalgebra.org): Easy to use linear algebra and matrix library


