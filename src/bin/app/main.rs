#[allow(non_snake_case)]

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
// https://amethyst.rs/posts/legion-ecs-v0.3

extern crate plotters;

pub mod plot;
pub mod distributions;
// Scenario specific
pub mod scenario_resources;
pub mod scenario_components;
pub mod scenario_configuration;

use formflight::simulation::Simulation;
use formflight::state::SimulationState;
use formflight::ecs::resources::*;
use formflight::configuration::{EngineConfig, SimulationConfig};
// use formflight::scenario::SimpleScenario;
use crate::scenario_configuration::TrackingScenario; // TODO: put this in its own Scenarios directory

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulationConfig::default();
    let sim_state = SimulationState::new(engine_config, sim_config);
    let scenario = TrackingScenario::default();
    // let scenario = SimpleScenario::new();

    let mut simulation = Simulation::new(sim_state, scenario);
    simulation.build();
    simulation.run();

    // NOTE: post processing
    // TODO: safely unwrap resources.get()
    let time_history = simulation.state.resources.get::<SimulationTimeHistory>().unwrap();
    let result = simulation.state.resources.get::<SimulationResult>().unwrap();
    for (uuid, trajectory) in result.data.iter() {
        println!("Entity: {:?}", uuid);
        println!("length: {:?}", trajectory.len());
        for state in trajectory {
            println!("{:?}", state);
        }
    }

    println!("{:?}", time_history.data.len());
    for t in time_history.data.iter() {

        println!("{:?}", t);

    }

    match plot::plot_trajectory(&time_history, &result) {

        Ok(()) => println!("hi"),
        Err(_) => println!("plot error")

    };

 }

