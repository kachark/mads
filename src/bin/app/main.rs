#[allow(non_snake_case)]

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
// https://amethyst.rs/posts/legion-ecs-v0.3

extern crate plotters;

pub mod plot;
pub mod distributions;
pub mod scenarios;

use formflight::simulator::configuration::{EngineConfig, SimulationConfig};
use formflight::simulator::simulation::Simulation;
use formflight::simulator::state::SimulationState;
use formflight::ecs::resources::*;

// use formflight::scene::scenario::SimpleScenario;
// use crate::scenarios::tracking_scenario::TrackingScenario;
use crate::scenarios::nonlinear_scenario::NonlinearScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulationConfig::default();
    let sim_state = SimulationState::new(engine_config, sim_config);
    // let scenario = TrackingScenario::default();
    let scenario = NonlinearScenario::default();
    // let scenario = SimpleScenario::new();

    let mut simulation = Simulation::new(sim_state, scenario);
    simulation.build();
    simulation.run();

    // NOTE: post processing
    // TODO: safely unwrap resources.get()
    let time_history = simulation.state.resources.get::<SimulationTimeHistory>().unwrap();
    let result = simulation.state.resources.get::<SimulationResult>().unwrap();
    // let targetable_set_atomic = simulation.state.resources.get_mut::<TargetableSet>().unwrap();
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

    // println!("{:?}", targetable_set_atomic);

    match plot::plot_trajectory_3d(&time_history, &result) {

        Ok(()) => println!("hi"),
        Err(_) => println!("plot error")

    };

 }

