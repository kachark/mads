#[allow(non_snake_case)]

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
// https://amethyst.rs/posts/legion-ecs-v0.3

extern crate plotters;
extern crate polars;

pub mod configuration;
pub mod setup;
pub mod plot;
pub mod distributions;
// Scenario specific
pub mod tracking_scenario;
pub mod scenario_resources;
pub mod scenario_components;

use std::collections::HashMap;
use legion::*;
use nalgebra::DVector;
use polars::prelude::{DataFrame, Field, Schema, Series, DataType};
use formflight::ecs::components::*; // TOOD: remove eventually
use formflight::ecs::resources::*;
use formflight::math::integrators::IntegratorType;

use crate::scenario_components::Agent;
use crate::configuration::{EngineParameter, SimulationParameter, ScenarioParameter};

fn main() {

    // Engine parameters
    let start_time = EngineParameter::SimulationTime(0f32);
    let maxtime = EngineParameter::MaxSimulationTime(10f32);
    let engine_step = EngineParameter::EngineStep(0.1f32);

    // Simulation parameters
    let integrator = SimulationParameter::Integrator(IntegratorType::RK45);
    let integrator_step = SimulationParameter::IntegratorStep(0.1);

    // Scenario parameters
    let num_agents = ScenarioParameter::NumAgents(1);
    let num_targets = ScenarioParameter::NumTargets(1);

    let mut engine_params = HashMap::new();
    engine_params.entry("SimulationTime".to_string()).or_insert(start_time);
    engine_params.entry("MaxSimulationTime".to_string()).or_insert(maxtime);
    engine_params.entry("EngineStep".to_string()).or_insert(engine_step);

    let mut sim_params = HashMap::new();
    sim_params.entry("Integrator".to_string()).or_insert(integrator);
    sim_params.entry("IntegratorStep".to_string()).or_insert(integrator_step);

    let mut scene_params = HashMap::new();
    scene_params.entry("NumAgents".to_string()).or_insert(num_agents);
    scene_params.entry("NumTargets".to_string()).or_insert(num_targets);

    // Get World, Resources, and engine time values
    let (mut world, mut resources, times) = setup::setup(&engine_params, &sim_params);
    tracking_scenario::setup_scenario(&mut world, &mut resources, &scene_params);

    // Schedule systems for exectution
    let mut schedule = setup::setup_systems();

    // NOTE: simulation loop
    for _ in times {

        schedule.execute(&mut world, &mut resources);
        update_targetable_set(&mut world, &mut resources);
        // update_storage(&mut world, &mut test_storage);

    }

    let time_history = resources.get::<SimulationTimeHistory>().unwrap();
    let result = resources.get::<SimulationResult>().unwrap();
    for (uuid, trajectory) in result.data.iter() {
        println!("Entity: {:?}", uuid);
        for state in trajectory {
            println!("{:?}", state);
        }
    }

    match plot::plot_trajectory(&time_history, &result) {

        Ok(()) => println!("hi"),
        Err(_) => println!("plot error")

    };

 }

fn update_targetable_set(world: &mut World, resources: &mut Resources) {

    // TODO: update TargetableSet - update_targetable_set() - scenario specific
    // query for 'Targetable' components
    let mut targetable_set_atomic = resources.get_mut::<TargetableSet>().unwrap();
    let mut query = <(&SimID, &FullState, &mut Targetable)>::query();
    for mut chunk in query.iter_chunks_mut(world) {
        // we can access information about the archetype (shape/component layout) of the entities
        println!(
            "the entities in the chunk have {:?} components",
            chunk.archetype().layout().component_types(),
        );

        // we can iterate through a tuple of component references per entity
        for (id, state, targetable) in chunk {
            if targetable.0 == true {
                continue
            } else {
                targetable_set_atomic.0.entry(id.uuid).or_insert(state.clone());
            }
        }
    }

}

// NOTE: TEST for double integrator
fn get_schema() -> Schema {
    Schema::new(vec![
        Field::new("Time", DataType::Float32),
        Field::new("x", DataType::Float32),
        Field::new("y", DataType::Float32),
        Field::new("z", DataType::Float32),
        Field::new("vx", DataType::Float32),
        Field::new("vy", DataType::Float32),
        Field::new("vz", DataType::Float32)
    ])
}

// fn update_storage(world: &mut World, storage: &mut Vec<DVector<f32>>) {

//     // TODO: 
//     let mut query2 = <(&SimID, &FullState, &Agent)>::query();
//     for mut chunk in query2.iter_chunks_mut(world) {
//         // we can access information about the archetype (shape/component layout) of the entities
//         println!(
//             "the entities in the chunk have {:?} components",
//             chunk.archetype().layout().component_types(),
//         );

//         // we can iterate through a tuple of component references per entity
//         for (id, state, agent) in chunk {
//             if agent.0 == true {
//                 // store deepcopy of fullstate
//                 storage.push(state.0.clone());
//             }
//         }
//     }

// }
