#[allow(non_snake_case)]

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
// https://amethyst.rs/posts/legion-ecs-v0.3

extern crate plotters;

pub mod plot;
pub mod distributions;
pub mod scenario;
// Scenario specific
pub mod tracking_scenario;
pub mod scenario_resources;
pub mod scenario_components;
pub mod scenario_configuration;

use std::collections::HashMap;
use legion::*;
use formflight::setup::{setup, setup_systems};
use formflight::ecs::components::*; // TOOD: remove eventually
use formflight::ecs::resources::*;
use formflight::configuration::{EngineConfig, SimulationConfig};
use crate::scenario_configuration::ScenarioConfig;

fn main() {

    let engine_config = EngineConfig::default();
    let sim_config = SimulationConfig::default();
    let scene_config = ScenarioConfig::default();

    // Get World, Resources, and engine time values
    let (mut world, mut resources, times) = setup(&engine_config, &sim_config);
    tracking_scenario::setup_scenario(&mut world, &mut resources, &scene_config);

    // Schedule systems for exectution
    let mut schedule = setup_systems();

    // NOTE: simulation loop
    for _ in times {

        schedule.execute(&mut world, &mut resources);
        update_targetable_set(&mut world, &mut resources);

    }

    // TODO: safely unwrap resources.get()
    let time_history = resources.get::<SimulationTimeHistory>().unwrap();
    let result = resources.get::<SimulationResult>().unwrap();
    for (uuid, trajectory) in result.data.iter() {
        println!("Entity: {:?}", uuid);
        println!("length: {:?}", trajectory.len());
        for state in trajectory {
            println!("{:?}", state);
        }
    }

    println!("{:?}", time_history.data.len());

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
