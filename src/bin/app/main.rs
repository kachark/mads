#[allow(non_snake_case)]

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
// https://amethyst.rs/posts/legion-ecs-v0.3

use std::collections::HashMap;

use legion::*;
use formflight::ecs::components::*; // TOOD: remove eventually
use formflight::ecs::resources::*;
use formflight::util::range_step;

extern crate plotters;

pub mod configuration;
pub mod setup;
pub mod plot;

use crate::configuration::EngineParameter;

fn main() {

    // Simulation parameters
    let start_time = EngineParameter::SimulationTime(0f32);
    let maxtime = EngineParameter::MaxSimulationTime(10f32);
    let engine_step = EngineParameter::EngineStep(1f32);

    let mut params = HashMap::new();
    params.entry("SimulationTime".to_string()).or_insert(start_time);
    params.entry("MaxSimulationTime".to_string()).or_insert(maxtime);
    params.entry("EngineStep".to_string()).or_insert(engine_step);

    // Get World, Resources, and engine time values
    let (mut world, mut resources, times) = setup::setup(&params);

    // Schedule systems for exectution
    let mut schedule = setup::setup_systems();

    // NOTE: simulation loop
    for _ in times {

        schedule.execute(&mut world, &mut resources);

        // query for 'Targetable' components
        let mut targetable_set_atomic = resources.get_mut::<TargetableSet>().unwrap();
        let mut query = <(&SimID, &FullState, &mut Targetable)>::query();
        for mut chunk in query.iter_chunks_mut(&mut world) {
            // we can access information about the archetype (shape/component layout) of the entities
            println!(
                "the entities in the chunk have {:?} components",
                chunk.archetype().layout().component_types(),
            );

            // we can iterate through a tuple of component references per entity
            for (id, state, target) in chunk {
                if target.0 == true {
                    continue
                } else {
                    targetable_set_atomic.0.entry(id.uuid).or_insert(state.clone());
                }
            }
        }

    }

    match plot::plot_trajectory() {

        Ok(()) => println!("hi"),
        Err(_) => println!("plot error")

    };

 }

