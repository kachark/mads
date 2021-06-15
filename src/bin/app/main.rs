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
use formflight::math::integrators::IntegratorType;

extern crate plotters;

pub mod configuration;
pub mod setup;
pub mod plot;

use crate::configuration::{EngineParameter, SimulationParameter, ScenarioParameter};

fn main() {

    // Engine parameters
    let start_time = EngineParameter::SimulationTime(0f32);
    let maxtime = EngineParameter::MaxSimulationTime(10f32);
    let engine_step = EngineParameter::EngineStep(1f32);

    // Simulation parameters
    let integrator = SimulationParameter::Integrator(IntegratorType::RK45);
    let integrator_step = SimulationParameter::IntegratorStep(0.1);

    let mut engine_params = HashMap::new();
    engine_params.entry("SimulationTime".to_string()).or_insert(start_time);
    engine_params.entry("MaxSimulationTime".to_string()).or_insert(maxtime);
    engine_params.entry("EngineStep".to_string()).or_insert(engine_step);

    let mut sim_params = HashMap::new();
    sim_params.entry("Integrator".to_string()).or_insert(integrator);
    sim_params.entry("IntegratorStep".to_string()).or_insert(integrator_step);

    // Get World, Resources, and engine time values
    let (mut world, mut resources, times) = setup::setup(&engine_params, &sim_params);

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

