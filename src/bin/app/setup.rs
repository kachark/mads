

use std::collections::HashMap;

use legion::{World, Resources, Schedule};

use formflight::dynamics::models::linear::double_integrator::*;
use formflight::util::range_step;

use formflight::ecs::resources::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::systems::simple_systems::*;

use crate::configuration::*;

/// Define World, Resources, and Components
pub fn setup(
    engine_params: &HashMap<String, EngineParameter>,
    sim_params: &HashMap<String, SimulationParameter>
) -> (World, Resources, Vec<f32>)
{

    // Declare World and Resources
    let world = World::default();
    let mut resources = Resources::default();

    // Generate Engine time values
    let times = get_times(engine_params);

    // Populate resources
    add_engine_resources(&mut resources, engine_params);
    add_simulation_resources(&mut resources, sim_params);

    (world, resources, times)

}

/// Inserts Resources to ECS derived from EngineParameters
fn add_engine_resources(resources: &mut legion::Resources, params: &HashMap<String, EngineParameter>) {

    for (_, parameter) in params.iter() {
        match parameter {
            EngineParameter::SimulationTime(value) => resources.insert(SimulationTime(*value)),
            EngineParameter::MaxSimulationTime(value) => resources.insert(MaxSimulationTime(*value)),
            EngineParameter::EngineStep(value) => resources.insert(EngineStep(*value)),
        }
    }

}


/// Inserts Resources to ECS derived from SimulationParameters
fn add_simulation_resources(resources: &mut legion::Resources, params: &HashMap<String, SimulationParameter>) {

    for (_, parameter) in params.iter() {
        match parameter {
            SimulationParameter::IntegratorStep(value) => resources.insert(IntegratorStep(*value)),
            SimulationParameter::Integrator(value) => resources.insert(Integrator(*value))
        }
    }

}

/// Generate Vector of values for the time history of the simulation from HashMap of
/// EngineParameters
fn get_times(params: &HashMap<String, EngineParameter>) -> Vec<f32> {

    let mut start_time: f32 = 0f32;
    let mut max_time: f32 = 0f32;
    let mut step: f32 = 0f32;

    for (_, parameter) in params.iter() {
        match parameter {
            EngineParameter::SimulationTime(value) => start_time = *value,
            EngineParameter::MaxSimulationTime(value) => max_time = *value,
            EngineParameter::EngineStep(value) => step = *value,
            // _ => continue // need this when there are more params
        }
    }

    range_step(start_time, max_time, step)

}


/// Define systems to be run per loop iteration and return a Schedule to execute
pub fn setup_systems() -> legion::Schedule {

    let schedule = Schedule::builder()
        .add_system(increment_time_system())
        .add_system(print_id_system())
        // .add_system(print_errorstate_system()) // TODO: make this generic over two FullStates
        .add_system(print_state_system())
        .add_system(update_position_system()) // implicitly adds 'system' to the end
        .add_system(dynamics_lqr_solver_system::<DoubleIntegrator2D>())
        .add_system(dynamics_lqr_solver_system::<DoubleIntegrator3D>())
        .build();

    schedule

}
