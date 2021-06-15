

use std::collections::HashMap;

use nalgebra::{DMatrix, DVector};
use legion::*;
use uuid::Uuid;

use formflight::dynamics::models::linear::double_integrator::*;
use formflight::dynamics::models::linear::inverted_pendulum::*;
use formflight::controls::models::lqr::LinearQuadraticRegulator;
use formflight::util::range_step;

use formflight::ecs::resources::*;
use formflight::ecs::components::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::systems::simple_systems::*;

use crate::configuration::*;

/// Define World, Resources, and Components
pub fn setup(params: &HashMap<String, EngineParameter>) -> (legion::World, legion::Resources, Vec<f32>) {

    // Declare World and Resources
    let mut world = World::default();
    let mut resources = Resources::default();

    // Generate Engine time values
    let times = get_times(params);

    // TODO: setup_engine_resources(params: HashMap-EngineParams)
    // setup_simulation_resources(params: HashMap-SimulationParams)
    // setup_scenario_resources(params: HashMap-ScenarioParams)
    // Populate resources
    //
    // TODO: scenario and simulation parameters
    let num_agents = NumAgents(1);
    let mut targetable_set = TargetableSet(HashMap::new());
    resources.insert(IntegratorStep(0.1));

    for (_, parameter) in params.iter() {
        match parameter {
            EngineParameter::SimulationTime(value) => resources.insert(SimulationTime(*value)),
            EngineParameter::MaxSimulationTime(value) => resources.insert(MaxSimulationTime(*value)),
            EngineParameter::EngineStep(value) => resources.insert(EngineStep(*value)),
        }
    }

    // TODO: setup_agents()
    // NOTE: define components for each entity
    // TODO: Hardcoded
    let double_integrator = DoubleIntegrator3D::new();
    let A1 = double_integrator.dynamics.A.clone();
    let B1 = double_integrator.dynamics.B.clone();
    let Q1 = DMatrix::<f32>::identity(6, 6);
    let R1 = DMatrix::<f32>::identity(3, 3);

    // TODO: Hardcoded
    let inverted_pendulum = InvertedPendulum::new();
    let A2 = inverted_pendulum.dynamics.A.clone();
    let B2 = inverted_pendulum.dynamics.B.clone();
    let Q2 = DMatrix::<f32>::identity(4, 4);
    let R2 = DMatrix::<f32>::identity(1, 1);

    // TODO: Hardcoded
    let di_2d = DoubleIntegrator2D::new();
    let A3 = di_2d.dynamics.A.clone();
    let B3 = di_2d.dynamics.B.clone();
    let Q3 = DMatrix::<f32>::identity(4, 4);
    let R3 = DMatrix::<f32>::identity(2, 2);

    // // can actually put this into it's own system - setup system
    // let entities: &[Entity] = world.extend(vec![
    //     (Position { x: 0.0, y: 0.0, z: 0.0 }, Velocity { x: 0.0, y: 0.0, z: 0.0 }),
    //     (Position { x: 1.0, y: 1.0, z: 1.0 }, Velocity { x: 0.0, y: 0.0, z: 0.0 }),
    //     (Position { x: 2.0, y: 2.0, z: 2.0 }, Velocity { x: 0.0, y: 0.0, z: 0.0 }),
    // ]);

    // world.extend(vec![
    //     (FullState { 0: DVector::<f32>::from_vec(vec![0.0, 5.0, 2., 3.]) },
    //        LinearizedInvertedPendulumDynamics { name: InvertedPendulum::new() },
    //        LinearFeedbackController { name: LinearQuadraticRegulator::new(A2.clone(), B2.clone(), Q2.clone(), R2.clone()) }); 5
    // ]);

    // NOTE: we can easily do heterogeneous swarms!!!
    for i in 0..num_agents.0 {
       let id = Uuid::new_v4();
       let name = "test_name".to_string();
       world.extend(vec![
           (FullState { 0: DVector::<f32>::from_vec(vec![0.0, 5.0, 10.0, 11.0, 2., 3.]) },
               DynamicsModel { model: DoubleIntegrator3D::new() },
               LQRController { model: LinearQuadraticRegulator::new(A1.clone(), B1.clone(), Q1.clone(), R1.clone()) },
               SimID { uuid: id, name: name },
               Agent { 0: true })
       ]);

       // NOTE: generic dynamics component so we don't have to make many dynamics components per
       // model
       let id2 = Uuid::new_v4();
       world.extend(vec![
           (FullState { 0: DVector::<f32>::from_vec(vec![0.0, 0.0, 1.0, 1.0]) },
               DynamicsModel { model: DoubleIntegrator2D::new() },
               LQRController { model: LinearQuadraticRegulator::new(A3.clone(), B3.clone(), Q3.clone(), R3.clone()) },
               SimID { uuid: id2, name: "generic_component".to_string() },
               Agent { 0: true })
       ]);

       let id3 = Uuid::new_v4();
       world.extend(vec![
           (FullState { 0: DVector::<f32>::from_vec(vec![5.0, 5.0, 5.0, 5.0]) },
               DynamicsModel { model: DoubleIntegrator2D::new() },
               LQRController { model: LinearQuadraticRegulator::new(A3.clone(), B3.clone(), Q3.clone(), R3.clone()) },
               SimID { uuid: id3, name: "generic_component_2".to_string() },
               Agent { 0: true })
       ]);

    }

    // TODO: setup_targets()
    let num_targets = 1;
    for i in 0..num_targets {
       let id = Uuid::new_v4();
       let name = "target_1".to_string();
       let state = FullState { 0: DVector::<f32>::from_vec(vec![0.0, 5.0, 10.0, 11.0, 2., 3.]) };
       targetable_set.0.entry(id).or_insert(state.clone());
       world.extend(vec![
           (state,
               SimID { uuid: id, name: name },
               Targetable { 0: true })
       ]);
    }

    // NOTE: have to insert these resources afterwards since they're used in setup
    resources.insert(num_agents);
    resources.insert(TargetableSet(targetable_set.0.clone()));

    // NOTE: to have an entity track another:
    // - update FullState component (by augmenting or difference) to be the difference
    // - track ErrorState or AugmentedState as new Components
    // - (preferred) somehow access other entitites FullState component's within a system and compute
    //     the difference for the operation, then record the new full state

    (world, resources, times)

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

    let mut schedule = Schedule::builder()
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
