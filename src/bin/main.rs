#[allow(non_snake_case)]

use std::collections::HashMap;

use nalgebra::{DMatrix, DVector};
use legion::*;
use uuid::Uuid;

use formflight::dynamics::models::linear::double_integrator::*;
use formflight::dynamics::models::linear::inverted_pendulum::*;
use formflight::controls::lqr::LinearQuadraticRegulator;

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
// https://amethyst.rs/posts/legion-ecs-v0.3
use formflight::ecs::resources::*;
use formflight::ecs::components::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::systems::simple_systems::*;

 fn main() {

     let mut world = World::default();

     let mut resources = Resources::default();

     let max_time = MaxSimulationTime(10f32);
     let num_agents = NumAgents(1);
     let mut targetable_set = TargetableSet(HashMap::new());
     resources.insert(MaxSimulationTime(max_time.0));
     resources.insert(SimulationTime(0.0));
     resources.insert(EngineStep(1f32));
     resources.insert(IntegratorStep(0.1));

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

     for i in 0..num_agents.0 {
        let id = Uuid::new_v4();
        let name = "test_name".to_string();
        world.extend(vec![
            (FullState { 0: DVector::<f32>::from_vec(vec![0.0, 5.0, 10.0, 11.0, 2., 3.]) },
                DoubleIntegratorDynamics3D { name: DoubleIntegrator3D::new() },
                LinearFeedbackController { name: LinearQuadraticRegulator::new(A1.clone(), B1.clone(), Q1.clone(), R1.clone()) },
                SimID { uuid: id, name: name },
                Agent { 0: true })
        ]);
     }

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

     let mut schedule = Schedule::builder()
         .add_system(print_id_system())
         .add_system(print_errorstate_system())
         .add_system(update_position_system()) // implicitly adds 'system' to the end
         .add_system(LinearizedInvertedPendulumLQRSystem_system()) // implicitly adds 'system' to the end
         .add_system(DoubleIntegrator3DLQRSystem_system()) // implicitly adds 'system' to the end
         .build();

     // NOTE: to have an entity track another:
     // - update FullState component (by augmenting or difference) to be the difference
     // - track ErrorState or AugmentedState as new Components
     // - (preferred) somehow access other entitites FullState component's within a system and compute
     //     the difference for the operation, then record the new full state

     for _ in  0..(max_time.0 as i32) {

        schedule.execute(&mut world, &mut resources);
        //TODO: how to update time? if it's a resource?

        // query for targetable components
        let mut query = <(&SimID, &FullState, &mut Targetable)>::query();
        for mut chunk in query.iter_chunks_mut(&mut world) {
            // we can access information about the archetype (shape/component layout) of the entities
            println!(
                "the entities in the chunk have {:?} components",
                chunk.archetype().layout().component_types(),
            );

            // we can iterate through a tuple of component references
            for (id, state, target) in chunk {
                // position is a `&Position`
                // orientation is a `&mut Orientation`
                // they are both attached to the same entity
                if target.0 == true {
                    continue
                } else {
                    targetable_set.0.entry(id.uuid).or_insert(state.clone());
                }
            }
        }

     }

 }

