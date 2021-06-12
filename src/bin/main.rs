#[allow(non_snake_case)]

use nalgebra::{DMatrix, DVector};
use legion::*;

use formflight::dynamics::models::linear::double_integrator::*;
use formflight::dynamics::models::linear::inverted_pendulum::*;
use formflight::controls::lqr::LinearQuadraticRegulator;

// // NOTE: why use an ECS in the first place?
// // - structure of arrays -> more efficient memory model for large number of objects
// // - parallelism + performance
// // - composition over inheritance -> "data-driven"
use formflight::ecs::resources::*;
use formflight::ecs::components::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::systems::simple_systems::*;

 fn main() {

     let mut world = World::default();

     let mut resources = Resources::default();

     let max_time = MaxSimulationTime(100f32);
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

     // can actually put this into it's own system - setup system
     let entities: &[Entity] = world.extend(vec![
         (Position { x: 0.0, y: 0.0, z: 0.0 }, Velocity { x: 0.0, y: 0.0, z: 0.0 }),
         (Position { x: 1.0, y: 1.0, z: 1.0 }, Velocity { x: 0.0, y: 0.0, z: 0.0 }),
         (Position { x: 2.0, y: 2.0, z: 2.0 }, Velocity { x: 0.0, y: 0.0, z: 0.0 }),
     ]);

     world.extend(vec![
         (FullState { 0: DVector::<f32>::from_vec(vec![0.0, 5.0, 2., 3.]) },
            LinearizedInvertedPendulumDynamics { name: InvertedPendulum::new() },
            LinearFeedbackController { name: LinearQuadraticRegulator::new(A2.clone(), B2.clone(), Q2.clone(), R2.clone()) }); 5
     ]);

     world.extend(vec![
         (FullState { 0: DVector::<f32>::from_vec(vec![0.0, 5.0, 10.0, 11.0, 2., 3.]) },
            DoubleIntegratorDynamics3D { name: DoubleIntegrator3D::new() },
            LinearFeedbackController { name: LinearQuadraticRegulator::new(A1.clone(), B1.clone(), Q1.clone(), R1.clone()) }); 10
     ]);

     let mut schedule = Schedule::builder()
         .add_system(update_position_system()) // implicitly adds 'system' to the end
         .add_system(LinearizedInvertedPendulumLQRSystem_system()) // implicitly adds 'system' to the end
         .add_system(DoubleIntegrator3DLQRSystem_system()) // implicitly adds 'system' to the end
         .build();


     for _ in  0..(max_time.0 as i32) {

        schedule.execute(&mut world, &mut resources);

     }

 }

