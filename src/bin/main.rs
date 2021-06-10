
use nalgebra::{DMatrix, DVector};
use specs::{Builder, World, WorldExt, RunNow};

use formflight::dynamics::euler_hill::*;
use formflight::dynamics::double_integrator::*;
use formflight::controls::lqr::LinearQuadraticRegulator;

use formflight::ecs::resources::*;
use formflight::ecs::components::*;
use formflight::ecs::systems::dynamics_systems::*;

fn main() {

    let mut world = World::new();
    world.register::<FullState>();
    world.register::<DoubleIntegratorDynamics3D>();
    world.register::<EulerHillDynamics>();
    world.register::<LinearFeedbackController>();

    let max_time = MaxSimulationTime(100f32);
    world.insert(MaxSimulationTime(max_time.0));
    world.insert(SimulationTime(0.0));
    world.insert(EngineStep(1f32));
    world.insert(IntegratorStep(0.1));

    // TODO: Hardcoded
    let double_integrator = DoubleIntegrator3D::new();
    let A = double_integrator.dynamics.A.clone();
    let B = double_integrator.dynamics.B.clone();
    let Q = DMatrix::<f32>::identity(6, 6);
    let R = DMatrix::<f32>::identity(3, 3);


//     // TODO: Hardcoded
//     let model = EulerHill3D::new();
//     let A = model.dynamics.A.clone();
//     let B = model.dynamics.B.clone();
//     let Q = DMatrix::<f32>::identity(6, 6);
//     let R = DMatrix::<f32>::identity(3, 3);

    for _num_entities in 0..2 {

        // TODO: need a better way to setup the entities - random pose/states (formations)
        // TODO: better component naming
        world.create_entity()
            .with(FullState { 0: DVector::<f32>::from_vec(vec![100.0, 100.0, 100., 0., 0., 0.]) })
            .with(DoubleIntegratorDynamics3D { name: DoubleIntegrator3D::new() })
            .with(LinearFeedbackController { name: LinearQuadraticRegulator::new(A.clone(), B.clone(), Q.clone(), R.clone()) })
            .build();

    }

    let mut dyn_sys = ContinuousDoubleIntegratorLQR3D;
    // let mut dyn_sys = ContinuousEulerHillLQR;

    for _ in  0..(max_time.0 as i32) {

        dyn_sys.run_now(&world);
        world.maintain();

    }


}
