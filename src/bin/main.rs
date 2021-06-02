
use nalgebra::{DMatrix, DVector};
use specs::{Builder, Component, Read, ReadStorage, WriteStorage, System, VecStorage, World, WorldExt, RunNow};

use formflight::dynamics::double_integrator::*;
use formflight::dynamics::linear_system::*;
use formflight::controls::lqr::LinearQuadraticRegulator;
use formflight::math::ivp_solver::rk45::RungeKutta45;


// Define Resources
#[derive(Default)]
struct SimStepSize(u32);

#[derive(Default)]
struct StepSize(f32);

#[derive(Component, Debug)]
#[storage(VecStorage)]
struct Position {
    x: f32,
    y: f32,
    z: f32
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
struct Velocity {
    x: f32,
    y: f32,
    z: f32
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
struct FullState(DVector<f32>);


#[derive(Component, Debug)]
#[storage(VecStorage)]
struct LinearDynamics {
    name: DoubleIntegrator2D
}


#[derive(Component, Debug)]
#[storage(VecStorage)]
struct LinearFeedbackController {
    name: LinearQuadraticRegulator
}


struct PositionSystem;

impl<'a> System<'a> for PositionSystem {
    type SystemData = ReadStorage<'a, Position>;

    fn run(&mut self, position: Self::SystemData) {
        use specs::Join;

        for position in position.join() {
            println!("Hello, {:?}", &position);
        }
    }
}


struct ContinuousDynamicsSystem;
impl<'a> System<'a> for ContinuousDynamicsSystem {
    type SystemData = (
        Read<'a, SimStepSize>,
        Read<'a, StepSize>,
        WriteStorage<'a, FullState>,
        ReadStorage<'a, LinearDynamics>,
        ReadStorage<'a, LinearFeedbackController>
    );

    fn run(&mut self, data: Self::SystemData) {
        use specs::Join;

        let (sim_step, step, mut state, dynamics, controller) = data;

        let dt = sim_step.0;
        let h = step.0;

        // For each entity that has this set of components
        for (x, dyna, ctrl) in (&mut state, &dynamics, &controller).join() {

            // business logic

            // Define initial conditions
            let x0 = x.0.clone();

            // Solve the LQR controller
            let (K, _P) = match ctrl.name.solve() {
                Ok((value1, value2)) => (value1, value2),
                Err(_) => (DMatrix::<f32>::zeros(1, 1), DMatrix::<f32>::zeros(1, 1)),
            };

            // Simulate
            let mut trajectory: Vec<DVector<f32>> = vec![x0];
            let step = h;
            let t0 = 0;
            let tf = 10;
            for t in t0..tf {

                let x_prev = trajectory[trajectory.len()-1].clone();

                // Wrap dynamics/controls in appropriately defined closure - f(t, x)
                let f = |t: f32, x: &DVector<f32>| {
                    let u = -&K * x;
                    dyna.name.dynamics.f(t, x, Some(&u))
                };

                // Integrate dynamics
                let (_t_history, traj) = RungeKutta45(f, t as f32, x_prev, (t as f32)+1f32, step, 1E-5);

                // Store result
                for state in traj {

                    trajectory.push(state);

                }

            }

            for x in trajectory {

                println!("{:?}", &x.data);

            }

        }

    }

}

fn main() {
    let mut world = World::new();
    world.register::<Position>();
    world.register::<Velocity>();
    world.register::<FullState>();
    world.register::<LinearDynamics>();
    world.register::<LinearFeedbackController>();

    world.insert(SimStepSize(1));
    world.insert(StepSize(0.1));

    world.create_entity().with(Position { x: 4.0, y: 7.0, z: 0.0 }).build();
    world.create_entity().with(Velocity { x: -4.0, y: -7.0, z: 0.0 }).build();

    let dynamics = DoubleIntegrator2D::new();
    let A = dynamics.dynamics.A.clone();
    let B = dynamics.dynamics.B.clone();
    let Q = DMatrix::<f32>::identity(4, 4);
    let R = DMatrix::<f32>::identity(2, 2);

    world.create_entity()
        .with(FullState { 0: DVector::<f32>::from_vec(vec![10., 10., 0., 0.]) })
        .with(LinearDynamics { name: dynamics })
        .with(LinearFeedbackController { name: LinearQuadraticRegulator::new(A, B, Q, R) })
        .build();

    // let mut position_sys = PositionSystem;
    // position_sys.run_now(&world);
    let mut dyn_sys = ContinuousDynamicsSystem;
    dyn_sys.run_now(&world);
    world.maintain();
}
