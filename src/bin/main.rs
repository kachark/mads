
use nalgebra::{DMatrix, DVector};
use specs::{Builder, Component, Read, ReadStorage, Write, WriteStorage, System, VecStorage, World, WorldExt, RunNow};

use formflight::dynamics::double_integrator::*;
use formflight::dynamics::linear_system::*;
use formflight::controls::lqr::LinearQuadraticRegulator;
use formflight::math::ivp_solver::rk45::RungeKutta45;


// Define Resources
#[derive(Default)]
struct Time(f32);

#[derive(Default)]
struct SimStepSize(f32);

#[derive(Default)]
struct StepSize(f32);

// Define Components
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


#[derive(Component, Debug)]
#[storage(VecStorage)]
struct Plotter {

}


// Define Systems
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


struct ContinuousDoubleIntegratorLQR;
impl<'a> System<'a> for ContinuousDoubleIntegratorLQR {
    type SystemData = (
        Read<'a, SimStepSize>,
        Read<'a, StepSize>,
        Write<'a, Time>,
        WriteStorage<'a, FullState>,
        ReadStorage<'a, LinearDynamics>,
        ReadStorage<'a, LinearFeedbackController>
    );

    fn run(&mut self, data: Self::SystemData) {
        use specs::Join;

        let (sim_step, step, mut time, mut state, dynamics, controller) = data;

        let dt = sim_step.0;
        let h = step.0;

        println!("TIME: {:?}", time.0);

        // For each entity that has this set of components
        for (n, (x, dyna, ctrl)) in (&mut state, &dynamics, &controller).join().enumerate() {

            println!("ENTITY: {:?}", n);

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
            // let t0 = 0;
            let tf = time.0 + dt;
            let rtol = 1E-3;


            let x_prev = trajectory[trajectory.len()-1].clone();

            // Wrap dynamics/controls in appropriately defined closure - f(t, x)
            let f = |t: f32, x: &DVector<f32>| {
                let u = -&K * x;
                dyna.name.dynamics.f(t, x, Some(&u))
            };

            // Integrate dynamics
            let (_t_history, traj) = RungeKutta45(f, time.0, x_prev, tf, step, rtol);

            // Update entity FullState component
            x.0 = traj[traj.len()-1].clone();

            // Store result
            for state in traj {

                trajectory.push(state);

            }

            for x in trajectory {

                println!("{:?}", &x.data);

            }

        }

        // increment time
        time.0 += dt;

    }

}


fn main() {

    let mut world = World::new();
    world.register::<FullState>();
    world.register::<LinearDynamics>();
    world.register::<LinearFeedbackController>();

    world.insert(Time(0.0));
    world.insert(SimStepSize(1f32));
    world.insert(StepSize(0.1));

    let double_integrator = DoubleIntegrator2D::new();
    let A = double_integrator.dynamics.A.clone();
    let B = double_integrator.dynamics.B.clone();
    let Q = DMatrix::<f32>::identity(4, 4);
    let R = DMatrix::<f32>::identity(2, 2);

    for _num_entities in 0..5 {

        world.create_entity()
            .with(FullState { 0: DVector::<f32>::from_vec(vec![10., 10., 0., 0.]) })
            .with(LinearDynamics { name: DoubleIntegrator2D::new() })
            .with(LinearFeedbackController { name: LinearQuadraticRegulator::new(A.clone(), B.clone(), Q.clone(), R.clone()) })
            .build();

    }

    let mut dyn_sys = ContinuousDoubleIntegratorLQR;

    for _ in  0..10 {

        dyn_sys.run_now(&world);
        world.maintain();

    }


}
