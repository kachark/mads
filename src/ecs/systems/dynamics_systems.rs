
use nalgebra::{DMatrix, DVector};
use specs::{Read, ReadStorage, Write, WriteStorage, System};

use crate::dynamics::linear_system::*;
use crate::math::ivp_solver::rk45::RungeKutta45;
use crate::ecs::resources::*;
use crate::ecs::components::*;

pub struct ContinuousDoubleIntegratorLQR3D;
impl<'a> System<'a> for ContinuousDoubleIntegratorLQR3D {
    type SystemData = (
        Read<'a, EngineStep>,
        Read<'a, IntegratorStep>,
        Write<'a, SimulationTime>,
        WriteStorage<'a, FullState>,
        ReadStorage<'a, DoubleIntegratorDynamics3D>,
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


pub struct ContinuousEulerHillLQR;
impl<'a> System<'a> for ContinuousEulerHillLQR {
    type SystemData = (
        Read<'a, EngineStep>,
        Read<'a, IntegratorStep>,
        Write<'a, SimulationTime>,
        WriteStorage<'a, FullState>,
        ReadStorage<'a, EulerHillDynamics>,
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

            // // Wrap dynamics/controls in appropriately defined closure - f(t, x)
            // let f = |t: f32, x: &DVector<f32>| {
            //     let u = -&K * x;
            //     dyna.name.dynamics.f(t, x, Some(&u))
            // };

            let f = |t: f32, x: &DVector<f32>| {
                dyna.name.dynamics.f(t, x, None)
            };

            // Integrate dynamics
            let (_t_history, traj) = RungeKutta45(f, time.0, x_prev, tf, step, rtol);

            // Update entity FullState component
            x.0 = traj[traj.len()-1].clone();

            // Store result
            for state in traj {

                trajectory.push(state);

            }

            // for x in trajectory {

            //     println!("{:?}", &x.data);

            // }

            println!("{:?}", x.0.data);

        }

        // increment time
        time.0 += dt;

    }

}


