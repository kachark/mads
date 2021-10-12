
use nalgebra::{DMatrix, DVector};
use legion::*;
use legion::storage::Component;
use crate::dynamics::statespace::StateSpaceRepresentation;
use crate::math::integrate::runge_kutta::{RK45, RKF45};
use crate::math::integrate::euler::ForwardEuler;
use crate::math::integrate::euler::MidPointEuler;
use crate::math::integrators::IntegratorType;
use crate::ecs::resources::*;
use crate::ecs::components::*;

// NOTE: to parallelize with Rayon, use par_for_each
// #[system(for_each)]
#[system(par_for_each)]
pub fn simulate_lqr_dynamics<T>(
    state: &mut FullState,
    dynamics: &DynamicsModel<T>,
    controller: &LQRController,
    #[resource] time: &SimulationTime,
    #[resource] sim_step: &EngineStep,
    #[resource] integrator: &Integrator,
    #[resource] step: &IntegratorStep
)
where
    T: Component + StateSpaceRepresentation // Need to include Component trait from Legion
{

    let dt = sim_step.0;
    let h = step.0;

    // Define initial conditions
    let x0 = state.data.clone();

    // Solve the LQR controller
    let (K, _P) = match controller.model.solve() {
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
        dynamics.model.f(t, x, Some(&u))
    };

    // Integrate dynamics
    let (_t_history, traj) = match integrator.0 {
        IntegratorType::ForwardEuler => ForwardEuler(f, time.0, x_prev, tf, step),
        IntegratorType::MidpointEuler => MidPointEuler(f, time.0, x_prev, tf, step),
        IntegratorType::RKF45 => RKF45(f, time.0, x_prev, tf, step, rtol),
        IntegratorType::RK45 => RK45(f, time.0, x_prev, tf, step, rtol)
    };

    // Update entity FullState component
    state.data = traj[traj.len()-1].clone();

    // Store result
    for state in traj {

        trajectory.push(state);

    }

    // DEBUG: show integrated dynamics
    // for x in trajectory {

    //     println!("{:?}", &x.data);

    // }

}


// NOTE: to parallelize with Rayon, use par_for_each
// #[system(for_each)]
#[system(par_for_each)]
pub fn simulate_dynamics<T>(
    state: &mut FullState,
    dynamics: &DynamicsModel<T>,
    #[resource] time: &SimulationTime,
    #[resource] sim_step: &EngineStep,
    #[resource] integrator: &Integrator,
    #[resource] step: &IntegratorStep
)
where
    T: Component + StateSpaceRepresentation // Need to include Component trait from Legion
{

    let dt = sim_step.0;
    let h = step.0;

    // Define initial conditions
    let x0 = state.data.clone();

    // Simulate
    let mut trajectory: Vec<DVector<f32>> = vec![x0];
    let step = h;
    // let t0 = 0;
    let tf = time.0 + dt;
    let rtol = 1E-3;


    let x_prev = trajectory[trajectory.len()-1].clone();

    // Wrap dynamics/controls in appropriately defined closure - f(t, x)
    let f = |t: f32, x: &DVector<f32>| {
        dynamics.model.f(t, x, None)
    };

    // Integrate dynamics
    let (_t_history, traj) = match integrator.0 {
        IntegratorType::ForwardEuler => ForwardEuler(f, time.0, x_prev, tf, step),
        IntegratorType::MidpointEuler => MidPointEuler(f, time.0, x_prev, tf, step),
        IntegratorType::RKF45 => RKF45(f, time.0, x_prev, tf, step, rtol),
        IntegratorType::RK45 => RK45(f, time.0, x_prev, tf, step, rtol)
    };

    // Update entity FullState component
    state.data = traj[traj.len()-1].clone();

    // Store result
    for state in traj {

        trajectory.push(state);

    }

    // DEBUG: show integrated dynamics
    // for x in trajectory {

    //     println!("{:?}", &x.data);

    // }

}

