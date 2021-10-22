
use nalgebra::{DMatrix, DVector};
use legion::*;
use legion::storage::Component;
use thiserror::Error;
use crate::dynamics::statespace::StateSpaceRepresentation;
use crate::dynamics::closed_form::ClosedFormRepresentation;
use crate::math::integrate::{solve_ivp, SolverOptions, IntegrateError};
use crate::ecs::resources::*;
use crate::ecs::components::*;

// NOTE: to parallelize with Rayon, use par_for_each
// #[system(for_each)]
#[system(par_for_each)]
pub fn integrate_lqr_dynamics<T>(
    state: &mut FullState,
    dynamics: &DynamicsModel<T>,
    controller: &LQRController,
    #[resource] time: &SimulationTime,
    #[resource] sim_step: &EngineStep,
    #[resource] integrator: &Integrator,
    #[resource] step: &IntegratorStep
) -> Result<(), IntegrateError>
where
    T: Component + StateSpaceRepresentation // Need to include Component trait from Legion
{

    // Define initial conditions
    let x0 = state.data.clone();
    let mut trajectory: Vec<DVector<f32>> = vec![x0.clone()];

    // Solve the LQR controller
    let (K, _P) = match controller.model.solve() {
        Ok((value1, value2)) => (value1, value2),
        Err(_) => (DMatrix::<f32>::zeros(1, 1), DMatrix::<f32>::zeros(1, 1)),
    };

    // Parameters
    let dt = sim_step.0;
    let step = step.0;
    let t0 = time.0;
    let tf = time.0 + dt;
    let t_span = (t0, tf);
    let rtol = 1E-3;

    // Wrap dynamics/controls in appropriately defined closure - f(t, x)
    let f = |t: f32, x: &DVector<f32>| {
        let u = -&K * x;
        dynamics.model.f(t, x, Some(&u))
    };

    // Integrate dynamics
    let opts = SolverOptions{ first_step: Some(step), rtol, ..SolverOptions::default() };
    let (_times, traj) = solve_ivp(f, t_span, x0, integrator.0, opts)?;

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

    Ok(())

}


// NOTE: to parallelize with Rayon, use par_for_each
// #[system(for_each)]
#[system(par_for_each)]
pub fn integrate_dynamics<T>(
    state: &mut FullState,
    dynamics: &DynamicsModel<T>,
    #[resource] time: &SimulationTime,
    #[resource] sim_step: &EngineStep,
    #[resource] integrator: &Integrator,
    #[resource] step: &IntegratorStep
) -> Result<(), IntegrateError>
where
    T: Component + StateSpaceRepresentation // Need to include Component trait from Legion
{

    // Define initial conditions
    let x0 = state.data.clone();
    let mut trajectory: Vec<DVector<f32>> = vec![x0.clone()];

    // Parameters
    let dt = sim_step.0;
    let step = step.0;
    let t0 = time.0;
    let tf = time.0 + dt;
    let t_span = (t0, tf);
    let rtol = 1E-3;

    // Wrap dynamics/controls in appropriately defined closure - f(t, x)
    let f = |t: f32, x: &DVector<f32>| {
        dynamics.model.f(t, x, None)
    };

    // Integrate dynamics
    let opts = SolverOptions{ first_step: Some(step), rtol, ..SolverOptions::default() };
    let (_times, traj) = solve_ivp(f, t_span, x0, integrator.0, opts)?;

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

    Ok(())

}


// NOTE: to parallelize with Rayon, use par_for_each
// #[system(for_each)]
#[system(par_for_each)]
pub fn evaluate_closed_form<T>(
    state: &mut FullState,
    dynamics: &ClosedForm<T>,
    #[resource] time: &SimulationTime,
    #[resource] sim_step: &EngineStep,
    #[resource] step: &IntegratorStep
)
where
    T: Component + ClosedFormRepresentation
{

    let dt = sim_step.0;
    let tf = time.0 + dt;
    let x_prev = state.data.clone();

    // Solve for state at tf
    let new_state = dynamics.model.rhs(tf, &x_prev);

    // Update entity FullState component
    state.data = new_state.clone();

}

