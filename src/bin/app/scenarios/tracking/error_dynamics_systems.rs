
// How Legion System macros work
// https://docs.rs/legion/0.4.0/legion/attr.system.html

use nalgebra::{DVector, DMatrix};
use legion::*;
use legion::storage::Component;

use formflight::dynamics::statespace::StateSpaceRepresentation;
use formflight::math::ivp_solver::rk45::RungeKutta45;
use formflight::math::ivp_solver::euler::ForwardEuler;
use formflight::math::ivp_solver::euler::MidPointEuler;
use formflight::math::integrators::IntegratorType;
use formflight::ecs::resources::*;
use formflight::ecs::components::*;

use crate::scenarios::tracking::resources::Assignment;
use crate::scenarios::tracking::components::Agent;

// TODO:
// 2 options: system or system(for_each)
// the difference between the two is that system we have to write the query ourselves
// and system(for_each) is a system defined as a query (args into the funciton handle)
//  - with system you write the query and loop over what it returns
//  - with system(for_each) the query is literally the system so you are "inside" that for loop
//  already
// using system would require a look up for the assigned target state for every agent state
// it would probably make more sense to compute the error state AT the assignment step rather than
// perform a look up per agent (and potentially lose performance/parallization)

// NOTE: to parallelize with Rayon, use par_for_each
// #[system(for_each)]
#[system(par_for_each)]
pub fn error_dynamics_lqr_solver<T>(
    agent: &Agent, // NOTE: test only evolving agents and NOT targets
    id: &SimID,
    state: &mut FullState,
    dynamics: &DynamicsModel<T>,
    controller: &LQRController,
    #[resource] time: &SimulationTime,
    #[resource] sim_step: &EngineStep,
    #[resource] integrator: &Integrator,
    #[resource] step: &IntegratorStep,
    #[resource] assignment: &Assignment
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

    // TODO: review this
    // Compute error state for the controller
    let target_state = match assignment.map.get(&id.uuid) {

        Some(item) => {
            let stored = item.clone();
            match stored {

                Some(target_vector) => target_vector.clone(),
                None => DVector::<f32>::zeros(x_prev.len()),

            }
        },
        None => DVector::<f32>::zeros(x_prev.len())

    };

    let error_state = &x_prev - &target_state;

    // Wrap dynamics/controls in appropriately defined closure - f(t, x)
    let f = |t: f32, x: &DVector<f32>| {
        let u = -&K * &error_state;
        dynamics.model.f(t, x, Some(&u))
    };

    // Integrate dynamics
    let (_t_history, traj) = match integrator.0 {
        IntegratorType::ForwardEuler => ForwardEuler(f, time.0, x_prev, tf, step),
        IntegratorType::MidpointEuler => MidPointEuler(f, time.0, x_prev, tf, step),
        IntegratorType::RK45 => RungeKutta45(f, time.0, x_prev, tf, step, rtol)
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
