
use na::DVector;
use crate::math::integrate::euler::{ForwardEuler, MidPointEuler};
use crate::math::integrate::runge_kutta::{RK45, RKF45};

pub mod euler;
pub mod runge_kutta;

use thiserror::Error;

#[derive(Error, Debug)]
pub enum IntegrateError {
    #[error("Improper argument: {0}")]
    ArgError(String),

    #[error("Max iterations, {0}, reached")]
    MaxIterationsError(usize),

    #[error("Did not converge")]
    NoCoverganceError,
}

/// IVP Integrators
#[derive(Copy, Clone, PartialEq)]
pub enum IntegratorType {
    ForwardEuler,
    MidpointEuler,
    RKF45,
    RK45,
}

impl Default for IntegratorType {

    fn default() -> Self { IntegratorType::RK45 }

}

/// Defines options to be passed to the chosen IVP solver
pub struct SolverOptions {
    pub first_step: Option<f32>,
    pub rtol: f32,
    pub atol: f32,
}

impl Default for SolverOptions {
    fn default() -> Self {
        Self { first_step: None, rtol: 1E-3, atol: 1E-6 }
    }
}

/// Integrates a system of ordinary differential equations given an initial value
pub fn solve_ivp<F>(fun: F, t_span: (f32, f32), y0: DVector<f32>, method: IntegratorType, options: SolverOptions)
    -> Result< (Vec<f32>, Vec<DVector<f32>>), IntegrateError >
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>,
{

    if t_span.0 < 0.0 || t_span.0 >= t_span.1 || t_span.1 <= 0.0 {
        return Err(IntegrateError::ArgError("t_span".to_string()));
    }

    let (t0, tf) = t_span;
    let step = match options.first_step {
        Some(value) => value,
        None => (tf - t0)/1000.0
    };
    let rtol = options.rtol;
    let _atol = options.atol;

    // TODO: propogate errors from the individual functions
    let (t_steps, trajectory) = match method {

        IntegratorType::ForwardEuler => ForwardEuler(fun, t0, y0, tf, step),
        IntegratorType::MidpointEuler => MidPointEuler(fun, t0, y0, tf, step),
        IntegratorType::RKF45 => RKF45(fun, t0, y0, tf, step, rtol),
        IntegratorType::RK45 => RK45(fun, t0, y0, tf, step, rtol)

    };

    Ok( (t_steps, trajectory) )

}

#[cfg(test)]
mod tests {

    use crate::dynamics::models::linear::inverted_pendulum::InvertedPendulum;
    use crate::dynamics::statespace::StateSpaceRepresentation;
    use super::{SolverOptions, IntegratorType};
    use na::DVector;

    #[test]
    fn test_solve_ivp() {

        // define a dynamics model
        let model = InvertedPendulum::new();

        // explicitly capture the model ODE within a closure of the form f(t, x)
        let f = |t: f32, x: &DVector<f32>| model.dynamics().f(t, x, None);

        // integrate and handle errors
        let t_span = (0.0, 1.0);
        let x0 = DVector::from_vec(vec![10., 10., 10., 10.]);
        let opts = SolverOptions{ first_step: Some(0.1), ..SolverOptions::default() };
        let (_times, trajectory) = match super::solve_ivp(f, t_span, x0, IntegratorType::RK45, opts) {
            Ok(ode_result) => (ode_result.0, ode_result.1),
            Err(_) => panic!("solve_ivp failed"),
        };

        println!("{:?}", trajectory);

    }

}
