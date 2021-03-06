
use na::{DMatrix, DVector};
use crate::dynamics::nonlinear_system::{NonlinearStateSpaceModel, NonlinearStateSpace_fn};
use crate::dynamics::statespace::{StateSpace, StateSpaceType, StateSpaceRepresentation};

fn equations_of_motion(_t: f32, x: &DVector<f32>, _u: Option<&DVector<f32>>) -> DVector<f32> {

    // Double pendulum model params
    // https://web.mit.edu/jorloff/www/chaosTalk/double-pendulum/double-pendulum-en.html
    let g = 9.81; // gravity
    let l1 = 1.0; // length of rod 1
    let l2 = 1.0; // length of rod 2
    let m1 = 2.0; // mass at end of rod 1
    let m2 = 2.0; // mass at end of rod 2

    // Pendulum equations
    let mut res = DVector::<f32>::zeros(x.len());

    let x1 = x[0]; // theta rod 1
    let x2 = x[1]; // omega rod 1
    let x3 = x[2]; // theta rod 2
    let x4 = x[3]; // omega rod 2

    res[0] = x2;

    let num1 = -g * (2.0*m1 + m2) * x1.sin()
        - m2 * g * (x1 - (2.0*x3)).sin()
        - 2.0
            * (x1-x3).sin()
            * m2
            * (x4.powf(2.)*l2
               + x2.powf(2.)*l1*(x1-x3).cos());

    let den1 = l1*( 2.0*m1 + m2 - m2*(2.0*x1 - 2.0*x3).cos());
    res[1] = num1 / den1;

    res[2] = x4;

    let num2 = 2.0
        * (x1-x3).sin()
        * (x2.powf(2.)*l1*(m1+m2)
           + g * (m1+m2)*x1.cos()
           + x4.powf(2.)*l2*m2*(x1-x3).cos());
    let den2 = l2*( 2.0*m1 + m2 - m2*(2.0*x1 - 2.0*x3).cos());

    res[3] = num2 / den2;

    res

}

fn output_equations(_t: f32, x: &DVector<f32>, _u: Option<&DVector<f32>>) -> DVector<f32> {

    let C = DMatrix::from_row_slice(2, 4,
            &[1., 0., 0., 0.,
            0., 0., 1., 0.]);

    C*x

}

/// Double pendulum dynamics model
///
/// x = [attitude0] (rod 1)\
///     [angular_velocity0] (rod 1)\
///     [attitude1] (rod 2)\
///     [angular_velocity1] (rod 2)\
///
/// [Reference](https://web.mit.edu/jorloff/www/chaosTalk/double-pendulum/double-pendulum-en.html)
///
pub struct DoublePendulum {

    dynamics: NonlinearStateSpaceModel::< NonlinearStateSpace_fn, NonlinearStateSpace_fn >,
    statespace: StateSpace,

}

impl DoublePendulum {

    pub fn new() -> Self {

        // explicitly convert Function Item to Function pointer
        let f = equations_of_motion as NonlinearStateSpace_fn;
        let h = output_equations as NonlinearStateSpace_fn;
        let dynamics = NonlinearStateSpaceModel::new(f, h, 2, 1);

        let mut statespace = StateSpace::new(4);
        statespace.add_state(0, StateSpaceType::Attitude0);
        statespace.add_state(1, StateSpaceType::AngularVelocity0);
        statespace.add_state(2, StateSpaceType::Attitude1);
        statespace.add_state(3, StateSpaceType::AngularVelocity1);

        Self {
            dynamics,
            statespace
        }

    }

    pub fn dynamics(&self) -> &NonlinearStateSpaceModel< NonlinearStateSpace_fn, NonlinearStateSpace_fn > {

        &self.dynamics

    }

    pub fn statespace(&self) -> &StateSpace {

        &self.statespace

    }

}

impl StateSpaceRepresentation for DoublePendulum {

    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.f(t, x, u)

    }

    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.h(t, x, u)

    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::integrate::runge_kutta::RK45;
    use std::f32::consts::FRAC_PI_4;

    #[test]
    fn test_DoublePendulum() {

        let pendulum = DoublePendulum::new();

        // initial conditions
        // theta1, omega1, theta2, omega2
        let x0 = DVector::<f32>::from_vec(vec![FRAC_PI_4, 0.0, -FRAC_PI_4, 0.0]);
        let t0 = 0.0;

        // Integrate the dynamics

        // Wrap model in appropriately defined closure for integrator (ie. f(t,x))
        let dynamics = |t: f32, x: &DVector<f32>| {
            pendulum.f(t, x, None)
        };

        let tf = 10.0;
        let n = 1000.0;
        let step = (tf - t0) / n;
        let rtol = 1E-3;
        let (_t, y) = RK45(dynamics, t0, x0, tf, step, rtol);

        for ele in y.iter() {
            println!("{:?}", ele.data);
        }

    }

}
