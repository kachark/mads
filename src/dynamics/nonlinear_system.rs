
use na::DVector;
use crate::dynamics::statespace::StateSpaceRepresentation;
use crate::dynamics::closed_form::ClosedFormSolution;

/// An alias for function pointer type that satisfy NonlinearSystem trait bounds
pub type NonlinearStateSpaceFnType = fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>;

/// An alias for function pointer types that satisfy NonlinearExpression trait bounds
pub type NonlinearExpressionFnType = fn(f32, &DVector<f32>) -> DVector<f32>;


// https://stackoverflow.com/questions/27831944/how-do-i-store-a-closure-in-a-struct-in-rust
// https://doc.rust-lang.org/reference/types/closure.html
// NOTE: why generic over 2 function signatures?
// - allows you to define a closure as inputs for both state_equation AND output_equation.
// - This is because a closure type is unique and therefore could never satisfy constraining
// two struct fields simultaneously

/// A nonlinear state-space model of the form: \
/// x_dot(t) = f(t, x(t), u(t)) \
/// y(t) = h(t, x(t), u(t)) \
///
/// # Example
///
/// ```rust
/// use nalgebra::DVector;
/// use mads::dynamics::nonlinear_system::*;
///
/// fn equations_of_motion(t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {
///     (x*x).clone()
/// }
///
/// fn output_equations(t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {
///     x.clone()
/// }
///
/// let dx = 4;
/// let du = 2;
///
/// let f = equations_of_motion as NonlinearStateSpaceFnType;
/// let h = output_equations as NonlinearStateSpaceFnType;
/// let model = NonlinearStateSpaceModel::new(f, h, dx, du);
/// ```
///
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NonlinearStateSpaceModel<F, H>
where
    F: Fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>,
    H: Fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>,
{

    state_equation: F,
    output_equation: H,
    dx: usize,
    du: usize

}

impl<F, H> NonlinearStateSpaceModel<F, H>
where
    F: Fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>,
    H: Fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>,
{
    pub fn new(f: F, h: H, dx: usize, du: usize) -> Self
    {

        Self { state_equation: f, output_equation: h, dx, du }

    }

}

impl<F, H> StateSpaceRepresentation for NonlinearStateSpaceModel<F, H>
where
    F: Fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>,
    H: Fn(f32, &DVector<f32>, Option<&DVector<f32>>) -> DVector<f32>,
{

    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        (self.state_equation)(t, x, u) // syntax for calling the function of a function-typed field

    }

    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {


        (self.output_equation)(t, x, u)

    }

}



/// A generic nonlinear expression f(t, x) \
///
/// x(t) = f(t, x(t)) \
///
/// # Example
///
/// ```rust
/// use nalgebra::DVector;
/// use mads::dynamics::nonlinear_system::*;
///
/// fn closed_form_solution(t: f32, x: &DVector<f32>) -> DVector<f32> {
///     x.clone()
/// }
///
/// let model = NonlinearExpression::new(closed_form_solution);
/// ```
///
pub struct NonlinearExpression<F>
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>
{

    expression: F

}

impl<F> NonlinearExpression<F>
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>
{

    pub fn new(f: F) -> Self {

        Self { expression: f }

    }

}

impl<F> ClosedFormSolution for NonlinearExpression<F>
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>
{

    fn rhs(&self, t: f32, x: &DVector<f32>) -> DVector<f32> {

        (self.expression)(t, x)

    }

}




#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::models::closed_form_solution::ClohessyWiltshireSolution;

    #[test]
    fn test_NonlinearSystem() {

        use std::f32::consts as consts;
        use crate::math::integrate::runge_kutta::RK45;

        // Pendulum model params
        // https://ctms.engin.umich.edu/CTMS/index.php?aux=Activities_Pendulum
        let g = 9.81; // gravity
        let l = 5.0; // radius of pendulum
        let k = 0.7; // coeff. of friction
        let m = 3.0; // mass

        // Pendulum equations
        // Wrap equations of motion / state equations in appropriately defined closure for _model
        let f = |_t: f32, x: &DVector<f32>, _u: Option<&DVector<f32>>| {

            let mut res = DVector::<f32>::zeros(x.len());

            res[0] = x[1];
            res[1] = -(g/l)*x[0].sin() - (k/(m*l))*x[1].sin();

            res
        };

        // Wrap output equation in appropriately defined closure for _model
        let h = |_t: f32, x: &DVector<f32>, _u: Option<&DVector<f32>>| {
            x.clone()
        };

        let _model = NonlinearStateSpaceModel::new(f, h, 2, 1);

        // initial conditions
        let x0 = DVector::<f32>::from_vec(vec![consts::PI/4.0, -1.0]); // omega, omega_dot
        let t0 = 0.0;

        // println!("{:?}", _model.f(t0, &x0, None));
        // println!("{:?}", _model.h(t0, &x0, None));

        // Integrate the dynamics

        // Wrap model in appropriately defined closure for integrator (ie. f(t,x))
        let dynamics = |t: f32, x: &DVector<f32>| {
            _model.f(t, x, None)
        };

        let tf = 10.0;
        let n = 100.0;
        let step = (tf - t0) / n;
        let rtol = 1E-3;
        let (_t, y) = RK45(dynamics, t0, x0, tf, step, rtol);

        for ele in y.iter() {
            println!("{:?}", ele.data);
        }

    }

    #[test]
    fn test_NonlinearExpression() {

        let closed_form = NonlinearExpression::new(ClohessyWiltshireSolution);

        let dt = 20.0;
        let x0 = DVector::from_vec(vec![0., 0., 0., 0.1, 0., 0.]);
        let mut traj = Vec::new();
        for i in 0..100 {

            let t = dt*i as f32;
            let test = closed_form.rhs(t, &x0);

            traj.push(test);

        }

        for ele in traj.iter() {
            println!("{:?}", ele.data);
        }



    }


}
