
use na::DVector;
use crate::dynamics::closed_form::ClosedFormSolution;
use crate::dynamics::statespace::{StateSpace, StateSpaceType};
use crate::dynamics::nonlinear_system::{NonlinearExpressionFnType, NonlinearExpression};

pub fn ClohessyWiltshireSolution(t: f32, x: &DVector<f32>) -> DVector<f32> {

    // Initial conditions
    let x0 = x[0];
    let y0 = x[1];
    let z0 = x[2];
    let xdot0 = x[3];
    let ydot0 = x[4];
    let zdot0 = x[5];

    // TODO: needs to be user-defined
    let n: f32 = 0.00113; // LEO orbit - omega_dot
    // let R: f32 = 405.0 + 6870.0;
    // let mu = 398600.5;
    // let n = (mu/(R.powf(3.0))).sqrt();
    let tau = n*t;

    let cos_nt = tau.cos();
    let sin_nt = tau.sin();

    let xt = (4. - 3.*cos_nt)*x0 + (1./n)*sin_nt*xdot0 + (2./n)*(1.-cos_nt)*ydot0;

    let yt = 6.*(sin_nt - tau)*x0 + y0 - (2./n)*(1.-cos_nt)*xdot0 + (1./n)*(4.*sin_nt - 3.*tau)*ydot0;

    let zt = cos_nt*z0 + (1./n)*sin_nt*zdot0;

    let xtdot = 3.*n*sin_nt*x0 + cos_nt*xdot0 + 2.*sin_nt*ydot0;

    let ytdot = -6.*n*(1.-cos_nt)*x0 - 2.*sin_nt*xdot0 + (4.*cos_nt - 3.)*ydot0;

    let ztdot = -n*sin_nt*z0 + cos_nt*zdot0;

    DVector::<f32>::from_vec(vec![xt, yt, zt, xtdot, ytdot, ztdot])

}


/// Closed-form solution of the Clohessy-Wiltshire equations of motion
///
/// \ddot{x}(t) = 3n^2x(t) + 2n\dot{y}(t) \
/// \ddot{y}(t) = -2n\dot{x}(t) \
/// \ddot{z}(t) = -n^2z(t) \
///
/// These equations describe the relative motion of a satellite with respect to a target object
/// which is in a circular orbit about a central body, represented as a point mass.
///
/// The x-axis is along the radius vector of the target object, the z-axis is along the angular
/// momentum vector of the target object, and the y-axis is orthogonal to both. The central body is
/// toward the negative x direction and the y-axis points along the velocity vector of the target
/// object.
///
/// x = [position0]\
///     [position1]\
///     [position2]\
///     [velocity0]\
///     [velocity1]\
///     [velocity2]\
///
/// [Reference](http://www.ae.utexas.edu/courses/ase366k/cw_equations.pdf)
///
pub struct ClohessyWiltshire {

    dynamics: NonlinearExpression::< NonlinearExpressionFnType >,
    statespace: StateSpace,

}

impl ClohessyWiltshire {

    pub fn new() -> Self {

        // explicitly convert Function Item to Function pointer
        let model = ClohessyWiltshireSolution as NonlinearExpressionFnType;
        let expression = NonlinearExpression::new(model);

        let mut statespace = StateSpace::new(6);
        statespace.add_state(0, StateSpaceType::Position0);
        statespace.add_state(1, StateSpaceType::Position1);
        statespace.add_state(2, StateSpaceType::Position2);
        statespace.add_state(3, StateSpaceType::Velocity0);
        statespace.add_state(4, StateSpaceType::Velocity1);
        statespace.add_state(5, StateSpaceType::Velocity2);

        Self { dynamics: expression, statespace }

    }

    pub fn dynamics(&self) -> &NonlinearExpression< NonlinearExpressionFnType > {

        &self.dynamics

    }

    pub fn statespace(&self) -> &StateSpace {

        &self.statespace

    }

}

impl ClosedFormSolution for ClohessyWiltshire {

    fn rhs(&self, t: f32, x: &DVector<f32>) -> DVector<f32> {

        self.dynamics.rhs(t, x)

    }

}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ClohessyWiltshire() {

        let dynamics = ClohessyWiltshire::new();

        let x = DVector::from_vec(vec![0., 0., 0., 0.1, 0., 0.]);

        let t0 = 0.0;
        let dt = 20.0;
        let mut t = t0;
        let mut traj = Vec::new();
        for i in 0..100 {

            let test = dynamics.rhs(t, &x);

            traj.push(test);
            t = dt*i as f32;

        }

        for ele in traj.iter() {
            println!("{:?}", ele.data);
        }

    }
}
