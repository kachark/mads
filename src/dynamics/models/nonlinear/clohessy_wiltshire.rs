
use na::DVector;
use crate::dynamics::closed_form::ClosedFormSolution;
use crate::dynamics::statespace::{Statespace, StatespaceType};
use crate::dynamics::closed_form::{NonlinearExpression_fn, NonlinearExpression};

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

pub struct ClohessyWiltshire {

    dynamics: NonlinearExpression::< NonlinearExpression_fn >,
    statespace: Statespace,

}

impl ClohessyWiltshire {

    pub fn new() -> Self {

        // explicitly convert Function Item to Function pointer
        let model = ClohessyWiltshireSolution as NonlinearExpression_fn;
        let expression = NonlinearExpression::new(model);

        let mut statespace = Statespace::new(6);
        statespace.add_state(0, StatespaceType::Position0);
        statespace.add_state(1, StatespaceType::Position1);
        statespace.add_state(2, StatespaceType::Position2);
        statespace.add_state(3, StatespaceType::Velocity0);
        statespace.add_state(4, StatespaceType::Velocity1);
        statespace.add_state(5, StatespaceType::Velocity2);

        Self { dynamics: expression, statespace }

    }

    pub fn dynamics(&self) -> &NonlinearExpression< NonlinearExpression_fn > {

        &self.dynamics

    }

    pub fn statespace(&self) -> &Statespace {

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
