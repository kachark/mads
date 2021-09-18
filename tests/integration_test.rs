
use nalgebra::{DVector, DMatrix};
use mads::dynamics::models::linear::double_integrator::DoubleIntegrator3D;
use mads::dynamics::statespace::StateSpaceRepresentation;
use mads::controls::models::lqr::LinearQuadraticRegulator as LQR;
use mads::math::ivp_solver::euler::MidPointEuler;

#[test]
fn test_linear_dynamics() {
    let model = DoubleIntegrator3D::new();

    // Define matrices for a Linear Quadratic Regulator
    let Q = DMatrix::<f32>::identity(6, 6);
    let R = DMatrix::<f32>::identity(3, 3);
    let lqr = LQR::new(
        model.dynamics.A.clone(),
        model.dynamics.B.clone(),
        Q,
        R,
    );


    // Define initial conditions
    let x0 = DVector::from_vec(vec![10., 10., -10., -30., 12., 2.]);

    // Solve the LQR controller
    let (K, _P) = match lqr.solve() {
        Ok((value1, value2)) => (value1, value2),
        Err(_) => (DMatrix::<f32>::zeros(1, 1), DMatrix::<f32>::zeros(1, 1)),
    };

    // Simulate
    let mut trajectory: Vec<DVector<f32>> = vec![x0];
    let step = 0.1;
    let t0 = 0;
    let tf = 10;
    for t in t0..tf {

        let x_prev = trajectory[trajectory.len()-1].clone();

        // Wrap dynamics/controls in appropriately defined closure - f(t, x)
        let f = |t: f32, x: &DVector<f32>| {
            let u = -&K * x;
            model.f(t, x, Some(&u))
        };

        // Integrate dynamics
        let (_t_history, traj) = MidPointEuler(f, t as f32, x_prev, (t as f32)+1f32, step);

        // Store result
        for state in traj {

            trajectory.push(state);

        }

    }


}
