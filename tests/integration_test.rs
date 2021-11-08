
use nalgebra::{DVector, DMatrix};
use mads::dynamics::models::DoubleIntegrator3D;
use mads::dynamics::statespace::StateSpaceRepresentation;
use mads::controls::models::LinearQuadraticRegulator as LQR;
use mads::math::integrate::{solve_ivp, SolverOptions, IntegratorType};

#[test]
fn test_dynamics_no_ecs() {
    let model = DoubleIntegrator3D::new();

    // Define matrices for a Linear Quadratic Regulator
    let Q = DMatrix::<f32>::identity(6, 6);
    let R = DMatrix::<f32>::identity(3, 3);
    let lqr = LQR::new(
        model.dynamics().A.clone(),
        model.dynamics().B.clone(),
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

    // Evolve dynamics
    let mut trajectory: Vec<DVector<f32>> = vec![x0];
    let step = 0.1;
    let t0 = 0;
    let tf = 10;
    for t in t0..tf {

        let x_prev = trajectory[trajectory.len()-1].clone();

        println!("{:?}", &x_prev);

        // Wrap dynamics/controls in appropriately defined closure - f(t, x)
        let f = |t: f32, x: &DVector<f32>| {
            let u = -&K * x;
            model.f(t, x, Some(&u))
        };

        // Integrate dynamics
        let opts = SolverOptions{ first_step: Some(step), ..SolverOptions::default() };
        let (_t_history, traj) = match solve_ivp(f, (t as f32, (t as f32)+1f32), x_prev, IntegratorType::RK45, opts) {
            Ok(ode_result) => ode_result,
            Err(error) => panic!("solve_ivp error! {}", error)
        };

        // Store result
        for state in traj {

            trajectory.push(state);

        }

    }


}
