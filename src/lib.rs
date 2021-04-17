
extern crate nalgebra as na;

pub mod dynamics;
pub mod controls;
pub mod linalg;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn test_dynamics() {

        use super::na::{DMatrix, DVector};
        use super::dynamics::linear_dynamics::*;
        use super::controls::LinearQuadraticRegulator as LQR;

        // generate row-major matrices
        let A = DMatrix::from_row_slice(2,2, &[
                                0., 1.,
                                0., 0.
        ]);

        let B = DMatrix::from_row_slice(2,1, &[
                                0.,
                                1.
        ]);

        let C = A.clone();
        let D = B.clone();


        let Q = DMatrix::<f32>::identity(2, 2) * 1E-2;
        let R = DMatrix::from_vec(1,1, vec![1.]);

        // LinearSystem owns the dynamics model
        let mut double_integrator = LinearSystem::new(A, B, C, D);
        // LQR borrows the dynamics model
        let mut lqr = LQR::new(&double_integrator.A, &double_integrator.B, &Q, &R);

        let mut x = DVector::from_vec(vec![10.,10.]);
        let (K, P) = lqr.solve();
        for _ in 0..10 {
            let u = -&K * &x;
            x = double_integrator.solve(&x, Some(&u));

            println!("{:?}", &x);
        }

    }

}
