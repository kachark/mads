#![allow(non_snake_case)]

#[macro_use]
extern crate approx;

extern crate nalgebra as na;
extern crate lapack;
extern crate lapack_src;
extern crate serde_json;

pub mod dynamics;
pub mod math;
pub mod util;
pub mod physics;
pub mod scene;
pub mod ecs;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn test_dynamics() {

        use super::na::{DMatrix, DVector};
        use super::dynamics::linear_system::linear_dynamics::*;
        use super::dynamics::controls::lqr::LinearQuadraticRegulator as LQR;

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

        let Q = DMatrix::<f32>::identity(2, 2) * 1E-1;
        let R = DMatrix::from_vec(1,1, vec![1.]);

        // LinearSystem owns the dynamics matrices
        let double_integrator = LinearSystem::new(A, B, C, D);
        let lqr = LQR::new(double_integrator.A.clone(), double_integrator.B.clone(), Q, R);

        let mut x = DVector::from_vec(vec![10.,10.]);
        let (K, _P) = match lqr.solve(){
            Ok( (value1, value2) ) => (value1, value2),
            Err(_) => (DMatrix::<f32>::zeros(1,1), DMatrix::<f32>::zeros(1,1))
        };

        for _ in 0..10 {
            let u = -&K * &x;
            x = double_integrator.solve(&x, Some(&u));

            println!("{:?}", &x);
        }

    }

}
