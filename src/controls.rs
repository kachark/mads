
use na::DMatrix;

use crate::linalg;

/// Continuous Infinite-Horizon Linear Quadratic Regulator
pub struct LinearQuadraticRegulator<'a> {

    A: &'a DMatrix<f32>,
    B: &'a DMatrix<f32>,
    R: &'a DMatrix<f32>,
    Q: &'a DMatrix<f32>

}

impl<'a> LinearQuadraticRegulator<'a> {

    pub fn new(A: &'a DMatrix<f32>, B: &'a DMatrix<f32>, Q: &'a DMatrix<f32>, R: &'a DMatrix<f32>) -> Self {

        // TODO assert correct sizes
        assert_eq!(A.shape().0, A.shape().1);
        assert_eq!(A.shape().0, Q.shape().0);
        assert_eq!(A.shape().1, Q.shape().1);
        assert_eq!(B.shape().0, A.shape().0);
        assert_eq!(B.shape().1, R.shape().0);

        Self {
            A,
            B,
            Q,
            R
        }

    }

    /// Returns LQR gain and solution to the Continuous Algebraic Riccati Equation
    pub fn solve(&self) -> (DMatrix<f32>, DMatrix<f32>) {

        let K: DMatrix<f32>;
        let P: DMatrix<f32>;

        P = linalg::solve_continuous_riccati_iterative(&self.A, &self.B, &self.Q, &self.R,
                                0.001, 100000, 1E-5);

        let mut Rinv = self.R.clone_owned();
        Rinv.try_inverse_mut();

        K = Rinv*self.B.transpose()*&P;

        (K, P)

    }

}


#[cfg(test)]

#[test]
fn test_LinearQuadraticRegulator_solve() {

    // generate row-major matrices
    let A = DMatrix::from_row_slice(2,2, &[
                              0., 1.,
                              0., 0.
    ]);

    let B = DMatrix::from_row_slice(2,1, &[
                              0.,
                              1.
    ]);

    let Q = DMatrix::<f32>::identity(2, 2);
    let R = DMatrix::from_vec(1,1, vec![1.]);

    let controller = LinearQuadraticRegulator::new(&A, &B, &Q, &R);

    let (K, P) = controller.solve();

    let P_true = DMatrix::from_row_slice(2,2, &[
                            3.0_f32.sqrt(), 1.,
                            1., 3.0_f32.sqrt()
    ]);

    let K_true = DMatrix::from_row_slice(1,2, &[
                            1., 3.0_f32.sqrt()
    ]);

    println!("P: {:?}", P);
    println!("P_true: {:?}", P_true);
    println!("K: {:?}", K);
    println!("K_true: {:?}", -K_true);

}
