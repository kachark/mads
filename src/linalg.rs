

use na::DMatrix;

// Iterative solver for Algebraic Riccati Equation for continuous models
pub fn solve_continuous_riccati_iterative(A: &DMatrix<f32>,
                           B: &DMatrix<f32>,
                           Q: &DMatrix<f32>,
                           R: &DMatrix<f32>,
                           dt: f32, iter_max: u32, tolerance: f32) -> DMatrix<f32> {

    let mut P = Q.clone_owned();

    let mut P_next: DMatrix<f32>;

    let AT = A.transpose();
    let BT = B.transpose();
    let mut Rinv = R.clone_owned();
    Rinv.try_inverse_mut();

    let mut diff: f32;
    for i in 0..iter_max {

        P_next = &P + ((&P * A) + (&AT * &P) - (&P*B*&Rinv*&BT*&P) + Q) * dt;

        diff = (&P_next - &P).amax();
        P = P_next;

        if diff < tolerance {
            println!("{:?}", i);
            break;
        }
    }

    P

}

// Iterative solver for Algebraic Riccati Equation for discrete models
pub fn solve_discrete_riccati_iterative(A: &DMatrix<f32>,
                           B: &DMatrix<f32>,
                           Q: &DMatrix<f32>,
                           R: &DMatrix<f32>,
                           iter_max: u32, tolerance: f32) -> DMatrix<f32> {

    let mut P = Q.clone_owned();

    let mut P_next: DMatrix<f32>;

    let AT = A.transpose();
    let BT = B.transpose();
    let mut Rinv = R.clone_owned();
    Rinv.try_inverse_mut();

    let mut diff: f32;
    for i in 0..iter_max {

        let mut RBTPBinv = (R + &BT * &P * B).clone_owned();
        RBTPBinv.try_inverse_mut();

        P_next = &AT * &P * A - &AT * &P * B * RBTPBinv * &BT * &P * A + Q;

        diff = (&P_next - &P).amax();
        P = P_next;

        if diff < tolerance {
            println!("{:?}", i);
            break;
        }
    }

    P

}

// solve algebraic riccati equation using Hamiltonian
// fn solve_continuous_riccati_hamiltonian(&self) -> DMatrix<f32> {

//     let dx = self.A.shape().0;
//     let du = self.B.shape().1;

//     let P: DMatrix<f32>;
//     let hamiltonian = DMatrix::<f32>::zeros(2*dx, 2*du);

//     let R_inv: DMatrix<f32>;
//     match &self.R.try_inverse() {
//         Some(inverted) => R_inv = *inverted,
//         None => {}
//     }

//     let R_inv_BT = R_inv * &self.B.transpose();

//     let Z = DMatrix::from_row_slice(2, 2, &[
//                                     &self.A, &self.B,
//                                     -&self.Q, -(&self.A.transpose())
//     ]);

//     let eigvec = DMatrix::<f32>::zeros(2*dx, dx);
//     let j = 0;
//     for i in 0..2*dx {
//         eigvec.
//     }

// }





#[cfg(test)]

#[test]
fn test_solve_continuous_riccati_iterative() {

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

    let P = solve_continuous_riccati_iterative(&A, &B, &Q, &R,
                            0.001, 100000, 1E-5);

    let P_true = DMatrix::from_row_slice(2,2, &[
                            3.0_f32.sqrt(), 1.,
                            1., 3.0_f32.sqrt()
    ]);

    println!("P: {:?}", P);
    println!("P_true: {:?}", P_true);

}


#[test]
fn test_solve_discrete_riccati_iterative() {

    // generate row-major matrices
    let A = DMatrix::from_row_slice(2,2, &[
                              0., 1.,
                              0., -1.
    ]);

    let B = DMatrix::from_row_slice(2,2, &[
                              1., 0.,
                              2., 1.
    ]);

    let Q = DMatrix::from_row_slice(2,2, &[
                              -4., -4.,
                              -4., 7.
    ]);

    let R = DMatrix::from_row_slice(2,2, &[
                              9., 3.,
                              3., 1.
    ]);

    let P = solve_discrete_riccati_iterative(&A, &B, &Q, &R,
                            100000, 1E-5);

    let P_true = DMatrix::from_row_slice(2,2, &[
                            -4.0, -4.0,
                            -4.0, 7.0
    ]);

    println!("P: {:?}", P);
    println!("P_true: {:?}", P_true);

}


