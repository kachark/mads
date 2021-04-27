
use na::DMatrix;
use lapack::{Select2F32, sgees};

use crate::util::{block, hcombine, vcombine, MatrixCompareError, print_matrix};

// TODO error type for linalg failures?

/// Iterative solver for Algebraic Riccati Equation for continuous models
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

/// Iterative solver for Algebraic Riccati Equation for discrete models
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

// TODO return error on failure
/// solve algebraic riccati equation using Hamiltonian eigenvalue decomposition
/// Reference: "A Schur Method for Solving Algebraic Riccati Equations" - Alan J. Laub
fn solve_continuous_riccati_eigen(A: &DMatrix<f32>,
                                  B: &DMatrix<f32>,
                                  Q: &DMatrix<f32>,
                                  R: &DMatrix<f32>) -> DMatrix<f32> {

    let dx = A.shape().0;

    let P: DMatrix<f32>;

    let mut Rinv = DMatrix::<f32>::zeros(R.shape().0, R.shape().1);
    let r = R.clone_owned();
    na::linalg::try_invert_to(r, &mut Rinv);
    let BRinvBT = B*Rinv*B.transpose(); // matrix operations perform a move
    let AT = A.transpose();

    // multiplication of DMatrix returns DMatrix, not a borrow
    // row-major listing matrices to be placed into hamiltonian
    // let matrices: [&DMatrix<f32>; 4] = [A, &BRinvBT, &(-Q), &AT];

    let upper_block = match hcombine(A, &(-BRinvBT)) {
        Ok(concatenated) => concatenated, // assign concatenated to upper block
        Err(MatrixCompareError) => DMatrix::<f32>::zeros(2*dx, 2*dx)
    };

    let lower_block = match hcombine(&(-Q), &(-AT)) {
        Ok(concatenated) => concatenated, // assign concatenated to lower block
        Err(MatrixCompareError) => DMatrix::<f32>::zeros(2*dx, 2*dx)
    };

    let mut hamiltonian = match vcombine(&upper_block, &lower_block) {
        Ok(concatenated) => concatenated, // assign concatenated to hamiltonian
        Err(MatrixCompareError) => DMatrix::<f32>::zeros(2*dx, 2*dx)
    };

    // use LAPACK to compute ordered real schur decomposition of hamiltonian
    // Ordering ensures eigenvalues placed in top left of quasi-upper triangular matrix are stable / negative

    // C function to sort only the eigenvalues with a negative real part
    // "An eigenvalue wr(j)+sqrt(-1)wi(j) is selected if select(wr(j), wi(j)) is true"
    // wr_j: real part of the eigenvalue
    // wi_j: imaginary part of eigenvalue
    extern "C" fn selectfcn(wr_j: *const f32, _wi_j: *const f32) -> i32 { 
        unsafe {
            match wr_j.as_ref() { // dereference raw pointer
                Some(val) => (*val < 0.0) as i32,
                None => 0
            }
       }
    }

    let (nrows, _ncols) = hamiltonian.shape();
    let select: Select2F32 = Some(selectfcn);
    let lda = nrows as i32;
    let mut sdim = 0;
    // wr
    let mut wr = Vec::with_capacity(nrows);
    unsafe { wr.set_len(nrows) };
    // wi
    let mut wi = Vec::with_capacity(nrows);
    unsafe { wi.set_len(nrows) };
    // vs
    let ldvs = nrows as i32;
    let mut vs = Vec::with_capacity((ldvs as usize)*nrows);
    unsafe { vs.set_len((ldvs as usize) * nrows) };
    // rwork
    let mut work = Vec::with_capacity(1 as usize);
    unsafe { work.set_len(1 as usize) };
    // bwork
    let mut bwork = Vec::with_capacity(nrows);
    unsafe { bwork.set_len(nrows) };
    let lwork = -1;

    let mut info = 0;

    // Compute the optimal size of the workspace array
    unsafe {
        sgees(
            b'V',
            b'S',
            select,
            nrows as i32,
            hamiltonian.as_mut_slice(),
            lda,
            &mut sdim,
            &mut wr,
            &mut wi,
            &mut vs,
            ldvs,
            &mut work,
            lwork,
            &mut bwork,
            &mut info
        );
    }

    let lda = nrows as i32;
    let mut sdim = 0;
    let lwork = work[0] as i32; // retrieve optimal workspace size
    let mut wr = Vec::with_capacity(nrows);
    unsafe { wr.set_len(nrows) };
    let mut wi = Vec::with_capacity(nrows);
    unsafe { wi.set_len(nrows) };
    let ldvs = nrows as i32;
    let mut vs = Vec::with_capacity((ldvs as usize) * nrows);
    unsafe { vs.set_len((ldvs as usize) * nrows) };
    let mut work = Vec::with_capacity(lwork as usize);
    unsafe { work.set_len(lwork as usize) };
    let mut bwork = Vec::with_capacity(nrows);
    unsafe { bwork.set_len(nrows) };
    let mut info = 0;

    // Compute the Schur Factorization
    unsafe {
        sgees(
            b'V',
            b'S',
            select,
            nrows as i32,
            hamiltonian.as_mut_slice(),
            lda,
            &mut sdim,
            wr.as_mut_slice(),
            wi.as_mut_slice(),
            vs.as_mut_slice(),
            ldvs,
            &mut work,
            lwork,
            &mut bwork,
            &mut info
        );
    }

    // println!("Z"); //  Real-schur form matrix
    // print_matrix(&hamiltonian);
    // println!("T"); // unitary matrix
    let T = DMatrix::from_column_slice(nrows, _ncols, &vs);
    // print_matrix(&T);

    // println!("T Z T^T");
    // print_matrix(&(&T * &hamiltonian * &T.transpose()));

    let u11 = block((0,0), (1,1), &T);
    let u21 = block((2,0), (3,1), &T);

    // TODO handle optional safely
    let u11_inv = u11.clone().try_inverse().unwrap();

    P = u21 * u11_inv;
    P

}





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

    println!("P: ");
    print_matrix(&P);
    println!("P_true: ");
    print_matrix(&P_true);

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

    println!("P: ");
    print_matrix(&P);
    println!("P_true: ");
    print_matrix(&P_true);

}

#[test]
fn test_solve_continuous_riccati_eigen() {

    // generate row-major matrices
    let A = DMatrix::from_row_slice(2,2, &[
                              4., 3.,
                              -4.5, -3.5
    ]);

    let B = DMatrix::from_row_slice(2,1, &[
                              1.,
                              -1.
    ]);

    let Q = DMatrix::from_row_slice(2,2, &[
                                9., 6.,
                                6., 4.
    ]);
    let R = DMatrix::from_vec(1,1, vec![1.]);


    let P = solve_continuous_riccati_eigen(&A, &B, &Q, &R);

    let P_true = DMatrix::from_row_slice(2,2, &[
                            21.72792206, 14.48528137,
                            14.48528137, 9.65685425
    ]);

    println!("P: ");
    print_matrix(&P);
    println!("P_true: ");
    print_matrix(&P_true);

}



