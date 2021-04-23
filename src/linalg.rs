
use na::DMatrix;
use lapack::{Select2F32, sgees};

use crate::util::{hcombine, vcombine, MatrixCompareError, print_matrix};

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

/// solve algebraic riccati equation using Hamiltonian eigenvalue decomposition
/// Reference: "A Schur Method for Solving Algebraic Riccati Equations" - Alan J. Laub
/// Broken
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

    // multiplication of DMatrix returns DMatrix not a borrow
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

    let n = hamiltonian.shape().0 / 2; // U is shaped 2n x 2n

    print_matrix(&hamiltonian); // looks good here
    println!("**************");

    let mut u11 = DMatrix::<f32>::zeros(n,n);
    let mut u21 = DMatrix::<f32>::zeros(n,n);

    // manual stable eigenvalue/eigenvector extraction
    // let eigenvectors = hamiltonian.clone().symmetric_eigen().eigenvectors;
    // let eigenvalues = hamiltonian.clone().eigenvalues(); // row vector
    // // println!("{:?}", hamiltonian.clone().symmetric_eigen().eigenvalues);
    let mut U = DMatrix::<f32>::zeros(2*n, n);
    // let mut eigvec_list = Vec::<Vec<f32>>::new();
    // for (j, eig) in eigenvalues.unwrap().iter().enumerate() {
    //     println!("{:?}", eig);
    //     if eig < &0. {
    //         let mut eigvec = Vec::new();
    //         for i in 0..2*n {
    //             // println!("{:?}", eigenvectors[(i,j)]);
    //             // pull out the eigenvector corresponding to the stable eigenvalues
    //             eigvec.push(eigenvectors[(i,j)]);
    //         }
    //         eigvec_list.push(eigvec);
    //     }

    // }

    // println!("{:?}", eigvec_list);
    // // place eigenvector values into U
    // for (j, eigvec) in eigvec_list.iter().enumerate() {
    //     for (i, val) in eigvec.iter().enumerate() {
    //         // println!("{:?}", (i,j));
    //         U[(i,j)] = *val;
    //     }
    // }


    // perform Real Schur Decomposition (not a general schur decomposition)
    // Schur(hamiltonian) aka eigendecomposition of hamiltonian doesn't have eigenvalues on the
    // let schur = hamiltonian.schur();
    // let bub = hamiltonian.clone_owned();
    // let schur = hamiltonian.schur();

    // let (U, _Z) = schur.unpack();
    // need schur decomposition where upper eigenvalues are sorted by those in the left-hand plane
    // print_matrix(&U); // looks good here
    // println!("**************");
    // print_matrix(&_Z); // looks good here
    // println!("**************");
    // print_matrix(&_Z); // looks good here
    // println!("**************");

    //

    // The following works if schur decomposition can be ordered by lhp eigenvalues
    // indices defining U11 and U21 block matrices in U
    //

    // use LAPACK to compute ordered real schur decomposition of hamiltonian
    let (nrows, ncols) = hamiltonian.shape();
    extern "C" fn selectfcn(wr_j: *const f32, wr_i: *const f32) -> i32 { 1 } // TODO
    let select: Select2F32 = Some(selectfcn);
    let mut a = hamiltonian.as_mut_slice();
    let lda = n as i32;
    let mut sdim = 0;
    let mut wr = Vec::<f32>::with_capacity(n);
    let mut wi = Vec::<f32>::with_capacity(n);
    let ldvs = n as i32;
    let mut vs =  Vec::<f32>::with_capacity(ldvs as usize);
    let mut work = Vec::<f32>::with_capacity(n);
    let lwork = 3*n as i32;
    let mut bwork = Vec::<i32>::with_capacity(n);
    let mut info = 0;

    unsafe {
        sgees(
            b'V',
            b'S',
            select,
            n as i32,
            &mut a,
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






    println!("u11");
    print_matrix(&u11);
    println!("u21");
    print_matrix(&u21);

    // let u11_inv = u11.clone().try_inverse().unwrap();
    let u11_inv = u11.clone().try_inverse().unwrap();

    // TODO solve this as a linear system
    // P*u11 = u21
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



