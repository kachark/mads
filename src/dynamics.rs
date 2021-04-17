

pub mod linear_dynamics {

    use na::{DVector, DMatrix};

    pub trait StateSpaceRepresentation {

        fn solve(&self, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
        fn solve_output(&self, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;

    }

    pub struct LinearSystem {
        pub A: DMatrix<f32>,
        pub B: DMatrix<f32>,
        pub C: DMatrix<f32>,
        pub D: DMatrix<f32>,
        pub dx: u32,
        pub du: u32
    }

    impl LinearSystem {

        pub fn new(A: DMatrix<f32>, B: DMatrix<f32>, C: DMatrix<f32>, D: DMatrix<f32>) -> Self {

            let dx = A.shape().1 as u32;
            let du = B.shape().1 as u32;

            Self {
                A,
                B,
                C,
                D,
                dx,
                du
            }

        }
    }

    impl StateSpaceRepresentation for LinearSystem {

        fn solve(&self, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

            let result: DVector<f32>;

            match u {
                Some(u) => result = &self.A*x + &self.B*u,
                None => result = &self.A*x
            }

            result

        }

        fn solve_output(&self, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

            let result: DVector<f32>;

            match u {
                Some(u) => result = &self.C*x + &self.D*u,
                None => result = &self.C*x
            }

            result

        }

    }

}

#[cfg(test)]

#[test]
fn test_LinearSystem_solve() {

    // import StateSpaceRepresentation trait to access related methods
    use linear_dynamics::StateSpaceRepresentation;
    use na::{DVector, DMatrix};

    // generate row-major matrices
    let A = DMatrix::from_row_slice(2,2, &[
                              1., 1.,
                              0., 1.
    ]);

    let B = DMatrix::from_row_slice(2,1, &[
                              0.,
                              1.
    ]);

    let C = A.clone();
    let D = B.clone();

    // move A, B, C, D into double_integrator
    let mut double_integrator = linear_dynamics::LinearSystem::new(A, B, C, D);

    let x = DVector::from_vec(vec![10.,10.]);
    let u = DVector::from_vec(vec![10.]);

    let result = double_integrator.solve(&x, Some(&u));
    // let result = double_integrator.solve(&x, None);
    // let result = double_integrator.A.dot(&x);

    assert_eq!(result, DVector::from_vec(vec![20.,20.]));

}


#[test]
fn test_LinearSystem_solve_output() {

    // import StateSpaceRepresentation trait to access related methods
    use linear_dynamics::StateSpaceRepresentation;
    use na::{DVector, DMatrix};

    let C = DMatrix::from_row_slice(1, 2, &[
        1.,0.
    ]);
    let D = DMatrix::<f32>::zeros(1,1);
    let A = C.clone();
    let B = D.clone();

    // move A, B, C, D into double_integrator
    let mut double_integrator = linear_dynamics::LinearSystem::new(A, B, C, D);

    let x = DVector::from_vec(vec![
        10.,
        10.
    ]);

    let u = DVector::from_vec(vec![
        10.
    ]);

    let result = double_integrator.solve_output(&x, Some(&u));
    // let result = double_integrator.solve_output(&x, None);

    assert_eq!(result, DVector::from_vec(vec![10.]));

}




