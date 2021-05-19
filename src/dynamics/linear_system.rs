pub mod linear_dynamics {

    use na::{DMatrix, DVector};

    /// Defines an interface for solving linear systems of equations according to a State Space
    /// model
    pub trait StateSpaceRepresentation {
        fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
        fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
    }

    /// Defines a linear system of equations
    #[derive(Clone)]
    pub struct LinearSystem {
        /// A: State/system matrix
        /// B: Input matrix
        /// C: Output matrix
        /// D: Feedforward matrix
        /// dx: Statespace size
        /// du: control input size
        pub A: DMatrix<f32>,
        pub B: DMatrix<f32>,
        pub C: DMatrix<f32>,
        pub D: DMatrix<f32>,
        pub dx: u32,
        pub du: u32,
    }

    impl LinearSystem {
        pub fn new(A: DMatrix<f32>, B: DMatrix<f32>, C: DMatrix<f32>, D: DMatrix<f32>) -> Self {
            let dx = A.shape().1 as u32;
            let du = B.shape().1 as u32;

            Self { A, B, C, D, dx, du }
        }
    }

    impl StateSpaceRepresentation for LinearSystem {
        /// Implements a Linear Time-Invariant system

        /// Solves for the State vector - \dot{x} = Ax + Bu = f(x,u)
        /// t: time
        /// x: State vector
        /// u: Control/input vector
        fn f(&self, _t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {
            let result: DVector<f32>;

            match u {
                Some(u) => result = &self.A * x + &self.B * u,
                None => result = &self.A * x,
            }

            result
        }

        /// Solves for the Output vector - y = Cx + Du = h(x,u)
        /// t: time
        /// x: State vector
        /// u: Control/input vector
        fn h(&self, _t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {
            let result: DVector<f32>;

            match u {
                Some(u) => result = &self.C * x + &self.D * u,
                None => result = &self.C * x,
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
    use na::{DMatrix, DVector};

    // generate row-major matrices
    let A = DMatrix::from_row_slice(2, 2, &[1., 1., 0., 1.]);

    let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);

    let C = A.clone();
    let D = B.clone();

    // move A, B, C, D into double_integrator
    let double_integrator = linear_dynamics::LinearSystem::new(A, B, C, D);

    let x = DVector::from_vec(vec![10., 10.]);
    let u = DVector::from_vec(vec![10.]);

    let result = double_integrator.f(0f32, &x, Some(&u));
    // let result = double_integrator.solve(&x, None);
    // let result = double_integrator.A.dot(&x);

    assert_eq!(result, DVector::from_vec(vec![20., 20.]));
}

#[test]
fn test_LinearSystem_solve_output() {
    // import StateSpaceRepresentation trait to access related methods
    use linear_dynamics::StateSpaceRepresentation;
    use na::{DMatrix, DVector};

    let C = DMatrix::from_row_slice(1, 2, &[1., 0.]);
    let D = DMatrix::<f32>::zeros(1, 1);
    let A = C.clone();
    let B = D.clone();

    // move A, B, C, D into double_integrator
    let double_integrator = linear_dynamics::LinearSystem::new(A, B, C, D);

    let x = DVector::from_vec(vec![10., 10.]);

    let u = DVector::from_vec(vec![10.]);

    let result = double_integrator.h(0f32, &x, Some(&u));
    // let result = double_integrator.solve_output(&x, None);

    assert_eq!(result, DVector::from_vec(vec![10.]));
}
