
use na::{DMatrix, DVector};
use crate::dynamics::statespace::StateSpaceRepresentation;

/// Defines a linear time-invariant system of equations
///
/// A: State/system matrix \
/// B: Input matrix \
/// C: Output matrix \
/// D: Feedforward matrix \
/// dx: Statespace size \
/// du: control input size \
#[derive(Debug, Clone, PartialEq)]
pub struct LTISystem {
    pub A: DMatrix<f32>,
    pub B: DMatrix<f32>,
    pub C: DMatrix<f32>,
    pub D: DMatrix<f32>,
    pub dx: usize,
    pub du: usize,
}

impl LTISystem {
    pub fn new(A: DMatrix<f32>, B: DMatrix<f32>, C: DMatrix<f32>, D: DMatrix<f32>) -> Self {
        let dx = A.shape().1 as usize;
        let du = B.shape().1 as usize;

        Self { A, B, C, D, dx, du }
    }
}

impl StateSpaceRepresentation for LTISystem {

    /// Solves for the State vector - \dot{x} = Ax + Bu = f(x,u)
    /// t: time
    /// x: State vector
    /// u: Control/input vector
    fn f(&self, _t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        let result = match u {
            Some(u) => &self.A * x + &self.B * u,
            None => &self.A * x,
        };

        result
    }

    /// Solves for the Output vector - y = Cx + Du = h(x,u)
    /// t: time
    /// x: State vector
    /// u: Control/input vector
    fn h(&self, _t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        let result = match u {
            Some(u) => &self.C * x + &self.D * u,
            None => &self.C * x,
        };

        result
    }
}


#[cfg(test)]
mod tests {

    use super::*;
    use na::{DMatrix, DVector};

    #[test]
    fn test_LinearSystem_solve() {

        // generate row-major matrices
        let A = DMatrix::from_row_slice(2, 2, &[1., 1., 0., 1.]);
        let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);
        let C = A.clone();
        let D = B.clone();

        // move A, B, C, D into double_integrator
        let double_integrator = LTISystem::new(A, B, C, D);

        let x = DVector::from_vec(vec![10., 10.]);
        let u = DVector::from_vec(vec![10.]);

        let result = double_integrator.f(0f32, &x, Some(&u));

        assert_eq!(result, DVector::from_vec(vec![20., 20.]));

    }

    #[test]
    fn test_LinearSystem_solve_output() {

        let C = DMatrix::from_row_slice(1, 2, &[1., 0.]);
        let D = DMatrix::<f32>::zeros(1, 1);
        let A = C.clone();
        let B = D.clone();

        // move A, B, C, D into double_integrator
        let double_integrator = LTISystem::new(A, B, C, D);

        let x = DVector::from_vec(vec![10., 10.]);

        let u = DVector::from_vec(vec![10.]);

        let result = double_integrator.h(0f32, &x, Some(&u));

        assert_eq!(result, DVector::from_vec(vec![10.]));

    }
}
