
use na::DVector;

/// Defines an interface for solving closed-form equations
///
/// x(t) = f(t, x(t))
///
pub trait ClosedFormSolution {
    fn rhs(&self, t: f32, x: &DVector<f32>) -> DVector<f32>;
}

