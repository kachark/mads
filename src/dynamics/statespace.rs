
use na::DVector;

/// Defines an interface for solving systems of first-order ODEs according to the State Space model
pub trait StateSpaceRepresentation {
    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
}

