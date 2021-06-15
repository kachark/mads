
use na::DVector;

/// Defines an interface for solving systems of equations according to a State Space
/// model
pub trait StateSpaceRepresentation {
    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32>;
}

// /// Defines a generic statespace for dynamic mechanical systems in a Cartesian coordinate system
// pub struct StateSpace {
//     position: (i32, i32, i32),
//     velocity: (i32, i32, i32),
//     attitude: (i32, i32, i32),
//     angular_velocity: (i32, i32, i32),
// }
