
use nalgebra::DVector;
use specs::{Component, VecStorage};

use crate::dynamics::models::linear::double_integrator::*;
use crate::dynamics::models::linear::euler_hill::*;
use crate::controls::lqr::LinearQuadraticRegulator;


// Define Components
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Velocity {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct FullState(pub DVector<f32>);


#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct DoubleIntegratorDynamics2D {
    pub name: DoubleIntegrator2D
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct DoubleIntegratorDynamics3D {
    pub name: DoubleIntegrator3D
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct EulerHillDynamics {
    pub name: EulerHill3D
}

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct LinearFeedbackController {
    pub name: LinearQuadraticRegulator
}


#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Plotter {

}



