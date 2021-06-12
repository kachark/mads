
use nalgebra::DVector;

use crate::dynamics::models::linear::double_integrator::*;
use crate::dynamics::models::linear::inverted_pendulum::*;
use crate::dynamics::models::linear::euler_hill::*;
use crate::controls::lqr::LinearQuadraticRegulator;


// Define Components
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Velocity {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

#[derive(Clone, Debug, PartialEq)]
pub struct FullState(pub DVector<f32>);


#[derive(Clone, Debug, PartialEq)]
pub struct DoubleIntegratorDynamics2D {
    pub name: DoubleIntegrator2D
}

#[derive(Clone, Debug, PartialEq)]
pub struct DoubleIntegratorDynamics3D {
    pub name: DoubleIntegrator3D
}

#[derive(Clone, Debug, PartialEq)]
pub struct EulerHillDynamics {
    pub name: EulerHill3D
}

#[derive(Clone, Debug, PartialEq)]
pub struct LinearizedInvertedPendulumDynamics {
    pub name: InvertedPendulum
}

#[derive(Clone, Debug, PartialEq)]
pub struct LinearFeedbackController {
    pub name: LinearQuadraticRegulator
}


#[derive(Clone, Debug, PartialEq)]
pub struct Plotter {

}



