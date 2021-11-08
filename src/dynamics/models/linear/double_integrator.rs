
use na::{DMatrix, DVector};
use crate::dynamics::linear_system::LTISystem;
use crate::dynamics::statespace::{Statespace, StatespaceType, StateSpaceRepresentation};
// use crate::util::print_matrix;


// TODO: implement the println trait for each and print out the matrices nicely

#[derive(Debug, Clone, PartialEq)]
pub struct DoubleIntegrator1D {

    dynamics: LTISystem,
    statespace: Statespace

}

impl DoubleIntegrator1D {

    pub fn new() -> Self {

        let A = DMatrix::from_row_slice(2, 2, &[0., 1., 0., 0.]);
        let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);
        let C = A.clone();
        let D = B.clone();

        let dynamics = LTISystem::new(A, B, C, D);

        let mut statespace = Statespace::new(2);
        statespace.add_state(0, StatespaceType::Position0);
        statespace.add_state(1, StatespaceType::Velocity0);

        Self { dynamics, statespace }

    }

    pub fn dynamics(&self) -> &LTISystem { &self.dynamics }

    pub fn statespace(&self) -> &Statespace { &self.statespace }

}

impl StateSpaceRepresentation for DoubleIntegrator1D {

    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.f(t, x, u)

    }

    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.h(t, x, u)

    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DoubleIntegrator2D {

    dynamics: LTISystem,
    statespace: Statespace

}

impl DoubleIntegrator2D {

    pub fn new() -> Self {

        let A = DMatrix::from_row_slice(
            4, 4,
            &[
                0., 0., 1., 0.,
                0., 0., 0., 1.,
                0., 0., 0., 0.,
                0., 0., 0., 0.
            ]
        );

        let B = DMatrix::from_row_slice(
            4, 2, 
            &[
                0., 0.,
                0., 0.,
                1., 0.,
                0., 1.
            ]
        );

        let C = A.clone();
        let D = B.clone();

        let dynamics = LTISystem::new(A, B, C, D);

        let mut statespace = Statespace::new(4);
        statespace.add_state(0, StatespaceType::Position0);
        statespace.add_state(1, StatespaceType::Position1);
        statespace.add_state(2, StatespaceType::Velocity0);
        statespace.add_state(3, StatespaceType::Velocity1);

        Self { dynamics, statespace }

    }

    pub fn dynamics(&self) -> &LTISystem { &self.dynamics }

    pub fn statespace(&self) -> &Statespace { &self.statespace }

}

impl StateSpaceRepresentation for DoubleIntegrator2D {

    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.f(t, x, u)

    }

    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.h(t, x, u)

    }
}


#[derive(Debug, Clone, PartialEq)]
pub struct DoubleIntegrator3D {

    dynamics: LTISystem,
    statespace: Statespace,

}

impl DoubleIntegrator3D {

    pub fn new() -> Self {

        let A = DMatrix::from_row_slice(
            6, 6,
            &[
                0., 0., 0., 1., 0., 0.,
                0., 0., 0., 0., 1., 0.,
                0., 0., 0., 0., 0., 1.,
                0., 0., 0., 0., 0., 0.,
                0., 0., 0., 0., 0., 0.,
                0., 0., 0., 0., 0., 0.
            ]
        );

        let B = DMatrix::from_row_slice(
            6, 3, 
            &[
                0., 0., 0.,
                0., 0., 0.,
                0., 0., 0.,
                1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.
            ]
        );

        let C = A.clone();
        let D = B.clone();

        let dynamics = LTISystem::new(A, B, C, D);

        let mut statespace = Statespace::new(6);
        statespace.add_state(0, StatespaceType::Position0);
        statespace.add_state(1, StatespaceType::Position1);
        statespace.add_state(2, StatespaceType::Position2);
        statespace.add_state(3, StatespaceType::Velocity0);
        statespace.add_state(4, StatespaceType::Velocity1);
        statespace.add_state(5, StatespaceType::Velocity2);

        println!("{:?}", &statespace);

        Self { dynamics, statespace }

    }

    pub fn dynamics(&self) -> &LTISystem { &self.dynamics }

    pub fn statespace(&self) -> &Statespace { &self.statespace }

}

impl StateSpaceRepresentation for DoubleIntegrator3D {

    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.f(t, x, u)

    }

    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.h(t, x, u)

    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_DoubleIntegrator1D_f() {

        let model = DoubleIntegrator1D::new();
        let x0 = DVector::from_vec(vec![10., 10.]);

        let xdot = model.f(0f32, &x0, None);

        println!("{:?}", xdot);

    }

    #[test]
    fn test_DoubleIntegrator2D_f() {

        let model = DoubleIntegrator2D::new();
        let x0 = DVector::from_vec(vec![10., 10., 10., -10.]);

        let xdot = model.f(0f32, &x0, None);

        println!("{:?}", xdot);

    }

    #[test]
    fn test_DoubleIntegrator3D_f() {

        let model = DoubleIntegrator3D::new();
        let x0 = DVector::from_vec(vec![10., 10., 10., -10., -30., 0.]);

        let xdot = model.f(0f32, &x0, None);

        println!("{:?}", xdot);

    }
}
