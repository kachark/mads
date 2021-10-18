
use na::{DMatrix, DVector};
use crate::dynamics::linear_system::LinearSystem;
use crate::dynamics::statespace::StateSpaceRepresentation;


#[derive(Debug, Clone, PartialEq)]
pub struct EulerHill3D {

    pub dynamics: LinearSystem

}

impl EulerHill3D {

    pub fn new() -> Self {

        // let r: f32 = 42164E3; // geostationary orbit
        let r: f32 = 100E3; // LEO
        let mu: f32 = 3.986004418E14;

        let _n = (mu/r.powf(3f32)).sqrt();

        let n: f32 = 0.00113;

        let A = DMatrix::from_row_slice(
            6, 6,
            &[
                0., 0., 0., 1., 0., 0.,
                0., 0., 0., 0., 1., 0.,
                0., 0., 0., 0., 0., 1.,
                3.0*n.powf(2f32), 0., 0., 0., 2.0*n, 0.,
                0., 0., 0., -2.0*n, 0., 0.,
                0., 0., -n.powf(2f32), 0., 0., 0.
            ]
        );


//         let n: f32 = 0.00113; // LEO orbit - omega_dot
//         // let R: f32 = 405.0 + 6870.0;
//         // let mu = 398600.5;
//         // let n = (mu/(R.powf(3.0))).sqrt();
//         let tau = n*t;

//         let cos_nt = tau.cos();
//         let sin_nt = tau.sin();


//         // clohessy-whiltshire - take this and change with with small angle approx.
//         let A = DMatrix::from_row_slice(
//             6, 6,
//             &[
//                 (4. - 3.*cos_nt), 0., 0., (1./n)*sin_nt, (2./n)*(1.-cos_nt), 0.,
//                 6.*(sin_nt - tau), 1., 0., -(2./n)*(1.-cos_nt), (1./n)*(4.*sin_nt - 3.*tau), 0.,
//                 0., 0., cos_nt, 0., 0., (1./n)*sin_nt,
//                 3.*n*sin_nt, 0., 0., cos_nt, 2.*sin_nt, 0.,
//                 -6.*n*(1.-cos_nt), 0., 0., -2.*sin_nt, (4.*cos_nt - 3.), 0.,
//                 0., 0., -n*sin_nt, 0., 0., cos_nt
//             ]
//         );

        let B = DMatrix::zeros(6,3);
        let C = A.clone();
        let D = B.clone();

        let dynamics = LinearSystem::new(A, B, C, D);

        Self { dynamics }

    }



}

impl StateSpaceRepresentation for EulerHill3D {

    fn f(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.f(t, x, u)

    }

    fn h(&self, t: f32, x: &DVector<f32>, u: Option<&DVector<f32>>) -> DVector<f32> {

        self.dynamics.h(t, x, u)

    }
}


