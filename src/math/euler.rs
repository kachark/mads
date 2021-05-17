
// NOTE: temporary
use crate::dynamics::linear_system::linear_dynamics::*;
use crate::dynamics::controls::lqr::LinearQuadraticRegulator as LQR;

use na::{DMatrix, DVector};

pub fn EulerMethod(t0: f32, y0: DVector<f32>, h: f32, tn: f32) -> (Vec<f32>, Vec<DVector<f32>>) {


    // generate row-major matrices
    let A = DMatrix::from_row_slice(2,2, &[
                            0., 1.,
                            0., 0.
    ]);

    let B = DMatrix::from_row_slice(2,1, &[
                            0.,
                            1.
    ]);

    // unused - clone for completeness
    let C = A.clone();
    let D = B.clone();

    let Q = DMatrix::<f32>::identity(2, 2);
    let R = DMatrix::from_vec(1,1, vec![1.]);

    let double_integrator = LinearSystem::new(A, B, C, D);
    let lqr = LQR::new(double_integrator.A.clone(), double_integrator.B.clone(), Q, R);

    // let y0 = DVector::from_vec(vec![10.,10.]);
    let (K, _P) = match lqr.solve(){
        Ok( (value1, value2) ) => (value1, value2),
        Err(_) => (DMatrix::<f32>::zeros(1,1), DMatrix::<f32>::zeros(1,1))
    };

    // TODO: put into separate "linspace" function
    // linspace (t0, h, tn);

    let mut time = Vec::new();
    let mut count = t0;
    while count < tn {
        time.push(count);
        count += h;
    }
    // println!("{:?}", time);

    let mut y: Vec<DVector<f32>> = vec![DVector::<f32>::zeros(y0.len()); time.len()];
    y[0] = y0.clone();

    for k in 0..time.len()-1 {

        let tmp = &y[k].clone();

        let u = -&K * &y[k];
        let m = h * double_integrator.solve(&tmp, Some(&u));

        // Need to clone because y owns the state data
        y[k+1] = tmp + m;
    }

    (time, y)

}


#[test]

#[cfg(test)]

fn test_EulerMethod() {

    use crate::dynamics::linear_system::linear_dynamics::*;
    use crate::dynamics::controls::lqr::LinearQuadraticRegulator as LQR;

    // generate row-major matrices
    let A = DMatrix::from_row_slice(2,2, &[
                            0., 1.,
                            0., 0.
    ]);

    let B = DMatrix::from_row_slice(2,1, &[
                            0.,
                            1.
    ]);

    // unused - clone for completeness
    let C = A.clone();
    let D = B.clone();

    let mut Q = DMatrix::<f32>::identity(2, 2);
    // Q[(0,0)] = 10.;
    // Q[(1,1)] = 0.;
    let R = DMatrix::from_vec(1,1, vec![1.]);

    let double_integrator = LinearSystem::new(A, B, C, D);
    let lqr = LQR::new(double_integrator.A.clone(), double_integrator.B.clone(), Q, R);

    let y0 = DVector::from_vec(vec![10.,-40.]);
    let (K, _P) = match lqr.solve(){
        Ok( (value1, value2) ) => (value1, value2),
        Err(_) => (DMatrix::<f32>::zeros(1,1), DMatrix::<f32>::zeros(1,1))
    };


    // TODO: somehow pass double_integrator.solve() into the eulermethod -> maybe pass a function
    // signature with a LinearSystem trait?
    let (t, y) = EulerMethod(0.0, y0, 0.005, 20.0);

    for ele in y.iter() {
        println!("{:?}", ele.data);
    }

}
