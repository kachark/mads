
use crate::util::range_step;

use na::DVector;

pub fn ForwardEuler<F>(
    t0: f32,
    y0: DVector<f32>,
    h: f32,
    tn: f32,
    f: F,
) -> (Vec<f32>, Vec<DVector<f32>>)
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>,
{
    let time = range_step(t0, tn, h);

    let mut y: Vec<DVector<f32>> = vec![DVector::<f32>::zeros(y0.len()); time.len()];
    y[0] = y0.clone();

    for k in 0..time.len() - 1 {
        let yk = &y[k].clone();

        let m = h * f(time[k], &yk);

        // Need to clone because y owns the state data
        y[k + 1] = yk + m;
    }

    (time, y)
}


pub fn MidPointEuler<F>(
    t0: f32,
    y0: DVector<f32>,
    h: f32,
    tn: f32,
    f: F,
) -> (Vec<f32>, Vec<DVector<f32>>)
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>,
{
    let time = range_step(t0, tn, h);

    let mut y: Vec<DVector<f32>> = vec![DVector::<f32>::zeros(y0.len()); time.len()];
    y[0] = y0.clone();

    for k in 0..time.len() - 1 {
        let yk = &y[k].clone();

        let y_half = yk + (h / 2.0 * f(time[k], yk));

        y[k + 1] = yk + (h * f(time[k] + (h/2.0), &y_half));
    }

    (time, y)
}

#[cfg(test)]
#[test]
fn test_ForwardEuler() {

    use na::DMatrix;
    use crate::controls::lqr::LinearQuadraticRegulator as LQR;
    use crate::dynamics::linear_system::*;

    // generate row-major matrices
    let A = DMatrix::from_row_slice(
        2,
        2,
        &[
            0., 1.,
            0., 0.
        ]
    );

    let B = DMatrix::from_row_slice(
        2, 1,
        &[
            0.,
            1.
        ]
    );

    // unused - clone for completeness
    let C = A.clone();
    let D = B.clone();

    let Q = DMatrix::<f32>::identity(2, 2);
    let R = DMatrix::from_vec(1, 1, vec![1.]);

    let double_integrator = LinearSystem::new(A, B, C, D);
    let lqr = LQR::new(
        double_integrator.A.clone(),
        double_integrator.B.clone(),
        Q,
        R,
    );

    let y0 = DVector::from_vec(vec![10., -40.]);
    let (K, _P) = match lqr.solve() {
        Ok((value1, value2)) => (value1, value2),
        Err(_) => (DMatrix::<f32>::zeros(1, 1), DMatrix::<f32>::zeros(1, 1)),
    };

    // Wrap dynamics/controls in appropriately defined closure
    let f = |t: f32, x: &DVector<f32>| {
        let u = -&K * x;
        double_integrator.f(t, x, Some(&u))
    };

    let (_t, y) = ForwardEuler(0.0, y0, 0.1, 10.0, f);

    for ele in y.iter() {
        println!("{:?}", ele.data);
    }
}

#[test]
fn test_MidPointEuler() {

    use na::DMatrix;
    use crate::controls::lqr::LinearQuadraticRegulator as LQR;
    use crate::dynamics::linear_system::*;

    // generate row-major matrices
    let A = DMatrix::from_row_slice(
        4,
        4,
        &[
            0., 0., 1., 0.,
            0., 0., 0., 1.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
        ],
    );

    let B = DMatrix::from_row_slice(
        4,
        2,
        &[
            0., 0., 0., 0.,
            1., 0., 0., 1.
        ]
    );

    // unused - clone for completeness
    let C = A.clone();
    let D = B.clone();

    let Q = DMatrix::<f32>::identity(4, 4);
    let R = DMatrix::<f32>::identity(2, 2);

    let double_integrator = LinearSystem::new(A, B, C, D);
    let lqr = LQR::new(
        double_integrator.A.clone(),
        double_integrator.B.clone(),
        Q,
        R,
    );

    let (K, _P) = match lqr.solve() {
        Ok((value1, value2)) => (value1, value2),
        Err(_) => panic!["LQR solve"],
    };

    let y0 = DVector::from_vec(vec![10., 10., 10., 10.]);

    // Wrap dynamics/controls in appropriately defined closure
    let f = |t: f32, x: &DVector<f32>| {
        let u = -&K * x;
        double_integrator.f(t, x, Some(&u))
    };

    let (_t, y) = MidPointEuler(0.0, y0, 0.1, 10.0, f);

    for ele in y.iter() {
        println!("{:?}", ele.data);
    }
}
