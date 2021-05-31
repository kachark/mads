
use crate::util::range_step;

use std::cmp;
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


// Reference: https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf
pub fn RungeKutta45<F>(
    t0: f32,
    y0: DVector<f32>,
    h: f32,
    tn: f32,
    f: F,
) -> (Vec<f32>, Vec<DVector<f32>>)
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>,
{
    let mut h = h;
    let tol = 1E-5;
    let mut time: Vec<f32> = Vec::new();
    let mut y: Vec<DVector<f32>> = Vec::new();

    time.push(t0);
    y.push(y0);

    let mut _k = 0;
    while time[time.len()-1] < tn {

        h = h.min(tn - time[time.len()-1]);

        let tk = time[time.len()-1];
        let yk = &y[y.len()-1].clone();

        let k1 = h * f(tk, yk);
        let k2 = h * f(tk + (1./4.)*h, &(yk + (1./4.) * &k1));
        let k3 = h * f(tk + (3./8.)*h, &(yk + (3./32.) * &k1 + (9./32.) * &k2));
        let k4 = h * f(tk + (12./13.)*h, &(yk + (1932./2197.) * &k1 - (7200./2197.) * &k2 + (7296./2197.) * &k3));
        let k5 = h * f(tk + h, &(yk + (439./216.) * &k1 - 8. * &k2 + (3680./513.) * &k3 - (845./4104.) * &k4));
        let k6 = h * f(tk + (1./2.)*h, &(yk - (8./27.) * &k1 + 2. * &k2 - (3544./2565.) * &k3 + (1859./4104.) * &k4 - (11./40.) * &k5));

        // Fourth-order Runge-Kutta result
        let w1 = yk + (25.*&k1/216.) + (1408.*&k3/2565.) + (2197.*&k4/4104.) - (1.*&k5/5.);

        // Fifth-order Runge-Kutta result
        let w2 = yk + (16.*&k1/135.) + (6656.*&k3/12825.) + (28561.*&k4/56430.) - (9.*&k5/50.) + (2.*&k6/55.);

        let truncation_error: f32 = (1.0/h) * (&w2 - &w1).norm();
        let s = 0.84 * (tol / truncation_error).powf(0.25);

        // Step size satisfies error tolerance, accept this value
        if truncation_error <= tol {

            time.push(tk + h);
            y.push(w1);
            _k += 1;
            h = s*h;

        } else { // continue searching for a better step size

            h = s*h;

        }
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


#[test]
fn test_RungeKutta45() {

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

    let (_t, y) = RungeKutta45(0.0, y0, 0.1, 10.0, f);

    for ele in y.iter() {
        println!("{:?}", ele.data);
    }
}
