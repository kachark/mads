
use na::DVector;
use na::base::UniformNorm;
use std::f32;

// Reference: https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf
// Runge-Kutta-Fehlberg method
pub fn RKF45<F>(
    f: F,
    t0: f32,
    y0: DVector<f32>,
    tf: f32,
    step: f32,
    rtol: f32
) -> (Vec<f32>, Vec<DVector<f32>>)
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>,
{

    let iterations = (tf - t0) / step;

    let mut h = step;
    let mut time: Vec<f32> = Vec::with_capacity(iterations as usize);
    let mut y: Vec<DVector<f32>> = Vec::with_capacity(iterations as usize);
    let atol = 1E-10;

    time.push(t0);
    y.push(y0);

    let mut tk = t0;

    // inner-step iteration
    let mut _k = 0;

    // global iteration
    let mut _it = 1;

    let a2 = 1./4.;
    let a3 = 3./8.;
    let a4 = 12./13.;
    let a5 = 1.;
    let a6 = 1./2.;

    let b21 = 1./4.;
    let b31 = 3./32.;
    let b32 = 9./32.;
    let b41 = 1932./2197.;
    let b42 = -7200./2197.;
    let b43 = 7296./2197.;
    let b51 = 439./216.;
    let b52 = -8.;
    let b53 = 3680./513.;
    let b54 = -845./4104.;
    let b61 = -8./27.;
    let b62 = 2.;
    let b63 = -3544./2565.;
    let b64 = 1859./4104.;
    let b65 = -11./40.;

    let c1 = 25./216.;
    let c3 = 1408./2565.;
    let c4 = 2197./4104.;
    let c5 = -1./5.;

    let ch1 = 16./135.;
    let ch3 = 6656./12825.;
    let ch4 = 28561./56430.;
    let ch5 = -9./50.;
    let ch6 = 2./55.;

    let _ce1 = 1./360.;
    let _ce3 = -128./4275.;
    let _ce4 = -2197./75240.;
    let _ce5 = 1./50.;
    let _ce6 = 2./55.;

    while tk < tf {

        h = h.min(tf - tk);

        // let tk = time[time.len()-1];
        let yk = &y[y.len()-1];

        let k1 = h * f(tk, yk);
        let k2 = h * f(tk + a2*h, &(yk + b21 * &k1));
        let k3 = h * f(tk + a3*h, &(yk + b31 * &k1 + b32 * &k2));
        let k4 = h * f(tk + a4*h, &(yk + b41 * &k1 + b42 * &k2 + b43 * &k3));
        let k5 = h * f(tk + a5*h, &(yk + b51 * &k1 + b52 * &k2 + b53 * &k3 + b54 * &k4));
        let k6 = h * f(tk + a6*h, &(yk + b61 * &k1 + b62 * &k2 + b63 * &k3 + b64 * &k4 + b65 * &k5));

        // Fourth-order Runge-Kutta result
        let w1 = yk + c1*&k1 + c3*&k3 + c4*&k4 + c5*&k5;

        // Fifth-order Runge-Kutta result
        let w2 = yk + ch1*&k1 + ch3*&k3 + ch4*&k4 + ch5*&k5 + ch6*&k6;

        // Error tolerance
        let tol = atol + yk.apply_norm(&UniformNorm)*rtol;

        // L-infinity error norm
        let truncation_error = ((&w2 - &w1) / h).apply_norm(&UniformNorm);
        // let truncation_error = h * (ce1*&k1 + ce3*&k3 + ce4*&k4 + ce5*&k5 + ce6*&k6).apply_norm(&UniformNorm);

        // Optimal step size scale factor
        let s = 0.84 * (tol / truncation_error).powf(0.25);

        // If step size satisfies error tolerance, accept this value
        if truncation_error <= tol {

            tk += h;
            h = s*h;
            _it += 1;

            time.push(tk);
            y.push(w2);

            _k = 0;

        } else if _k == 0 { // Tolerance not met for first time in this step

            h = s*h;
            _k += 1;

        } else { // continue searching for a better step size

            h = h / 2.0;

        }
    }

    (time, y)

}


// Dormand-Prince Runge-Kutta method of orders 4 and 5
// "A family of embedded Runge-Kutta formulae"
// J.R.Dormand and P.J.Prince
pub fn RK45<F>(
    f: F,
    t0: f32,
    y0: DVector<f32>,
    tf: f32,
    step: f32,
    rtol: f32
) -> (Vec<f32>, Vec<DVector<f32>>)
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>,
{

    let iterations = (tf - t0) / step;

    let mut h = step;
    let mut time: Vec<f32> = Vec::with_capacity(iterations as usize);
    let mut y: Vec<DVector<f32>> = Vec::with_capacity(iterations as usize);
    let atol = 1E-10;

    time.push(t0);
    y.push(y0);

    let mut tk = t0;

    // inner-step iteration
    let mut _k = 0;

    // global iteration
    let mut _it = 1;

    let a2 = 1./5.;
    let a3 = 3./10.;
    let a4 = 4./5.;
    let a5 = 8./9.;
    let a6 = 1.;
    let a7 = 1.;

    let b21 = 1./5.;
    let b31 = 3./40.;
    let b32 = 9./40.;
    let b41 = 44./45.;
    let b42 = -56./15.;
    let b43 = 32./9.;
    let b51 = 19372./6561.;
    let b52 = -25360./2187.;
    let b53 = 64448./6561.;
    let b54 = -212./729.;
    let b61 = 9017./3168.;
    let b62 = -355./33.;
    let b63 = 46732./5247.;
    let b64 = 49./176.;
    let b65 = -5103./18656.;
    let b71 = 35./384.;
    let b73 = 500./1113.;
    let b74 = 125./192.;
    let b75 = -2187./6784.;
    let b76 = 11./84.;

    let c1 = 5179./57600.;
    let c3 = 7571./16695.;
    let c4 = 393./640.;
    let c5 = -92097./339200.;
    let c6 = 187./2100.;
    let c7 = 1./40.;

    let ch1 = 35./384.;
    let ch3 = 500./1113.;
    let ch4 = 125./192.;
    let ch5 = -2187./6784.;
    let ch6 = 11./84.;

    while tk < tf {

        h = h.min(tf - tk);

        // let tk = time[time.len()-1];
        let yk = &y[y.len()-1];

        let k1 = h * f(tk, yk);
        let k2 = h * f(tk + a2*h, &(yk + b21 * &k1));
        let k3 = h * f(tk + a3*h, &(yk + b31 * &k1 + b32 * &k2));
        let k4 = h * f(tk + a4*h, &(yk + b41 * &k1 + b42 * &k2 + b43 * &k3));
        let k5 = h * f(tk + a5*h, &(yk + b51 * &k1 + b52 * &k2 + b53 * &k3 + b54 * &k4));
        let k6 = h * f(tk + a6*h, &(yk + b61 * &k1 + b62 * &k2 + b63 * &k3 + b64 * &k4 + b65 * &k5));
        let k7 = h * f(tk + a7*h, &(yk + b71 * &k1 + b73 * &k3 + b74 * &k4 + b75 * &k5 + b76 * &k6));

        // Fourth-order Runge-Kutta result
        let w1 = yk + c1*&k1 + c3*&k3 + c4*&k4 + c5*&k5 + c6*&k6 + c7*&k7;

        // Fifth-order Runge-Kutta result
        let w2 = yk + ch1*&k1 + ch3*&k3 + ch4*&k4 + ch5*&k5 + ch6*&k6;

        // Error tolerance
        let tol = atol + yk.apply_norm(&UniformNorm)*rtol;

        // L-infinity error norm
        let truncation_error = ((&w2 - &w1) / h).apply_norm(&UniformNorm);

        // Optimal step size scale factor
        let s = 0.9 * (tol / truncation_error).powf(0.25);

        // If step size satisfies error tolerance, accept this value
        if truncation_error <= tol {

            tk += h;
            h = s*h;
            _it += 1;

            time.push(tk);
            y.push(w2);

            _k = 0;

        } else if _k == 0 { // Tolerance not met for first time in this step

            h = s*h;
            _k += 1;

        } else { // continue searching for a better step size

            h = h / 2.0;

        }
    }


    (time, y)

}




#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_RKF45() {

        use na::DMatrix;
        use crate::controls::models::LinearQuadraticRegulator as LQR;
        use crate::dynamics::models::*;
        use crate::dynamics::statespace::StateSpaceRepresentation;

        let model = DoubleIntegrator3D::new();

        let Q = DMatrix::<f32>::identity(6, 6);
        let R = DMatrix::<f32>::identity(3, 3);

        let lqr = LQR::new(
            model.dynamics().A.clone(),
            model.dynamics().B.clone(),
            Q,
            R,
        );

        let (K, _P) = match lqr.solve() {
            Ok((value1, value2)) => (value1, value2),
            Err(_) => panic!["LQR solve"],
        };

        let y0 = DVector::from_vec(vec![10., 10., 10., 10., 10., 10.]);

        // Wrap dynamics/controls in appropriately defined closure
        let f = |t: f32, x: &DVector<f32>| {
            let u = -&K * x;
            model.dynamics().f(t, x, Some(&u))
        };

        let t0 = 0.0;
        let tf = 10.0;
        let n = 1000.0;
        let step = (tf - t0) / n;
        let rtol = 1E-5;
        let (_t, y) = RKF45(f, t0, y0, tf, step, rtol);

        for ele in y.iter() {
            println!("{:?}", ele.data);
        } }

    #[test]
    fn test_RK45() {

        use na::DMatrix;
        use crate::controls::models::LinearQuadraticRegulator as LQR;
        use crate::dynamics::models::*;
        use crate::dynamics::statespace::StateSpaceRepresentation;

        let model = DoubleIntegrator3D::new();

        let Q = DMatrix::<f32>::identity(6, 6);
        let R = DMatrix::<f32>::identity(3, 3);

        let lqr = LQR::new(
            model.dynamics().A.clone(),
            model.dynamics().B.clone(),
            Q,
            R,
        );

        let (K, _P) = match lqr.solve() {
            Ok((value1, value2)) => (value1, value2),
            Err(_) => panic!["LQR solve"],
        };

        let y0 = DVector::from_vec(vec![10., 10., 10., 10., 10., 10.]);

        // Wrap dynamics/controls in appropriately defined closure
        let f = |t: f32, x: &DVector<f32>| {
            let u = -&K * x;
            model.dynamics().f(t, x, Some(&u))
        };

        let t0 = 0.0;
        let tf = 10.0;
        let n = 1000.0;
        let step = (tf - t0) / n;
        let rtol = 1E-6;
        let (_t, y) = RK45(f, t0, y0, tf, step, rtol);

        for ele in y.iter() {
            println!("{:?}", ele.data);
        }
    }



}
