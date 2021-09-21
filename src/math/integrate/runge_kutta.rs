
use na::DVector;

// Reference: https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf
// Runge-Kutta-Fehlberg method aka RK45
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
    let mut h = step;
    let mut time: Vec<f32> = Vec::new();
    let mut y: Vec<DVector<f32>> = Vec::new();

    time.push(t0);
    y.push(y0);

    let mut tk = t0;
    let mut _k = 0;
    let mut count = 0;
    while tk < tf {

        // h = h.min(tf - time[time.len()-1]);
        h = h.min(tf - tk);

        // let tk = time[time.len()-1];
        let yk = &y[y.len()-1];

        let k1 = h * f(tk, yk);
        let k2 = h * f(tk + (1./4.)*h, &(yk + (1./4.) * &k1));
        let k3 = h * f(tk + (3./8.)*h, &(yk + (3./32.) * &k1 + (9./32.) * &k2));
        let k4 = h * f(tk + (12./13.)*h, &(yk + (1932./2197.) * &k1 - (7200./2197.) * &k2 + (7296./2197.) * &k3));
        let k5 = h * f(tk + h, &(yk + (439./216.) * &k1 - 8. * &k2 + (3680./513.) * &k3 - (845./4104.) * &k4));
        let k6 = h * f(tk + (1./2.)*h, &(yk - (8./27.) * &k1 + 2. * &k2 - (3544./2565.) * &k3 + (1859./4104.) * &k4 - (11./40.) * &k5));

        // Fourth-order Runge-Kutta result
        let w1 = yk + (25.*&k1/216.) + (1408.*&k3/2565.) + (2197.*&k4/4104.) - (&k5/5.);

        // Fifth-order Runge-Kutta result
        let w2 = yk + (16.*&k1/135.) + (6656.*&k3/12825.) + (28561.*&k4/56430.) - (9.*&k5/50.) + (2.*&k6/55.);

        let truncation_error: f32 = (&w2 - &w1).norm() / h;
        let s = 0.84 * (rtol / truncation_error).powf(0.25);

        // If step size satisfies error tolerance, accept this value
        if truncation_error <= rtol {

            tk += h;
            time.push(tk);
            y.push(w1);
            _k += 1;
            h = s*h;

        } else { // continue searching for a better step size

            h = s*h;
            count += 1;

            // TODO: better handling of endless step size adjustments
            if count > 10000 {

                break;

            }

        }
    }


    (time, y)

}


// Dormand-Prince Runge-Kutta method
pub fn DOP853<F>(
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
    let mut h = step;
    let mut time: Vec<f32> = Vec::new();
    let mut y: Vec<DVector<f32>> = Vec::new();

    time.push(t0);
    y.push(y0);

    let mut tk = t0;
    let mut _k = 0;
    let mut count = 0;
    while tk < tf {

        // h = h.min(tf - time[time.len()-1]);
        h = h.min(tf - tk);

        // let tk = time[time.len()-1];
        let yk = &y[y.len()-1];

        let k1 = h * f(tk, yk);
        let k2 = h * f(tk + (1./5.)*h, &(yk + (1./5.) * &k1));
        let k3 = h * f(tk + (3./10.)*h, &(yk + (3./40.) * &k1 + (9./40.) * &k2));
        let k4 = h * f(tk + (4./5.)*h, &(yk + (44./45.) * &k1 - (56./15.) * &k2 + (32./9.) * &k3));
        let k5 = h * f(tk + (8./9.)*h, &(yk + (19372./6561.) * &k1 - (25360./2187.) * &k2 + (64448./6561.) * &k3 - (212./729.) * &k4));
        let k6 = h * f(tk + h, &(yk + (9017./3168.) * &k1 - (355./33.) * &k2 - (46732./5247.) * &k3 + (49./176.) * &k4 - (5103./18656.) * &k5));
        let k7 = h * f(tk + h, &(yk + (35./384.) * &k1 + (500./1113.) * &k3 + (125./192.) * &k4 - (2187./6784.) * &k5 + (11./84.) * &k6));

        // Fourth-order Runge-Kutta result
        let w1 = yk + (5179.*&k1/57600.) + (7571.*&k3/16695.) + (393.*&k4/640.) - (92097.*&k5/339200.) + (187.*&k6/2100.) + (1.*&k7/40.);

        // Fifth-order Runge-Kutta result
        let w2 = yk + (35.*&k1/384.) + (500.*&k3/1113.) + (125.*&k4/192.) - (2187.*&k5/6784.) + (11.*&k6/84.);

        let truncation_error: f32 = (&w2 - &w1).norm();
        let s = 0.84 * (rtol / truncation_error).powf(0.20);

        // If step size satisfies error tolerance, accept this value
        if truncation_error <= rtol {

            tk += h;
            time.push(tk);
            y.push(w1);
            _k += 1;
            h = s*h;

        } else { // continue searching for a better step size

            h = s*h;
            count += 1;

            // TODO: better handling of endless step size adjustments
            if count > 10000 {

                break;

            }

        }
    }


    (time, y)

}




#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_RungeKutta45() {

        use na::DMatrix;
        use crate::controls::models::lqr::LinearQuadraticRegulator as LQR;
        use crate::dynamics::models::linear::double_integrator::*;
        use crate::dynamics::statespace::StateSpaceRepresentation;

        let model = DoubleIntegrator3D::new();

        let Q = DMatrix::<f32>::identity(6, 6);
        let R = DMatrix::<f32>::identity(3, 3);

        let lqr = LQR::new(
            model.dynamics.A.clone(),
            model.dynamics.B.clone(),
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
            model.dynamics.f(t, x, Some(&u))
        };

        let t0 = 0.0;
        let tf = 10.0;
        let n = 1000.0;
        let step = (tf - t0) / n;
        let rtol = 1E-5;
        let (_t, y) = RK45(f, t0, y0, tf, step, rtol);

        for ele in y.iter() {
            println!("{:?}", ele.data);
        }
    }

    #[test]
    fn test_DormandPrince() {

        use na::DMatrix;
        use crate::controls::models::lqr::LinearQuadraticRegulator as LQR;
        use crate::dynamics::models::linear::double_integrator::*;
        use crate::dynamics::statespace::StateSpaceRepresentation;

        let model = DoubleIntegrator3D::new();

        let Q = DMatrix::<f32>::identity(6, 6);
        let R = DMatrix::<f32>::identity(3, 3);

        let lqr = LQR::new(
            model.dynamics.A.clone(),
            model.dynamics.B.clone(),
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
            model.dynamics.f(t, x, Some(&u))
        };

        let t0 = 0.0;
        let tf = 10.0;
        let n = 1000.0;
        let step = (tf - t0) / n;
        let rtol = 1E-5;
        let (_t, y) = DOP853(f, t0, y0, tf, step, rtol);

        for ele in y.iter() {
            println!("{:?}", ele.data);
        }
    }



}
