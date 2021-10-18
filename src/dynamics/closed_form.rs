
use na::DVector;

/// Defines an interface for solving closed-form equations
pub trait ClosedFormRepresentation {
    fn rhs(&self, t: f32, x: &DVector<f32>) -> DVector<f32>;
}


/// Captures a generic expression f(t, x)
pub struct NonlinearExpression<F>
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>
{

    pub expression: F

}

impl<F> NonlinearExpression<F>
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>
{

    pub fn new(f: F) -> Self {

        Self { expression: f }

    }

}

impl<F> ClosedFormRepresentation for NonlinearExpression<F>
where
    F: Fn(f32, &DVector<f32>) -> DVector<f32>
{

    fn rhs(&self, t: f32, x: &DVector<f32>) -> DVector<f32> {

        (self.expression)(t, x)

    }

}

#[cfg(test)]
mod tests {

    use crate::dynamics::models::nonlinear::clohessy_wiltshire::ClohessyWiltshireSolution;
    use na::DVector;

    use super::NonlinearExpression;
    use super::ClosedFormRepresentation;

    #[test]
    fn test_NonlinearExpression() {

        let closed_form = NonlinearExpression::new(ClohessyWiltshireSolution);

        let dt = 20.0;
        let x0 = DVector::from_vec(vec![0., 0., 0., 0.1, 0., 0.]);
        let mut traj = Vec::new();
        for i in 0..100 {

            let t = dt*i as f32;
            let test = closed_form.rhs(t, &x0);

            traj.push(test);

        }

        for ele in traj.iter() {
            println!("{:?}", ele.data);
        }



    }

}



