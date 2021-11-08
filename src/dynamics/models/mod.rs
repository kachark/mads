mod linear;
mod nonlinear;

pub mod closed_form_solution {
    pub use crate::dynamics::models::nonlinear::clohessy_wiltshire::ClohessyWiltshireSolution;
}

pub use self::linear::double_integrator::{DoubleIntegrator1D, DoubleIntegrator2D, DoubleIntegrator3D}; 
pub use self::linear::inverted_pendulum::InvertedPendulum;
pub use self::nonlinear::double_pendulum::DoublePendulum;
pub use self::nonlinear::inverted_pendulum::InvertedPendulum as NonlinearInvertedPendulum;
pub use self::nonlinear::clohessy_wiltshire::ClohessyWiltshire;
