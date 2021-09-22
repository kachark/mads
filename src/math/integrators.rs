
use thiserror::Error;

#[derive(Error, Debug)]
pub enum IntegrateError {
    // max iterations
    //

}

/// IVP Integrators
#[derive(Copy, Clone, PartialEq)]
pub enum IntegratorType {

    ForwardEuler,
    MidpointEuler,
    RKF45,
    RK45,

}

impl Default for IntegratorType {

    fn default() -> Self { IntegratorType::RK45 }

}
