
/// IVP Integrators
#[derive(Copy, Clone, PartialEq)]
pub enum IntegratorType {

    ForwardEuler,
    MidpointEuler,
    RK45,
    DOP583,

}

impl Default for IntegratorType {

    fn default() -> Self { IntegratorType::RK45 }

}
