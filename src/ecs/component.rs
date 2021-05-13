
use crate::dynamics::linear_system::linear_dynamics::LinearSystem;
use crate::dynamics::controls::lqr::LinearQuadraticRegulator;

// Miscellaneous
pub struct HealthComponent(i32);
pub struct NameComponent(String);
pub struct UUIDComponent(u32);

// Physics Components
pub struct FullStateComponent(Vec<f32>);
pub struct CollisionComponent();
pub struct LinearDynamicsComponent(LinearSystem);
pub struct LinearControllerComponent(LinearQuadraticRegulator); // TODO change to raw data, not borrows

// Renderable Components
pub struct ColorComponent(Vec<f32>);
