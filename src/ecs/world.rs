
use crate::ecs::component::*;

struct World {
    health_components: Vec<Option<Health>>,
    fullstate_components: Vec<Option<FullState>>,
    linear_dynamics_components: Vec<Option<LinearDynamics>>,
    // TODO consider making controllers own ABQR instead of borrow
}

impl World {

    fn new() -> Self {

        Self {
            health_components: Vec::new(),
            fullstate_components: Vec::new(),
            linear_dynamics_components: Vec::new()
        }

    }

}

