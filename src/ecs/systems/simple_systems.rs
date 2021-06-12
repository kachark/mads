
use legion::*;

use crate::ecs::components::{Position, Velocity};
use crate::ecs::resources::SimulationTime;

#[system(for_each)]
pub fn update_position(pos: &mut Position, vel: &Velocity, #[resource] time: &SimulationTime) {
    pos.x += vel.x * time.0;
    pos.y += vel.y * time.0;

    println!("{:?}", pos);
}


