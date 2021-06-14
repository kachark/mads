
use legion::*;

use crate::ecs::resources::*;
use crate::ecs::components::*;

#[system(for_each)]
pub fn update_position(pos: &mut Position, vel: &Velocity, #[resource] time: &SimulationTime) {
    pos.x += vel.x * time.0;
    pos.y += vel.y * time.0;

    println!("{:?}", pos);
}

// #[system(for_each)]
// pub fn assignment(

#[system(for_each)]
pub fn print_errorstate(agent: &Agent, state1: &FullState, #[resource] targetable_set: &TargetableSet) {

    // TODO: need each entity to see the list of available targets - make this a resource!

    for (id, state2) in targetable_set.0.iter() {
        println!("ERROR STATE: {:?}", &state1.0  - &state2.0);
    }

}

#[system(for_each)]
pub fn print_id(id: &SimID) {

    println!("{:?}", id);

}

#[system(for_each)]
pub fn print_state(state: &FullState) {

    println!("{:?}", state);

}
