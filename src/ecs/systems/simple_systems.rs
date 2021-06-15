
use legion::*;

use crate::ecs::resources::*;
use crate::ecs::components::*;

#[system(for_each)]
pub fn update_position(pos: &mut Position, vel: &Velocity, #[resource] time: &SimulationTime) {
    pos.x += vel.x * time.0;
    pos.y += vel.y * time.0;

    println!("{:?}", pos);
}

// TODO: Agent component is scenario-specific
// #[system(for_each)]
// pub fn print_errorstate(_agent: &Agent, state1: &FullState, #[resource] targetable_set: &TargetableSet) {

//     for (_id, state2) in targetable_set.0.iter() {
//         println!("ERROR STATE: {:?}", &state1.0  - &state2.0);
//     }

// }

#[system(for_each)]
pub fn print_id(id: &SimID) {

    println!("{:?}", id);

}

#[system(for_each)]
pub fn print_state(state: &FullState) {

    println!("{:?}", state);

}

#[system]
pub fn increment_time(#[resource] time: &mut SimulationTime, #[resource] step: &EngineStep) {

    println!("{:?}", time.0);
    time.0 += step.0;

}
