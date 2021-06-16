
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
pub fn increment_time(
    #[resource] time_history: &mut SimulationTimeHistory,
    #[resource] time: &mut SimulationTime,
    #[resource] step: &EngineStep
)
{

    // TODO: update resources to have actual field names, not zeros
    println!("{:?}", time.0);
    time.0 += step.0;
    time_history.data.push(time.0);

}

#[system(for_each)]
pub fn update_result(id: &SimID, state: &FullState, #[resource] storage: &mut SimulationResult) {

    // Access SimulationResult data and match against uuid keys
    match storage.data.entry(id.uuid) {
        // No previous entry for uuid: insert new Vector of DVector<f32>
        std::collections::hash_map::Entry::Vacant(e) => {
            e.insert(vec![state.0.clone()]);
        },
        // Previous entry for uuid: push DVector<f32> to existing Vector
        std::collections::hash_map::Entry::Occupied(mut e) => {
            e.get_mut().push(state.0.clone());
        }
    }

}
