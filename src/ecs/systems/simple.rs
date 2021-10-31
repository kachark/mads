
// system macro flags a function as a Legion ECS system and appends "_system" to the handle

use legion::*;
use crate::ecs::resources::*;
use crate::ecs::components::*;

#[system(for_each)]
pub fn print_id(id: &SimID) {

    println!("{:?}", id);

}

#[system(for_each)]
pub fn print_state(state: &FullState) {

    println!("{:?}", state);

}

#[system]
pub fn print_time(#[resource] time: &SimulationTime) {
    println!("{:?}", time.0);
}

#[system]
pub fn increment_time(
    #[resource] time_history: &mut SimulationTimeHistory,
    #[resource] time: &mut SimulationTime,
    #[resource] step: &EngineStep
)
{

    time.0 += step.0;
    time_history.data.push(time.0);

}

#[system(for_each)]
pub fn update_result(id: &SimID, state: &FullState, #[resource] storage: &mut SimulationResult) {

    // Access SimulationResult data and match against uuid keys
    match storage.data.entry(id.clone()) {
        // No previous entry for uuid: insert new Vector of DVector<f32>
        std::collections::hash_map::Entry::Vacant(e) => {
            e.insert(vec![state.clone()]);
        },
        // Previous entry for uuid: push DVector<f32> to existing Vector
        std::collections::hash_map::Entry::Occupied(mut e) => {
            e.get_mut().push(state.clone());
        }
    }

}
