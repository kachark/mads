
pub mod linear_scenario;
pub mod resources;

use mads::simulator::configuration::{EngineConfig, SimulationConfig};
use mads::simulator::simulation::Simulation;
use mads::simulator::state::SimulationState;

use crate::linear_scenario::LinearScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulationConfig::default();
    let sim_state = SimulationState::new(engine_config, sim_config);
    let scenario = LinearScenario::default();

    let mut simulation = Simulation::new(sim_state, scenario);
    simulation.build();
    simulation.run();

    // TODO: print out the results - maybe add a printSystem()?

 }
