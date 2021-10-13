
pub mod linear_scenario;
pub mod resources;

use mads::simulator::configuration::{EngineConfig, SimulatorConfig};
use mads::simulator::Simulator;
use mads::simulator::state::SimulatorState;
use crate::linear_scenario::LinearScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulatorConfig::default();
    let sim_state = SimulatorState::new(engine_config, sim_config);
    let scenario = LinearScenario::default();

    let mut simulator = Simulator::new(sim_state, scenario);
    simulator.build();
    simulator.run();

 }

