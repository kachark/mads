
pub mod my_scenario;

use mads::simulator::configuration::{EngineConfig, SimulatorConfig};
use mads::simulator::simulator::Simulator;
use mads::simulator::state::SimulatorState;
use crate::my_scenario::MyScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulatorConfig::default();
    let sim_state = SimulatorState::new(engine_config, sim_config);
    let scenario = MyScenario::new();

    let mut simulator = Simulator::new(sim_state, scenario);
    simulator.build();
    simulator.run();

 }

