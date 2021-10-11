
pub mod nonlinear_scenario;
pub mod resources;

use mads::simulator::configuration::{EngineConfig, SimulatorConfig};
use mads::simulator::simulator::Simulator;
use mads::simulator::state::SimulatorState;
use crate::nonlinear_scenario::NonlinearScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulatorConfig::default();
    let sim_state = SimulatorState::new(engine_config, sim_config);
    let scenario = NonlinearScenario::default();

    let mut simulator = Simulator::new(sim_state, scenario);
    simulator.build();
    simulator.run();

 }

