
use mads::simulator::configuration::{EngineConfig, SimulatorConfig};
use mads::simulator::Simulator;
use mads::simulator::state::SimulatorState;
use mads::log::{SimpleLogger, Logger, LogDataType};
use mads::scene::scenario::SimpleScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulatorConfig::default();
    let sim_state = SimulatorState::new(engine_config, sim_config);
    let scenario = SimpleScenario::new();

    println!("Running...");
    let mut simulator = Simulator::new(sim_state, scenario);
    simulator.build();
    simulator.run();

    // Serialize results to csv
    println!("Logging...");
    let logger = SimpleLogger;
    if let Err(err) = logger.to_csv(&simulator.get_state(), "./simple_example.csv", LogDataType::SimResult) {
        println!("csv write error, {}", err);
    };


 }

