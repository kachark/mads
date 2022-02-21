
use mads::prelude::*;
use mads::log::{SimpleLogger, Logger, LogDataType};
use mads::scene::scenario::SimpleScenario;

fn main() {

    // Configure simulation and engine
    let sim_state = SimulatorState::new(
        EngineConfig::default(),
        SimulatorConfig::default()
    );

    // Build and run a scenario
    let mut simulator = Simulator::new(
        sim_state,
        SimpleScenario::new()
    );
    simulator.build();
    simulator.run();

    println!("Running...");

    // Serialize results to csv
    println!("Logging...");
    let logger = SimpleLogger;
    if let Err(err) = logger.to_csv(&simulator.get_state(), "./simple_example.csv", LogDataType::SimResult) {
        println!("csv write error, {}", err);
    };


 }

