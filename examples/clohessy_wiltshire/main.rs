
pub mod my_scenario;

use mads::simulator::configuration::{EngineConfig, SimulatorConfig};
use mads::simulator::Simulator;
use mads::simulator::state::SimulatorState;
use mads::math::integrate::IntegratorType;
use mads::log::{Logger, LogDataType, SimpleLogger};

use crate::my_scenario::MyScenario;

fn main() {

    // Configure engine
    let start_time = 0.0;
    let max_time = 300.0;
    let engine_step = 0.5;
    let engine_config = EngineConfig::new(start_time, max_time, engine_step);

    // Configure simulator
    let integrator = IntegratorType::RK45;
    let integrator_step = 0.1;
    let sim_config = SimulatorConfig::new(integrator, integrator_step);

    // Initial simulator state
    let sim_state = SimulatorState::new(engine_config, sim_config);

    let scenario = MyScenario::new();

    let mut simulator = Simulator::new(sim_state, scenario);
    simulator.build();
    simulator.run();

    let logger = SimpleLogger;
    if let Err(err) = logger.to_csv(&simulator.state, "./my_scenario_results.csv", LogDataType::SimResult) {
        println!("csv write error, {}", err);
    };

 }

