use mads::simulator::configuration::{EngineConfig, SimulationConfig};
use mads::simulator::simulation::Simulation;
use mads::simulator::state::SimulationState;
use mads::ecs::resources::*;
use mads::log::simulation_logger::SimulationLogger;

use mads::scene::scenario::SimpleScenario;

fn main() {

    // Configure simulation, engine, and scenario
    let engine_config = EngineConfig::default();
    let sim_config = SimulationConfig::default();
    let sim_state = SimulationState::new(engine_config, sim_config);
    let scenario = SimpleScenario::new();

    let mut simulation = Simulation::new(sim_state, scenario);
    simulation.build();
    simulation.run();

 }

