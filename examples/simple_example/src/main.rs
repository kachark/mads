use formflight::simulator::configuration::{EngineConfig, SimulationConfig};
use formflight::simulator::simulation::Simulation;
use formflight::simulator::state::SimulationState;
use formflight::ecs::resources::*;
use formflight::log::simulation_logger::SimulationLogger;

use formflight::scene::scenario::SimpleScenario;

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

