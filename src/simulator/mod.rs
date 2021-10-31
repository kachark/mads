// Generate ECS World and default Resources
pub mod ecs;

// Define Simulator and Simulator states
pub mod configuration;
pub mod state;

use crate::simulator::state::{EngineState, SimulatorState};
use crate::scene::scenario::Scenario;

pub struct Simulator<T>
where
    T: Scenario
{

    state: SimulatorState,
    scenario: T

}

impl<T> Simulator<T>
where
    T: Scenario
{

    pub fn new(state: SimulatorState, scenario: T) -> Self {

        Self { state, scenario }

    }

    /// Builds Simulator using Simulator state ECS World and Resources
    pub fn build(&mut self) {

        // Add resources to simulation state and prepare scenario internal state
        self.scenario.setup(&mut self.state.ecs.world, &mut self.state.ecs.resources);

        // Define simulation state schedule
        self.state.ecs.schedule = self.scenario.build();

    }

    /// Simulator loop
    pub fn run(&mut self) {

        self.state.status = EngineState::Active;

        while self.state.status == EngineState::Active {

            // Execute ECS systems
            self.state.update();

            // update scenario or perform additional operations / queries on entities
            self.scenario.update(&mut self.state.ecs.world, &mut self.state.ecs.resources);

        }

    }

    pub fn get_state(&self) -> &SimulatorState {

        &self.state

    }

}

