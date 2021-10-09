
use std::error::Error;

use crate::simulator::state::SimulatorState;

pub trait SimulationLogger {

    fn to_csv(&self, sim_state: &SimulatorState) -> Result<(), Box<dyn Error>>;

}

