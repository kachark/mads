
use std::error::Error;
use thiserror::Error;
use crate::ecs::resources::{SimulationResult, SimulationTimeHistory};
use crate::simulator::state::SimulatorState;

#[derive(Error, Debug)]
pub enum LogError {
    #[error("{0} data not found in resources")]
    MissingDataError(String),
}


/// Generic interface for saving Simulator state
pub trait Logger {

    fn to_csv(&self, sim_state: &SimulatorState, filepath: &str) -> Result<(), Box<dyn Error>> {

        let mut wtr = csv::Writer::from_path(filepath)?;

        // Access time and entity FullStates
        let time_history = match sim_state.resources.get::<SimulationTimeHistory>() {
            Some(time_resource) => time_resource,
            None => return Err(Box::new(LogError::MissingDataError("Time history".to_string())))
        };

        let results = match sim_state.resources.get::<SimulationResult>() {
            Some(result_resource) => result_resource,
            None => return Err(Box::new(LogError::MissingDataError("Result".to_string())))
        };

        // Construct header
        let mut header: Vec<String> = vec!["Time".to_string()];
        for (id, fullstate_history) in results.data.iter() {

            let dx = fullstate_history[0].data.len();
            for _ in 0..dx {
                header.push(id.name.clone());
            }

        }

        wtr.write_record(&header)?;

        // Go row-by-row and serialize time and header aligned state values
        for (k, time) in time_history.data.iter().enumerate() {

            let mut row: Vec<f32> = vec![*time];
            for (_id, state_history) in results.data.iter() {
                let fullstate_k = &state_history[k];
                for dim in fullstate_k.data.iter() {
                    row.push(*dim);
                }
            }

            wtr.serialize(row)?;

        }

        wtr.flush()?;

        Ok(())

    }



}


/// Simple Logging struct for serializing time history and sim results into csv
pub struct SimpleLogger;

impl Logger for SimpleLogger {

    // Use default implementation of to_csv()

}

#[cfg(test)]
mod tests {

    use std::fs::{remove_file, File};
    use std::io::Read;
    use std::path::Path;

    use crate::scene::scenario::SimpleScenario;
    use crate::simulator::Simulator;
    use crate::simulator::configuration::{EngineConfig, SimulatorConfig};

    use super::*;

    #[test]
    fn test_SimpleLogger_to_csv() {

        let filepath = "./test_SimpleLogger_to_csv.csv";

        // Simple configuration for simulator
        let engine_config = EngineConfig::default();
        let sim_config = SimulatorConfig::default();
        let sim_state = SimulatorState::new(engine_config, sim_config);

        // Build a simulator, don't run
        let scenario = SimpleScenario::new();
        let mut simulator = Simulator::new(sim_state, scenario);
        simulator.build();

        // test.csv should have a time value of 0.0 because the sim wasn't run
        let logger = SimpleLogger;
        if let Err(err) = logger.to_csv(&simulator.state, filepath) {
            panic!("test_SimpleLogger to_csv() test failed: {}", err);
        };

        // confirm new csv
        let path = Path::new(filepath);
        let display = path.display();
        let mut file = match File::open(&path) {
            Err(why) => panic!("couldn't read {}: {}", display, why),
            Ok(file) => file,
        };

        let mut s = String::new();
        match file.read_to_string(&mut s) {
            Err(why) => panic!("couldn't read {}: {}", display, why),
            Ok(_) => print!("{} contains:\n{}", display, s),
        }

        // remove the test file
        match remove_file(filepath) {
            Err(why) => panic!("couldn't find {}: {}", display, why),
            Ok(_) => println!("done")
        };

    }

}


