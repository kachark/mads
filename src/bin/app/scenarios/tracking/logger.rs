
use std::fs;
use std::io::BufWriter;
use std::error::Error;

use formflight::ecs::resources::{SimulationResult, SimulationTimeHistory};
use formflight::simulator::state::SimulationState;
use formflight::log::simulation_logger::SimulationLogger;

use crate::scenarios::tracking::resources::AssignmentHistory;

pub struct Logger;

impl Logger {

    pub fn assignments_to_csv(&self, sim_state: &SimulationState) -> serde_json::Result<()> {

        let f = fs::File::create("assignments.json").expect("Unable to create file");
        let bw = BufWriter::new(f);

        // Access time and stored assignments
        // let time_history = sim_state.resources.get::<SimulationTimeHistory>().unwrap();
        let assignments = sim_state.resources.get_mut::<AssignmentHistory>().unwrap();

        // Serialize hashmap of assignment history to JSON
        let j = serde_json::to_string_pretty(&assignments.map)?;

        // println!("{}", j);

        serde_json::to_writer(bw, &j).expect("Failed writing : (");

        Ok(())

    }

}

impl SimulationLogger for Logger {

    fn to_csv(&self, sim_state: &SimulationState) -> std::result::Result<(), Box<dyn Error>> {

        let mut wtr = csv::Writer::from_path("test.csv")?;

        // Access stored time and entity FullStates
        let time_history = sim_state.resources.get::<SimulationTimeHistory>().unwrap();
        let results = sim_state.resources.get::<SimulationResult>().unwrap();

        println!("{:?}", time_history);

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


