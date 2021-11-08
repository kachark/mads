
use std::error::Error;
use std::fs;
use std::io::BufWriter;
use std::collections::HashMap;
use thiserror::Error;
use legion::*;
use crate::ecs::resources::{SimulationResult, SimulationTimeHistory};
use crate::ecs::components::*;
use crate::simulator::state::SimulatorState;
use crate::dynamics::statespace::Statespace;

#[derive(Error, Debug)]
pub enum LogError {
    #[error("Data not supported")]
    DataTypeError,
    #[error("{0} data not found in resources")]
    MissingDataError(String),
}

pub enum LogDataType {
    SimIds,
    SimTimeHistory,
    SimResult,
    SimStaticEntities,
    SimTargetEntities,
    SimWaypointEntities
}

pub enum LogFileType {
    CSV,
    JSON,
}


/// Generic interface for saving Simulator state
pub trait Logger {

    /// Saves MADS simulation data versus time as a csv
    /// Supported data:
    /// - SimulationTimeHistory
    /// - SimulationResult
    fn to_csv(&self, sim_state: &SimulatorState, filepath: &str, data_type: LogDataType) -> Result<(), Box<dyn Error>> {

        match data_type {
            LogDataType::SimTimeHistory => self.log_sim_time(sim_state, filepath, LogFileType::CSV),
            LogDataType::SimResult => self.log_sim_result(sim_state, filepath, LogFileType::CSV),
            _ => Err(Box::new(LogError::DataTypeError))
        }

    }

    fn to_json(&self, sim_state: &SimulatorState, filepath: &str, data_type: LogDataType) -> Result<(), Box<dyn Error>> {

        match data_type {
            LogDataType::SimIds => self.log_sim_id(sim_state, filepath, LogFileType::JSON),
            _ => Err(Box::new(LogError::DataTypeError))
        }

    }

    fn log_sim_id(&self, sim_state: &SimulatorState, filepath: &str, filetype: LogFileType) -> Result<(), Box<dyn Error>> {

        let f = fs::File::create(filepath).expect("Unable to create file");
        let bw = BufWriter::new(f);

        let mut entity_id_query = <(&SimID, &DynamicFlag, &StatespaceComponent)>::query();

        let mut entities = HashMap::<String, Vec<(&SimID, &StatespaceComponent)>>::new();
        // let mut entities = HashMap::<String, Vec<&SimID>>::new();

        for (id, _, statespace) in entity_id_query.iter(&sim_state.ecs.world) {
            entities.entry("Dynamics_entities".to_string()).or_insert(Vec::new()).push( (id, statespace) );
            // entities.entry("Entities".to_string()).or_insert(Vec::new()).push( &id );
        }

        // Serialize to JSON
        let entities_json = serde_json::to_string_pretty(&entities)?;

        // println!("{}", &entities_json);

        serde_json::to_writer(bw, &entities_json).expect("Failed writing : (");

        Ok(())


    }

    fn log_sim_time(&self, sim_state: &SimulatorState, filepath: &str, filetype: LogFileType) -> Result<(), Box<dyn Error>> {

        Ok(())

    }

    fn log_sim_result(&self, sim_state: &SimulatorState, filepath: &str, filetype: LogFileType) -> Result<(), Box<dyn Error>> {

        let mut wtr = csv::Writer::from_path(filepath)?;

        // Access time and entity FullStates
        let time_history = match sim_state.ecs.resources.get::<SimulationTimeHistory>() {
            Some(time_resource) => time_resource,
            None => return Err(Box::new(LogError::MissingDataError("Time history".to_string())))
        };

        let results = match sim_state.ecs.resources.get::<SimulationResult>() {
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
    use crate::ecs::systems::simulate::integrate_lqr_dynamics_system;
    use crate::ecs::systems::simple::*;

    use uuid::Uuid;
    use na::{DVector, DMatrix};
    use crate::scene::scenario::Scenario;
    use crate::dynamics::models::DoubleIntegrator3D;

    use super::*;

    struct TestScenario {

        name: String,
        time: f32

    }

    impl TestScenario {

        pub fn new() -> Self {

            let name = "TestScenario".to_string();

            Self { time: 0.0, name }

        }
    }

    impl Scenario for TestScenario {

        fn setup(&self, world: &mut World, resources: &mut Resources) {

            // Insert this Resource as a object to store Entity data
            let storage = SimulationResult{ data: HashMap::new() };
            resources.insert(storage);

            // Define an Entity by adding it's components
            let id = Uuid::new_v4();
            let sim_id = SimID { uuid: id, name: "Entity0".to_string() };

            // Initial conditions
            let state = DVector::<f32>::from_vec(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            let fullstate = FullState { data: state };

            // Agent dynamics model
            let dynamics_component = DoubleIntegrator3DComponent::new();
            let A = dynamics_component.dynamics().A.clone();
            let B = dynamics_component.dynamics().B.clone();

            // Agent controller
            let Q = DMatrix::<f32>::identity(6, 6);
            let R = DMatrix::<f32>::identity(3, 3);
            let controller_component = LQRComponent::new(A, B, Q, R);

            let dynamics_flag = DynamicFlag { 0: true };
            let statespace_component = dynamics_component.statespace().clone();

            let entity = (fullstate, dynamics_component, statespace_component, controller_component, sim_id, dynamics_flag);

            world.push(entity.clone());

            // add this entity to our storage resource
            resources.get_mut::<SimulationResult>().unwrap().data.entry(entity.4).or_insert(vec![entity.0.clone()]);

        }

        // Setup scenario systems
        fn build(&self) -> Schedule {

            let schedule = Schedule::builder()
                .add_system(print_time_system())
                .add_system(integrate_lqr_dynamics_system::<DoubleIntegrator3D>())
                .add_system(update_result_system())
                .add_system(increment_time_system())
                .build();

            schedule

        }

        fn update(&mut self, _world: &mut World, _resources: &mut Resources) {

            ()

        }

    }

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
        if let Err(err) = logger.to_csv(&simulator.get_state(), filepath, LogDataType::SimResult) {
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

    #[test]
    fn test_SimpleLogger_to_json() {

        let filepath = "./test_SimpleLogger_to_json.json";

        // Simple configuration for simulator
        let engine_config = EngineConfig::default();
        let sim_config = SimulatorConfig::default();
        let sim_state = SimulatorState::new(engine_config, sim_config);

        // Build a simulator, don't run
        let scenario = TestScenario::new();
        let mut simulator = Simulator::new(sim_state, scenario);
        simulator.build();

        // test.csv should have a time value of 0.0 because the sim wasn't run
        let logger = SimpleLogger;
        if let Err(err) = logger.to_json(&simulator.get_state(), filepath, LogDataType::SimIds) {
            panic!("test_SimpleLogger to_json() test failed: {}", err);
        };

        // confirm new json
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


