
use legion::{World, Resources};
use formflight::configuration::*;

pub trait Scenario {

    fn setup(
        &self, 
        engine_params: &EngineConfig,
        sim_params: &SimulationConfig
    ) -> (World, Resources, Vec<f32>);

}
