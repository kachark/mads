
use std::collections::HashMap;
use legion::{World, Resources, Schedule};

use crate::ecs::resources::{SimulationResult, SimulationTimeHistory};
use crate::ecs::systems::simple_systems::*;

// NOTE:
// this struct is a f(SimulationState)

pub trait Scenario {

    fn setup(&self, world: &mut World, resources: &mut Resources);
    fn build(&self) -> Schedule;
    fn update(&mut self, world: &mut World, resources: &mut Resources);

}

pub struct SimpleScenario {

    name: String,
    time: f32

}

impl SimpleScenario {

    pub fn new() -> Self {

        let name = "SimpleScenario".to_string();

        Self { time: 0.0, name }

    }
}

impl Scenario for SimpleScenario {

    fn setup(&self, world: &mut World, resources: &mut Resources) {

        let storage = SimulationResult{ data: HashMap::new() };
        resources.insert(storage);

    }

    // Setup scenario systems
    fn build(&self) -> Schedule {

        let schedule = Schedule::builder()
            .add_system(increment_time_system())
            .build();

        schedule

    }

    fn update(&mut self, world: &mut World, resources: &mut Resources) {

        ()

    }

}
