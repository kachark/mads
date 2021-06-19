

pub struct ScenarioConfig {

    pub num_agents: u32,
    pub num_targets: u32

}

impl ScenarioConfig {

    pub fn new(num_agents: u32, num_targets:u32) -> Self {

        Self {
            num_agents,
            num_targets
        }

    }



    pub fn default() -> Self {

        Self {
            num_agents: 5,
            num_targets: 5
        }

    }

}


