
use formflight::math::integrators::IntegratorType;

pub enum EngineParameter {

    SimulationTime(f32),
    MaxSimulationTime(f32),
    EngineStep(f32)

}

pub enum SimulationParameter {

    IntegratorStep(f32),
    Integrator(IntegratorType)

}

// pub enum SimulationOutputs {

//     SimulationTimeHistory,
//     SimulationResult

// }

// TODO: move to scenario.rs
pub enum ScenarioParameter {

    NumAgents(i32),
    NumTargets(i32)

}
