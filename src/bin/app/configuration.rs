
pub enum IntegratorType {

    ForwardEuler,
    MidpointEuler,
    RK45

}

pub enum EngineParameter {

    SimulationTime(f32),
    MaxSimulationTime(f32),
    EngineStep(f32)

}

pub enum SimulationParameter {

    IntegratorStep,
    Integrator(IntegratorType)

}

pub enum ScenarioParameter {

    NumAgents(i32),
    NumTargets(i32)

}
