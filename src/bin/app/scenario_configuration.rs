
use std::collections::HashMap;
use nalgebra::{DMatrix, DVector};
use legion::*;
use uuid::Uuid;

use formflight::scenario::Scenario;
use formflight::ecs::systems::simple_systems::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::components::*;
use formflight::ecs::resources::*;
use formflight::dynamics::statespace::{State, StateSpace};
use formflight::dynamics::models::linear::double_integrator::*;
use formflight::dynamics::models::linear::inverted_pendulum::*;
use formflight::controls::models::lqr::LinearQuadraticRegulator;

use crate::scenario_components::{Agent, Target};
use crate::scenario_resources::{NumAgents, NumTargets};

pub struct TrackingScenario {

    pub num_agents: u32,
    pub num_targets: u32

}

impl TrackingScenario {

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

    /// Keeps track of Entities that have a Trackable component
    fn update_targetable_set(&self, world: &mut World, resources: &mut Resources) {

        // query for 'Targetable' components
        let mut targetable_set_atomic = resources.get_mut::<TargetableSet>().unwrap();
        let mut query = <(&SimID, &FullState, &mut Targetable)>::query();
        for mut chunk in query.iter_chunks_mut(world) {
            // we can access information about the archetype (shape/component layout) of the entities
            println!(
                "the entities in the chunk have {:?} components",
                chunk.archetype().layout().component_types(),
            );

            // we can iterate through a tuple of component references per entity
            for (id, state, targetable) in chunk {
                if targetable.0 == true {
                    continue
                } else {
                    targetable_set_atomic.0.entry(id.uuid).or_insert(state.clone());
                }
            }
        }

    }

}

impl Scenario for TrackingScenario {

    /// Generate Resources for the scenario and insert into Simulator Resource pool
    fn setup(&self,
        world: &mut World,
        resources: &mut Resources,
    )
    {
        // scenario resources
        let num_agents = NumAgents(self.num_agents);
        let num_targets = NumTargets(self.num_targets);
        let mut targetable_set = TargetableSet(HashMap::new());
        let mut storage = SimulationResult{ data: HashMap::new() };

        // TODO: setup_agents()
        // NOTE: define components for each entity
        // TODO: Hardcoded
        let double_integrator = DoubleIntegrator3D::new();
        let A1 = double_integrator.dynamics.A.clone();
        let B1 = double_integrator.dynamics.B.clone();
        let Q1 = DMatrix::<f32>::identity(6, 6);
        let R1 = DMatrix::<f32>::identity(3, 3);

        // TODO: Hardcoded
        let inverted_pendulum = InvertedPendulum::new();
        let A2 = inverted_pendulum.dynamics.A.clone();
        let B2 = inverted_pendulum.dynamics.B.clone();
        let Q2 = DMatrix::<f32>::identity(4, 4);
        let R2 = DMatrix::<f32>::identity(1, 1);

        // TODO: Hardcoded
        let di_2d = DoubleIntegrator2D::new();
        let A3 = di_2d.dynamics.A.clone();
        let B3 = di_2d.dynamics.B.clone();
        let Q3 = DMatrix::<f32>::identity(4, 4);
        let R3 = DMatrix::<f32>::identity(2, 2);

        // NOTE: we can easily do heterogeneous swarms!!!
        for _ in 0..num_agents.0 {
            let id = Uuid::new_v4();
            let name = "test_name".to_string();
            let sim_id = SimID { uuid: id, name };
            let state = DVector::<f32>::from_vec(vec![0.0, 5.0, 10.0, 11.0, 2., 3.]);
            let statespace = StateSpace{
                position: State::Position{x: 0, y: 1, z: Some(2)},
                velocity: State::Velocity{x: 3, y: 4, z: Some(5)},
                attitude: State::Empty,
                angular_velocity: State::Empty
            };

            // initialize the SimulationResult storage
            storage.data.entry(sim_id.clone()).or_insert(vec![state.clone()]);

            world.extend(vec![
                (FullState { data: state, statespace },
                    DynamicsModel { model: DoubleIntegrator3D::new() },
                    LQRController { model: LinearQuadraticRegulator::new(A1.clone(), B1.clone(), Q1.clone(), R1.clone()) },
                    sim_id,
                    Agent { 0: true } )
            ]);

            // NOTE: generic dynamics component so we don't have to make many dynamics components per
            // model
            let id2 = Uuid::new_v4();
            let sim_id2 = SimID { uuid: id2, name: "generic_component".to_string() };
            let state2 = DVector::<f32>::from_vec(vec![0.0, 0.0, 1.0, 1.0]);
            let statespace2 = StateSpace{
                position: State::Position{x: 0, y: 1, z: None},
                velocity: State::Velocity{x: 2, y: 3, z: None},
                attitude: State::Empty,
                angular_velocity: State::Empty
            };

            storage.data.entry(sim_id2.clone()).or_insert(vec![state2.clone()]);

            world.extend(vec![
                (FullState { data: state2, statespace: statespace2 },
                    DynamicsModel { model: DoubleIntegrator2D::new() },
                    LQRController { model: LinearQuadraticRegulator::new(A3.clone(), B3.clone(), Q3.clone(), R3.clone()) },
                    sim_id2,
                    Agent { 0: true } )
            ]);

            let id3 = Uuid::new_v4();
            let sim_id3 = SimID { uuid: id3, name: "generic_component_2".to_string() };
            let state3 = DVector::<f32>::from_vec(vec![5.0, 5.0, 5.0, 5.0]);
            let statespace3 = StateSpace{
                position: State::Position{x: 0, y: 1, z: None},
                velocity: State::Velocity{x: 2, y: 3, z: None},
                attitude: State::Empty,
                angular_velocity: State::Empty
            };

            storage.data.entry(sim_id3.clone()).or_insert(vec![state3.clone()]);

            world.extend(vec![
                (FullState { data: state3, statespace: statespace3 },
                    DynamicsModel { model: DoubleIntegrator2D::new() },
                    LQRController { model: LinearQuadraticRegulator::new(A3.clone(), B3.clone(), Q3.clone(), R3.clone()) },
                    sim_id3,
                    Agent { 0: true } )
            ]);


        }

        // TODO: setup_targets()
        for _ in 0..num_targets.0 {
            let id = Uuid::new_v4();
            let name = "target_1".to_string();
            let sim_id = SimID { uuid: id, name };
            let statespace = StateSpace{
                position: State::Position{x: 0, y: 1, z: None},
                velocity: State::Velocity{x: 2, y: 3, z: None},
                attitude: State::Empty,
                angular_velocity: State::Empty
            };

            let state = FullState { 
                data: DVector::<f32>::from_vec(vec![0.0, 5.0, 10.0, 11.0, 2., 3.]),
                statespace
            };

            storage.data.entry(sim_id).or_insert(vec![state.data.clone()]);

            targetable_set.0.entry(id).or_insert(state.clone());
            world.extend(vec![
                (state,
                    Target { 0: true },
                    Targetable { 0: true })
            ]);
        }

        // NOTE: have to insert targetable_set afterwards since it's used in setup
        resources.insert(num_agents);
        resources.insert(num_targets);
        resources.insert(TargetableSet(targetable_set.0.clone()));


        // NOTE: TEST also have to insert this afterwards
        resources.insert(storage);

    }

    /// Define systems to be run per loop iteration and return a Schedule to execute
    fn build(&self) -> Schedule {

        let schedule = Schedule::builder()
            // .add_system(print_id_system())
            // .add_system(print_errorstate_system()) // TODO: make this generic over two FullStates
            // .add_system(print_state_system())
            .add_system(dynamics_lqr_solver_system::<DoubleIntegrator2D>())
            .add_system(dynamics_lqr_solver_system::<DoubleIntegrator3D>())
            .add_system(update_result_system())
            .add_system(increment_time_system())
            .build();

        schedule

    }

    /// Update and perform necessary logic specific to the scenario
    fn update(&mut self, world: &mut World, resources: &mut Resources) {

        // Updates entities flagged as Targetable
        self.update_targetable_set(world, resources);

    }

}


