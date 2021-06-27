
use std::collections::HashMap;
use nalgebra::{DMatrix, DVector};
use legion::*;
use uuid::Uuid;

use formflight::scene::scenario::Scenario;
use formflight::ecs::systems::simple_systems::*;
use formflight::ecs::systems::dynamics_systems::*;
use formflight::ecs::components::*;
use formflight::ecs::resources::*;
use formflight::dynamics::statespace::{State, StateSpace};
use formflight::dynamics::models::linear::double_integrator::*;
use formflight::controls::models::lqr::LinearQuadraticRegulator;

use crate::scenarios::tracking::components::{Agent, Target};
use crate::scenarios::tracking::resources::{NumAgents, NumTargets};
use crate::distributions::*;

pub struct TrackingScenario {

    pub num_agents: u32,
    pub num_targets: u32,
    pub agent_formation: Distribution,
    pub target_formation: Distribution

}

impl TrackingScenario {

    pub fn new(num_agents: u32, num_targets:u32) -> Self {

        let agent_formation = Distribution::Sphere;
        let target_formation = Distribution::Circle3D;

        Self {
            num_agents,
            num_targets,
            agent_formation,
            target_formation
        }

    }

    pub fn default() -> Self {

        let agent_formation = Distribution::Sphere;
        let target_formation = Distribution::Circle3D;

        Self {
            num_agents: 5,
            num_targets: 5,
            agent_formation,
            target_formation
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

    fn setup_agents(&self, world: &mut World, resources: &mut Resources) {

        let radius = 10f32;
        let mut storage = resources.get_mut::<SimulationResult>().unwrap();

        // Generate initial states
        let formation = match self.agent_formation {
            Distribution::Circle2D => circle_3d(radius, self.num_agents), // force 3d scenario
            Distribution::Circle3D => circle_3d(radius, self.num_agents),
            Distribution::Sphere => sphere(radius, self.num_agents)
        };

        // For now just use a double integrator and LQR
        let double_integrator = DoubleIntegrator3D::new();
        let A = double_integrator.dynamics.A.clone();
        let B = double_integrator.dynamics.B.clone();
        let Q = DMatrix::<f32>::identity(6, 6);
        let R = DMatrix::<f32>::identity(3, 3);

        // Define agent entities
        let agents: Vec<(FullState, DynamicsModel::<DoubleIntegrator3D>, LQRController, SimID, Agent)> = (0..self.num_agents).into_iter()
            .zip(formation.iter())
            .map(| (i, pose) | -> (FullState, DynamicsModel::<DoubleIntegrator3D>, LQRController, SimID, Agent) {

                let name = "Agent".to_string() + &i.to_string();
                let id = Uuid::new_v4();
                let sim_id = SimID { uuid: id, name };
                let state = DVector::<f32>::from_vec(vec![pose.0, pose.1, pose.2, 0.0, 0.0, 0.0]);
                let statespace = StateSpace{
                    position: State::Position{x: 0, y: 1, z: Some(2)},
                    velocity: State::Velocity{x: 3, y: 4, z: Some(5)},
                    attitude: State::Empty,
                    angular_velocity: State::Empty
                };
                let fullstate = FullState { data: state, statespace };
                let dynamics = DynamicsModel { model: DoubleIntegrator3D::new() };
                let controller = LQRController { model: LinearQuadraticRegulator::new(A.clone(), B.clone(), Q.clone(), R.clone()) };
                let agent_flag = Agent { 0: true };

                (fullstate, dynamics, controller, sim_id, agent_flag)
            })
            .collect();

        // Add agents to storage resource
        for agent in agents.iter() {
            storage.data.entry(agent.3.clone()).or_insert(vec![agent.0.data.clone()]);
        }

        world.extend(agents);

    }

    fn setup_targets(&self, world: &mut World, resources: &mut Resources) {

        let radius = 10f32;
        let mut storage = resources.get_mut::<SimulationResult>().unwrap();
        let mut targetable_set = resources.get_mut::<TargetableSet>().unwrap();

        // Generate initial states
        let formation = match self.agent_formation {
            Distribution::Circle2D => circle_3d(radius, self.num_agents), // force 3d scenario
            Distribution::Circle3D => circle_3d(radius, self.num_agents),
            Distribution::Sphere => sphere(radius, self.num_agents)
        };

        // For now just use a double integrator and LQR
        let double_integrator = DoubleIntegrator3D::new();
        let A = double_integrator.dynamics.A.clone();
        let B = double_integrator.dynamics.B.clone();
        let Q = DMatrix::<f32>::identity(6, 6);
        let R = DMatrix::<f32>::identity(3, 3);

        // Define target entities
        let targets: Vec<(FullState, DynamicsModel::<DoubleIntegrator3D>, LQRController, SimID, Target)>
            = (0..self.num_targets).into_iter()
            .zip(formation.iter())
            .map(| (i, pose) | -> (FullState, DynamicsModel::<DoubleIntegrator3D>, LQRController, SimID, Target) {

                let name = "Target".to_string() + &i.to_string();
                let id = Uuid::new_v4();
                let sim_id = SimID { uuid: id, name };
                let state = DVector::<f32>::from_vec(vec![pose.0, pose.1, pose.2, 0.0, 0.0, 0.0]);
                let statespace = StateSpace{
                    position: State::Position{x: 0, y: 1, z: Some(2)},
                    velocity: State::Velocity{x: 3, y: 4, z: Some(5)},
                    attitude: State::Empty,
                    angular_velocity: State::Empty
                };
                let fullstate = FullState { data: state, statespace };
                let dynamics = DynamicsModel { model: DoubleIntegrator3D::new() };
                let controller = LQRController { model: LinearQuadraticRegulator::new(A.clone(), B.clone(), Q.clone(), R.clone()) };
                let target_flag = Target { 0: true };

                (fullstate, dynamics, controller, sim_id, target_flag)

            })
            .collect();

        // Add targets to storage resource
        // Add targets to targetable set resource
        for target in targets.iter() {
            storage.data.entry(target.3.clone()).or_insert(vec![target.0.data.clone()]);
            targetable_set.0.entry(target.3.uuid.clone()).or_insert(target.0.clone());
        }

        // NOTE: have to insert targetable_set afterwards since it's used in setup
        // resources.insert(TargetableSet(targetable_set.0.clone()));


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
        let targetable_set = TargetableSet(HashMap::new());
        let storage = SimulationResult{ data: HashMap::new() };
        resources.insert(num_agents);
        resources.insert(num_targets);
        resources.insert(storage);
        resources.insert(targetable_set);

        self.setup_agents(world, resources);
        self.setup_targets(world, resources);

    }

    /// Define systems to be run per loop iteration and return a Schedule to execute
    fn build(&self) -> Schedule {

        let schedule = Schedule::builder()
            // .add_system(print_id_system())
            // .add_system(print_errorstate_system()) // TODO: make this generic over two FullStates
            // .add_system(print_state_system())
            .add_system(dynamics_lqr_solver_system::<DoubleIntegrator3D>()) // can add any dynamics type here
            .add_system(update_result_system())
            .add_system(increment_time_system())
            .build();

        schedule

    }

    /// Update and perform necessary logic specific to the scenario
    fn update(&mut self, world: &mut World, resources: &mut Resources) {

        // Updates entities flagged as Targetable
        self.update_targetable_set(world, resources);

        // Perform assignment of Agents to Targets
        // self.assign(world, resources);

    }

}


