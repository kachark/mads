
// What is an Entity Component System
// https://www.david-colson.com/2020/02/09/making-a-simple-ecs.html
// https://austinmorlan.com/posts/entity_component_system/
// https://kyren.github.io/2018/09/14/rustconf-talk.html

use crate::ecs::entity::*;
use crate::ecs::component::*;
use crate::ecs::generational_index::*;

// NOTE what we want to capture in the simulation
pub struct SimState {

    iterations: usize,
    total_agents: usize

}

pub struct SimRules {

    max_iterations: usize

}

// TODO this should be the Scene
pub struct Scene {

    // The scene cares about entities
    entities: Vec<Entity>,
    entity_allocator: GenerationalIndexAllocator,

    health_components: EntityMap<HealthComponent>,
    name_components: EntityMap<NameComponent>,
    uuid_components: EntityMap<UUIDComponent>,
    collision_components: EntityMap<CollisionComponent>,
    linear_dynamics_components: EntityMap<LinearDynamicsComponent>,
    // linear_controller_components: EntityMap<LinearControllerComponent>,
    fullstate_components: EntityMap<FullStateComponent>,
    color_components: EntityMap<ColorComponent>


}
