
use crate::ecs::generational_index::*;

pub type Entity = GenerationalIndex;

// Alias to map Generational Index (Entity) to some type T
pub type EntityMap<T> = GenerationalIndexArray<T>;
