
use specs::{ReadStorage, System};

use crate::ecs::components::Position;

// Define Systems
pub struct PositionSystem;

impl<'a> System<'a> for PositionSystem {
    type SystemData = ReadStorage<'a, Position>;

    fn run(&mut self, position: Self::SystemData) {
        use specs::Join;

        for position in position.join() {
            println!("Hello, {:?}", &position);
        }
    }
}


