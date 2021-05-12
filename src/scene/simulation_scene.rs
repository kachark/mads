

use crate::scene::scene::Scene;
use crate::scene::scene_graph::SceneGraph;
use crate::scene::node::Node;

pub struct SimulationScene {

    id: u32, // TODO SceneID
    root: Node, // TODO NodeID
    graph: SceneGraph

}

impl SimulationScene {

    fn new() -> Self {

        Self {

            id: 0,
            root: Node::init(),
            graph: SceneGraph::new()

        }

    }

}

impl Scene for SimulationScene {

    // called when scene initially created
    fn on_create(&self) {} 
    // called when scene destroyed
    fn on_destroy(&self) {}
    // called whenever a scene is transitioned into
    fn on_activate(&self) {}
    // called whenever a transition out of a scene occurs
    fn on_deactivate(&self) {}
    fn process_input(&self) {}
    fn update(&mut self) {}


}
