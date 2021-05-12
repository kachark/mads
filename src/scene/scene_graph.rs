
use std::collections::HashMap;

use serde_json::{Result, Value};
use crate::scene::node::{Node, NodeID, Edge};

pub struct SceneGraph {

    graph: HashMap<NodeID, Vec<Edge>>,
    nodes: HashMap<NodeID, Node> // TODO the scene graph should not own the nodes. why? Nodes == Entities. could just have the entity have a copy of NodeID

}

impl SceneGraph {

    pub fn new() -> Self {

        Self {

            graph: HashMap::<NodeID, Vec<Edge>>::new(),
            nodes: HashMap::<NodeID, Node>::new()

        }

    }

    pub fn add_node(&mut self) -> NodeID {

        // Generate new node
        let uuid: NodeID = 0;
        let node = Node {
            uuid
        };

        // add NodeId to graph and map to an empty edge, return copy of its node ID
        self.graph.entry(uuid).or_insert(Vec::<Edge>::new());
        self.nodes.entry(uuid).or_insert(node);

        return uuid

    }

    pub fn remove_node(&mut self, id: &NodeID) {

        self.graph.remove(id);

    }

    pub fn get(&self, id: &NodeID) -> Option<&Node> {

        self.nodes.get(id)

    }

    pub fn get_mut(&mut self, id: &NodeID) -> Option<&mut Node> {

        self.nodes.get_mut(id)

    }

    pub fn from_json(&mut self, data: &str) -> Result<()> {

        // Parse the string of data
        let v: Value = serde_json::from_str(data)?;

        println!("{}", v["1"]);

        Ok(())

    }

}


#[cfg(test)]

#[test]
fn test_SceneGraph_add_node() {

    let mut graph = SceneGraph::new();
    graph.add_node();

    assert_eq!(graph.graph.len(), 1);

}


#[test]
fn test_SceneGraph_remove_node() {

    let mut graph = SceneGraph::new();
    let node_id = graph.add_node();
    graph.remove_node(&node_id);

    println!("{:?}", node_id);

    assert_eq!(graph.graph.len(), 0);

}


#[test]
fn test_SceneGraph_from_json() {

    let mut graph = SceneGraph::new();

    // Some JSON input data as a &str. Maybe this comes from the user.
    let data = r#"
        {
            "1": {
                "parent": "root",
                "children": [
                    "2",
                    "3"
                ]
            },
            "4": {
                "parent": "root",
                "children": []
            }
        }"#;

    match graph.from_json(data) {
        Ok(_) => println!("ok"),
        Err(_) => println!("err")
    };

}

