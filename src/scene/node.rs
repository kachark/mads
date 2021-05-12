

pub type NodeID = u32;
pub type Edge = (Node, Node); // undirected edge

#[derive(Clone, Hash, PartialEq, Eq)]
pub struct Node {

    pub uuid: NodeID

}

impl Node {

    pub fn init() -> Self {

        let uuid = 0;

        Self {
            uuid
        }

    }

}



