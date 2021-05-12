

pub type SceneID = i32;

pub trait Scene {

    // called when scene initially created
    fn on_create(&self);
    // called when scene destroyed
    fn on_destroy(&self);
    // called whenever a scene is transitioned into
    fn on_activate(&self);
    // called whenever a transition out of a scene occurs
    fn on_deactivate(&self);
    fn process_input(&self);
    fn update(&mut self);

}

