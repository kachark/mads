
use std::collections::HashMap;

use crate::scene::scene::{Scene, SceneID};


pub struct SceneManager {

    scenes: HashMap<SceneID, Box<dyn Scene>>, // value is trait object that exists on the heap
    current_scene: Option<SceneID>,

}

impl SceneManager {

    fn new() -> Self {

        Self {

            scenes: HashMap::<SceneID, Box<dyn Scene>>::new(),
            current_scene: None

        }

    }

    fn process_input(&self) {

        match self.current_scene {
            Some(id) => {
                if let Some(scene) = self.scenes.get(&id) {
                    scene.process_input();
                } else {
                    ()
                }
            },
            None => ()
        };

    }


    fn update(&mut self, delta_time: f32) {

        match self.current_scene {
            Some(id) => {
                if let Some(scene) = self.scenes.get_mut(&id) {
                    scene.update();
                } else {
                    ()
                }
            },
            None => ()
        };

    }


    fn draw(&self) {}


    fn add(&self) {}


    fn switch(&self) {}


    fn remove(&self) {}

}
