#![allow(non_snake_case)]
#[macro_use]

extern crate approx;
extern crate lapack;
extern crate lapack_src;
extern crate nalgebra as na;
extern crate serde;
extern crate rand;

// MADS Core
pub mod controls;
pub mod dynamics;
pub mod math;
pub mod log;
pub mod util;

// MADS Engine
pub mod simulator;
pub mod scene;
pub mod ecs;

pub mod prelude;
