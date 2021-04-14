
extern crate nalgebra as na;

pub mod dynamics;
pub mod controls;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
