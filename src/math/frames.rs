
use na::{Rotation3, Vector3, Matrix3};

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct ReferenceFrame {

    origin: Vector3<f32>,
    coordinates: Matrix3<f32>

}

impl ReferenceFrame {

    pub fn new(origin: (f32, f32, f32)) -> Self {

        let o = Vector3::new(origin.0, origin.1, origin.2);
        let mut coordinates = Matrix3::from_element(0.0);
        coordinates.fill_with_identity();

        Self { origin: o, coordinates }

    }

    /// Generates a ReferenceFrame at (0,0,0) with cartesian unit vectors +x, +y, +z
    pub fn default() -> Self {

        let origin = Vector3::new(0f32, 0f32, 0f32);
        let mut coordinates = Matrix3::from_element(0.0);
        coordinates.fill_with_identity();

        Self { origin, coordinates }

    }

    /// Rotates frame using euler angles rotations with a ZYX sequence
    pub fn rotate(&mut self, roll: f32, pitch: f32, yaw: f32) {

        let rot = Rotation3::from_euler_angles(roll, pitch, yaw);

        // self.axes = rot.transpose() * self.axes // transpose because nalgebra uses col-major matrices
        self.coordinates = rot * self.coordinates

    }

}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_ReferenceFrame_rotate() {

        let mut some_frame = ReferenceFrame::default();
        some_frame.rotate(std::f32::consts::FRAC_PI_4, 0.0, std::f32::consts::FRAC_PI_6);

        let correct = Matrix3::from_row_slice(
            &[1.0, 0.0, 0.0,
            0.0, 0.707, -0.707,
            0.0, 0.707, 0.707]);

        println!("{:?}", some_frame);

        let _ = relative_eq!(some_frame.coordinates, correct);
    }

}

