
use pyo3::prelude::*;
use pyo3::types::{PyList, PyTuple, IntoPyDict};
use numpy::{PyArray, PyArray2, IntoPyArray};
use ordered_float::OrderedFloat;


pub fn emd_assignment(agent_states: Vec<Vec<f32>>, target_states: Vec<Vec<f32>>) -> PyResult< Vec<Vec<u32>> > {

    // generate weights for the discrete distributions
    let a = vec![1f32 / (agent_states.len() as f32); agent_states.len()];
    let b = vec![1f32 / (target_states.len() as f32); target_states.len()];

    // Start a python instance
    Python::with_gil(|py| {

        // Import python modules
        let ot = PyModule::import(py, "ot")?;
        let np = PyModule::import(py, "numpy")?;
        let locals = [("np", np)].into_py_dict(py);

        // Cast rust 2d vector to numpy 2d array
        let xs = PyArray::from_vec2(py, &agent_states).unwrap();
        let xt = PyArray::from_vec2(py, &target_states).unwrap();

        let apy = PyArray::from_vec(py, a);
        let bpy = PyArray::from_vec(py, b);

        // Call OT module 'dist' function
        let args = (xs, xt);
        let mut M: Vec<Vec<f32>> = ot.call1("dist", args)?.extract()?;

        // get flattened vector of OrderedFloats which implement Ord
        let Mnicefloats: Vec<OrderedFloat<f32>> = M.clone().into_iter()
            .flatten()
            .map(|val| OrderedFloat(val))
            .collect();

        // get max value of M as an OrderedFloat
        let max_value = Mnicefloats.into_iter().max().unwrap();

        // divide M by it's max value
        for row in M.iter_mut() {
            for m_ij in row.iter_mut() {
                *m_ij /= max_value.0;
            }
        }

        // Call OT module 'emd' function
        let args = (apy, bpy, M, false);
        let G0: Vec<Vec<f32>> = ot.call1("emd", args)?.extract()?;


        // TODO: figure out a threshold
        let thresh = 1f32 / agent_states.len() as f32;

        // Return binary matrix corresponding to the assignment
        let mut assignment: Vec<Vec<u32>> = vec![vec![0; G0[0].len()]; G0.len()];
        for (i, row) in G0.iter().enumerate() {
            for (j, g_ij) in row.iter().enumerate() {
                if g_ij >= &thresh {
                    assignment[i][j] = 1;
                } else {
                    assignment[i][j] = 0;
                }
            }
        }

        Ok(assignment)
    })

}
