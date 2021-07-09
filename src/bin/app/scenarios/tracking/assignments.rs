
use pyo3::prelude::*;
use pyo3::types::{PyList, PyTuple, IntoPyDict};
use numpy::{PyArray, PyArray2, IntoPyArray};
use ordered_float::OrderedFloat;

// TODO: optimize this a bit
// prefer to use PyArray objects since they trigger errors and tracing

pub fn emd_assignment(agent_states: &Vec<Vec<f32>>, target_states: &Vec<Vec<f32>>) -> PyResult< Vec<Vec<u32>> > {

    // Start a python instance and aquire python GIL
    Python::with_gil(|py| {

        // Import python modules
        let ot = PyModule::import(py, "ot")?;

        // Cast rust 2d vector to numpy 2d array
        let xs = PyArray::from_vec2(py, &agent_states).unwrap();
        let xt = PyArray::from_vec2(py, &target_states).unwrap();

        // generate weights for the discrete distributions of Agents and Targets
        let a = PyArray::from_vec(py, vec![1f32 / (agent_states.len() as f32); agent_states.len()]);
        let b = PyArray::from_vec(py, vec![1f32 / (target_states.len() as f32); target_states.len()]);

        // Compute distance/cost matrix between distributions
        // Default: squared euclidean distance
        let args = PyTuple::new(py, &[xs, xt]);
        let mut cost: Vec<Vec<f32>> = ot.call1("dist", args)?.extract()?;

        let cost_ordfloat: Vec<OrderedFloat<f32>> = cost.clone().into_iter()
            .flatten()
            .map(|val| OrderedFloat(val))
            .collect();

        // get max value of M as an OrderedFloat
        let max_value = cost_ordfloat.into_iter().max().unwrap();

        // divide values of M by it's max value
        for row in cost.iter_mut() {
            for m_ij in row.iter_mut() {
                *m_ij /= max_value.0;
            }
        }

        // cost as a Py object
        let cost = PyArray2::from_vec2(py, &cost).unwrap();

        // Compute optimal coupling matrix
        let args = (a, b, cost, false);
        // let args = PyTuple::new(py, &[a, b, cost]);
        let _g: Vec<Vec<f32>> = ot.call1("emd", args)?.extract()?;

        // convert g0 to matrix of orderedfloat
        let mut tmp = vec![vec![OrderedFloat(1.0); _g[0].len()]; _g.len()];
        for (i, row) in _g.iter().enumerate() {
            for (j, g_ij) in row.iter().enumerate() {
                tmp[i][j] = OrderedFloat(*g_ij);
            }
        }

        // find the max of each row of matrix and collect into vec
        let threshold_values: Vec<OrderedFloat<f32>> = tmp.iter()
            .map(|row| {
                let max_val: OrderedFloat<f32> = *row.into_iter().max().unwrap();
                max_val
            })
            .collect::<Vec<_>>();

        // Return binary matrix corresponding to the assignment
        let mut assignment: Vec<Vec<u32>> = vec![vec![0; _g[0].len()]; _g.len()];
        for ((i, row), thresh) in tmp.iter().enumerate().zip(threshold_values) {
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


pub fn unbalanced_emd_assignment(agent_states: &Vec<Vec<f32>>, target_states: &Vec<Vec<f32>>) -> PyResult< Vec<Vec<u32>> > {

    // Start a python instance and aquire python GIL
    Python::with_gil(|py| {

        // Import python modules
        let ot = PyModule::import(py, "ot")?;

        // Cast rust 2d vector to numpy 2d array
        let xs = PyArray::from_vec2(py, &agent_states).unwrap();
        let xt = PyArray::from_vec2(py, &target_states).unwrap();

        // generate weights for the discrete distributions of Agents and Targets
        let a = PyArray::from_vec(py, vec![1f32 / (agent_states.len() as f32); agent_states.len()]);
        let b = PyArray::from_vec(py, vec![1f32 / (target_states.len() as f32); target_states.len()]);

        // Compute distance/cost matrix between distributions
        // Default: squared euclidean distance
        let args = PyTuple::new(py, &[xs, xt]);
        let mut cost: Vec<Vec<f32>> = ot.call1("dist", args)?.extract()?;

        let cost_ordfloat: Vec<OrderedFloat<f32>> = cost.clone().into_iter()
            .flatten()
            .map(|val| OrderedFloat(val))
            .collect();

        // get max value of M as an OrderedFloat
        let max_value = cost_ordfloat.into_iter().max().unwrap();

        // divide values of M by it's max value
        for row in cost.iter_mut() {
            for m_ij in row.iter_mut() {
                *m_ij /= max_value.0;
            }
        }

        // cost as a Py object
        let cost = PyArray2::from_vec2(py, &cost).unwrap();

        // Compute optimal coupling matrix
        let args = (a, b, cost, 0.1, 1.0);
        let _g: Vec<Vec<f32>> = ot.call1("sinkhorn_unbalanced", args)?.extract()?;

        // convert g0 to matrix of orderedfloat
        let mut tmp = vec![vec![OrderedFloat(1.0); _g[0].len()]; _g.len()];
        for (i, row) in _g.iter().enumerate() {
            for (j, g_ij) in row.iter().enumerate() {
                tmp[i][j] = OrderedFloat(*g_ij);
            }
        }

        // find the max of each row of matrix and collect into vec
        let threshold_values: Vec<OrderedFloat<f32>> = tmp.iter()
            .map(|row| {
                let max_val: OrderedFloat<f32> = *row.into_iter().max().unwrap();
                max_val
            })
            .collect::<Vec<_>>();

        // Return binary matrix corresponding to the assignment
        let mut assignment: Vec<Vec<u32>> = vec![vec![0; _g[0].len()]; _g.len()];
        for ((i, row), thresh) in tmp.iter().enumerate().zip(threshold_values) {
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


