
use std::error::Error;
use nalgebra::{DVector, DMatrix};
use ot::emd;
use ot::unbalanced::{sinkhorn_unbalanced, UnbalancedSolverType}; 
use ot::utils::metrics::{dist, MetricType};

// TODO: one single function for assignments that match for nagents = ntargets or not

pub fn emd_assignment(agent_states: &Vec<Vec<f32>>, target_states: &Vec<Vec<f32>>) -> Result<Vec<Vec<u32>>, Box<dyn Error>> {

    let nagents = agent_states.len();
    let dim_agents = agent_states[0].len();

    let ntargets = target_states.len();
    let dim_targets = target_states[0].len();

    // Agent/target states in a single vector
    let mut xs_vec = Vec::new();
    for state in agent_states.iter() {
        for ele in state.iter() {
            xs_vec.push(*ele as f64);
        }
    }

    let mut xt_vec = Vec::<f64>::new();
    for state in target_states.iter() {
        for ele in state.iter() {
            xt_vec.push(*ele as f64);
        }
    }

    // Create matrices where each row is a state using vector slices
    let xs = DMatrix::<f64>::from_row_slice(nagents, dim_agents, xs_vec.as_slice());
    let xt = DMatrix::<f64>::from_row_slice(ntargets, dim_targets, xt_vec.as_slice());

    // Weights of discrete distribution masses representing agents/target states
    // For now: Uniform distribution
    let mut a = DVector::<f64>::from_vec(vec![1f64 / (nagents as f64); nagents]);
    let mut b = DVector::<f64>::from_vec(vec![1f64 / (ntargets as f64); ntargets]);

    // Get Euclidean distance cost between distributions of agent/target states
    let mut cost = dist(&xs, Some(&xt), MetricType::SqEuclidean);

    // Normalize each cost row by it's maximum value
    for mut row in cost.row_iter_mut() {
        let max_ele = row.max();
        row.scale_mut(1f64 / max_ele);
    }

    // Get coupling matrix according to a given cost
    let gamma = emd(&mut a, &mut b, &mut cost, 100000, true);

    // Convert coupling matrix to binary coupling matrix
    let mut binary = vec![vec![0; ntargets]; nagents];
    for (i, row) in gamma.row_iter().enumerate() {
        let threshold = row.max();
        for (j, ele) in row.iter().enumerate() {
            if ele >= &threshold {
                binary[i][j] = 1;
            } else {
                binary[i][j] = 0;
            }
        }
    }

    Ok(binary)

}


pub fn unbalanced_emd_assignment(agent_states: &Vec<Vec<f32>>, target_states: &Vec<Vec<f32>>) -> Result< Vec<Vec<u32>>, Box<dyn Error> > {

    let nagents = agent_states.len();
    let dim_agents = agent_states[0].len();

    let ntargets = target_states.len();
    let dim_targets = target_states[0].len();

    // Agent/target states in a single vector
    let mut xs_vec = Vec::new();
    for state in agent_states.iter() {
        for ele in state.iter() {
            xs_vec.push(*ele as f64);
        }
    }

    let mut xt_vec = Vec::<f64>::new();
    for state in target_states.iter() {
        for ele in state.iter() {
            xt_vec.push(*ele as f64);
        }
    }

    // Create matrices where each row is a state using vector slices
    let xs = DMatrix::<f64>::from_row_slice(nagents, dim_agents, xs_vec.as_slice());
    let xt = DMatrix::<f64>::from_row_slice(ntargets, dim_targets, xt_vec.as_slice());

    // Weights of discrete distribution masses representing agents/target states
    // For now: Uniform distribution
    let mut a = DVector::<f64>::from_vec(vec![1f64 / (nagents as f64); nagents]);
    let mut b = DMatrix::<f64>::from_row_slice(ntargets, 1, vec![1f64 / (ntargets as f64); ntargets].as_slice());

    // Get Euclidean distance cost between distributions of agent/target states
    let mut cost = dist(&xs, Some(&xt), MetricType::SqEuclidean);

    // Normalize each cost row by it's maximum value
    for mut row in cost.row_iter_mut() {
        let max_ele = row.max();
        row.scale_mut(1f64 / max_ele);
    }

    // Get coupling matrix according to a given cost
    let gamma = sinkhorn_unbalanced(&mut a, &mut b, &mut cost, 0.1, 1.0,
                                    UnbalancedSolverType::Sinkhorn,
                                    None, None, None);

    // Convert coupling matrix to binary coupling matrix
    let mut binary = vec![vec![0; ntargets]; nagents];
    for (i, row) in gamma.row_iter().enumerate() {
        let threshold = row.max();
        for (j, ele) in row.iter().enumerate() {
            if ele >= &threshold {
                binary[i][j] = 1;
            } else {
                binary[i][j] = 0;
            }
        }
    }

    Ok(binary)

}


