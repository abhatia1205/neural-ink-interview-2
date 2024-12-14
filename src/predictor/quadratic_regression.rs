use std::{char::MAX, f32::MIN};

use crate::interface::OCTError;
use tokio::time::Instant;
use nalgebra::{DMatrix, DVector};
use crate::predictor::BrainPredictor;

const MAX_LATENCY_MS: u64 = 18;
const LR_SIZE: usize = 5;
const MAX_LR_LATENCY_MS: u64 = LR_SIZE as u64 * 25 ;

//To predict where the brain will be in the future, we use a Taylor series approximation of degree 2
//This code, however, generalizes to many degrees
//The code first checks that the data aligns with all of its assumptions before it predicts
//then, it returns a function that predicts the relative position of the brain to the inserter wrt time sinze the function is created


pub struct QuadraticRegression;

impl QuadraticRegression{

    fn regress(distance_queue: &Vec<u64>, time_queue: &Vec<Instant>) -> Option<Vec<f64>>{
        let mut x_rows = Vec::new();
        let comp_time = *time_queue.last().unwrap();

        for i in 0..distance_queue.len(){
            let time = comp_time.duration_since(time_queue[i]).as_millis() as f64;
            x_rows.push(vec![1.0, -time, time*time]);
        }
        let x = DMatrix::from_vec(3, x_rows.len(), x_rows.concat()).transpose();
        let y = DVector::from_vec(distance_queue.iter().map(|x| *x as f64).collect());
        let xt_x = x.transpose() * x.clone();
        let xt_y = x.transpose() * y;

        if let Some(xt_x_inv) = xt_x.try_inverse() {
            let weights = xt_x_inv * xt_y;
            return Some(vec![weights[0], weights[1], weights[2]]);
        } else {
            println!("X^T * X is not invertible");
            return None;
        }
    }

    //Check if our assumptions for prediction hold
    fn passes_predict_assumptions(distance_queue: &Vec<Result<u64, OCTError>>, time_queue: &Vec<Instant>) -> Result<(f64, Vec<u64>, Vec<Instant>), ()> {
        let keep_indices = distance_queue.iter().enumerate().filter(|(_, x)| x.is_ok()).map(|(i, _)| i).collect::<Vec<usize>>();
        let mut distance_queue = distance_queue.iter().filter(|x| x.is_ok()).map(|x| *x.as_ref().unwrap()).collect::<Vec<u64>>();
        let mut time_queue = time_queue.iter().enumerate().filter(|(i, _)| keep_indices.contains(i)).map(|(_, x)| *x).collect::<Vec<Instant>>();
        let Some(distance_queue) = distance_queue.last_chunk_mut::<LR_SIZE>() else {
            println!("Failing because distance queue is too small");
            return Err(());
        };
        let Some(time_queue) = time_queue.last_chunk_mut::<LR_SIZE>() else{ return Err(()); };
        //Our data must be relatively new (cannot be stale)
        if Instant::now().duration_since(*time_queue.first().unwrap()).as_millis() as u64 > MAX_LR_LATENCY_MS{
            println!("Failing because latency is too big: {}", Instant::now().duration_since(*time_queue.first().unwrap()).as_millis());
            return Err(());
        }
        let times = time_queue.windows(2).map(|w| w[1].duration_since(w[0]).as_millis() as f64).collect::<Vec<f64>>();
        let times_len = times.len() as f64;
        let latency_mean = times.iter().sum::<f64>() / times_len;
        //The latency must be reasonable, and the std must be small to assure low variance on the taylor series approximations
        if latency_mean > MAX_LATENCY_MS as f64{
            return Err(());
        }
        return Ok((latency_mean, distance_queue.to_vec(), time_queue.to_vec()));
    }
}

impl BrainPredictor for QuadraticRegression {
    fn predict(&self, distances: &Vec<Result<u64, OCTError>>, times: &Vec<Instant>, print_coefs: bool) -> Option<impl Fn(f64) -> f64>{
        let Ok((__, distance_queue, time_queue)) = Self::passes_predict_assumptions(distances, times) else {
            return None
        };
        let Some(coefs) = Self::regress(&distance_queue, &time_queue) else {
            return None;
        };
        if print_coefs{
            println!("Coefs: {:?}", coefs);
        }
        //Return the function of relative brain position wrt time
        return Some( move |x: f64|{
            //x += OCT_LATENCY_MS as f64;
            coefs[0] + coefs[1]*x + coefs[2]*x*x
        });
    }
}