use tokio::time::Instant;
use crate::interface::OCTError;
use crate::predictor::BrainPredictor;
const MAX_LATENCY_MS: u64 = 18;
const MAX_LATENCY_STD_MS: u64 = 3;
const TAYLOR_POLY_ORDER: u64 = 2; 

pub struct TaylorQuadraticApproximator;

impl TaylorQuadraticApproximator{
    fn _get_taylor_coefs(data: &Vec<u64>, n: u64, latency: f64) -> Vec<f64>{
        assert!(n > 0 && n <= data.len() as u64);
        let mut current = data.iter().map(|&x| x as f64).collect::<Vec<f64>>();
        let mut coefs = Vec::new();
        coefs.push(current[current.len() - 1] as f64);
        let mut factorial = 1;

        // Apply the backward difference n times
        for i in 0..n {
            // Compute the backward difference: Δf(x) = f(x) - f(x-1)
            factorial *= i+1;
            let next = current
                .windows(2)
                .map(|w| (w[1] - w[0]) as f64/ latency as f64) // (w[1] - w[0])
                .collect::<Vec<f64>>();
            coefs.push(next[next.len() - 1] as f64 / factorial as f64);
            current = next;
        }
        return coefs;
    }

    fn passes_predict_assumptions(distance_queue: &Vec<Result<u64, OCTError>>, time_queue: &Vec<Instant>) -> Result<(f64, f64, Vec<u64>, Vec<Instant>), ()> {
        const data_len: usize = TAYLOR_POLY_ORDER as usize+1;
        //We must have enough data to do a Taylor approximation
        if distance_queue.len() < data_len{
            println!("Failing because distance queue is too small");
            return Err(());
        }
        let mut distance_queue = Vec::from(distance_queue.clone());
        let Some(distance_queue) = distance_queue.last_chunk_mut::<data_len>() else {return Err(()); };
        let mut time_queue = Vec::from(time_queue.clone());
        let Some(time_queue) = time_queue.last_chunk_mut::<data_len>() else{ return Err(()); };
        //Our data must be relatively new (cannot be stale)
        if Instant::now().duration_since(time_queue[time_queue.len()-1]).as_millis() as u64 > MAX_LATENCY_MS{
            println!("Failing because latency is too big: {}", Instant::now().duration_since(time_queue[time_queue.len()-1]).as_millis());
            return Err(());
        }
        let times = time_queue.windows(2).map(|w| w[1].duration_since(w[0]).as_millis() as f64).collect::<Vec<f64>>();
        let times_len = times.len() as f64;
        let latency_mean = times.iter().sum::<f64>() / times_len;
        let latency_std = (times.clone().into_iter().map(|x| (x - latency_mean).powi(2)).sum::<f64>() / times_len).sqrt();
        //The latency must be reasonable, and the std must be small to assure low variance on the taylor series approximations
        if latency_mean > MAX_LATENCY_MS as f64 || latency_std > MAX_LATENCY_STD_MS as f64{
            println!("Latency too big: {} {}", latency_mean, latency_std);
            println!("Times: {:?}", times );
            return Err(());
        }
        //We must have enough non error data to do a Taylor approximation
        let distance_queue = distance_queue.iter().filter(|x| x.is_ok()).map(|x| *x.as_ref().unwrap()).collect::<Vec<u64>>();
        if distance_queue.len() < data_len{
            return Err(());
        }
        return Ok((latency_mean, latency_std, distance_queue, Vec::from(time_queue)));
    }
}

impl BrainPredictor for TaylorQuadraticApproximator {
    fn predict(&self, distances: &Vec<Result<u64, OCTError>>, times: &Vec<Instant>, print_coefs: bool) -> Option<impl Fn(f64) -> f64>{
        let Ok((latency_mean, _, distance_queue, __)) = Self::passes_predict_assumptions(distances, times) else {
            return None
        };
        let coefs = Self::_get_taylor_coefs(&distance_queue, TAYLOR_POLY_ORDER, latency_mean);
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