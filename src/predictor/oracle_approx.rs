//THE FOLLOWING CODE IS BUGGY, DO NOT USE
use tokio::time::Instant;
use crate::interface::OCTError;
use crate::predictor::BrainPredictor;
const MIN_SIZE: usize =3;
const MAX_LATENCY_MS: u64 = 18;

pub struct OraclePredictor{
    init_time: Instant,
}

impl OraclePredictor{
    pub fn new() -> OraclePredictor{
        OraclePredictor{
            init_time: Instant::now(),
        }
    }

    fn passes_predict_assumptions(distance_queue: &Vec<Result<u64, OCTError>>, time_queue: &Vec<Instant>) -> Result<(Vec<u64>, Vec<Instant>), ()> {
        const data_len: usize = MIN_SIZE+1;
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
        //We must have enough non error data to do a Taylor approximation
        let distance_queue = distance_queue.iter().filter(|x| x.is_ok()).map(|x| *x.as_ref().unwrap()).collect::<Vec<u64>>();
        if distance_queue.len() < data_len{
            println!("Failing because distance queue has too many errors");
            return Err(());
        }
        return Ok((distance_queue, Vec::from(time_queue)));
    }
}

impl BrainPredictor for OraclePredictor{
    fn predict(&self, distances: &Vec<Result<u64, OCTError>>, times: &Vec<Instant>, _: bool) -> Option<impl Fn(f64) -> f64>{
        if !Self::passes_predict_assumptions(distances, times).is_ok(){
            return None
        };
        return Some(  |x: f64| {
            let x = x + self.init_time.elapsed().as_millis() as f64;
            7_000_000.0 - 5332309.0 
                + 500_000.0 * (6.0 * x as f64/1000.0).sin()
                + 1_000_000.0 * (x as f64/1000.0).sin()
        });
    }
}