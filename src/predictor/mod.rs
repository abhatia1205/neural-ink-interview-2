use crate::interface::OCTError;
use tokio::time::Instant;

pub mod oracle_approx;
pub mod quadratic_regression;
pub mod taylor_approx;

pub trait BrainPredictor {
    fn predict(&self, distances: &Vec<Result<u64, OCTError>>, times: &Vec<Instant>, print_coefs: bool) -> Option<impl Fn(f64) -> f64>;
    fn train(&self) -> bool{
        return true;
    }
}