//NOT USED


// use nalgebra::{DMatrix, DVector};
// use crate::interface::OCTError;
// use std::collections::VecDeque;
// extern crate approx;

// const MIN_NUM_POINTS: u64 = 8;


// pub struct ArimaError;


// pub struct ARIMA{
//     l1_coef: f64,
//     l2_coef: f64,
//     constant: f64,
//     trained: bool,
//     min_num_points: u64
// }

// impl ARIMA{
//     pub fn new(min_num_points: u64) -> ARIMA{
//         ARIMA{
//             l1_coef: 0.0,
//             l2_coef: 0.0,
//             constant: 0.0,
//             trained: false,
//             min_num_points: min_num_points
//         }
//     }

//     pub fn is_trained(&self) -> bool{
//         self.trained
//     }

//     pub fn predict (&self, lag1: f64, lag2: f64) -> Result<f64, ArimaError>{
//         if !self.trained{
//             return Err(ArimaError{});
//         }
//         Ok(self.l1_coef * lag1 + self.l2_coef * lag2 + self.constant)
//     }

//     pub fn train(&mut self, vec_deque: &VecDeque<Result<f64, OCTError>>) -> bool {
//         let data = Vec::from(vec_deque.clone());
//         let mut x_rows = Vec::new();
//         let mut y_rows = Vec::new();

//         if data.len() < self.min_num_points as usize {
//             println!("Not enough data points: {}", data.len());
//             return false;
//         }
    
//         // Iterate through the vector to find consecutive triplets
//         for i in 0..data.len().saturating_sub(2) {
//             if let (Ok(a), Ok(b), Ok(c)) = (&data[i], &data[i + 1], &data[i + 2]) {
//                 x_rows.push(vec![*a, *b, 1.0]);
//                 y_rows.push(*c);
//             }
//         }

//         if x_rows.len() < self.min_num_points as usize {
//             println!("Not enough data points: {}", x_rows.len());
//             return false;
//         }
//         let x_matrix = DMatrix::from_vec(3, x_rows.len(), x_rows.concat()).transpose();
//         let y_matrix = DVector::from_vec(y_rows);
//         let xt_x = x_matrix.transpose() * x_matrix.clone();

//         // Compute X^T * y
//         let xt_y = x_matrix.transpose() * y_matrix;

//         if let Some(xt_x_inv) = xt_x.try_inverse() {
//             let weights = xt_x_inv * xt_y;
//             self.l1_coef = weights[0];
//             self.l2_coef = weights[1];
//             self.constant = weights[2];
//             self.trained = true;
//             return true;
//         } else {
//             println!("X^T * X is not invertible");
//             return false;
//         }
//     }

//     pub fn train_u64(&mut self, deque: &VecDeque<Result<u64, OCTError>>) -> bool {
//         let temp = deque.into_iter().map(|x| if x.is_err() {Err(x.clone().unwrap_err())} else {Ok(x.clone().unwrap() as f64)}).collect();
//         return self.train(&temp);
//     }
// }

// #[cfg(test)]
// mod tests {
//     use approx::assert_relative_eq;
//     use rand::Rng;
//     use super::*;

//     // Testing ARIMA with initial states 1,2 and equation x[i] = 0.6*x[i-1] + 0.3*x[i-2] + 1
//     #[test]
//     fn test_arima() {
//         let mut arima = ARIMA::new(MIN_NUM_POINTS);
//         let mut deque: VecDeque<f64> = VecDeque::new();
//         deque.push_back(1.0);
//         deque.push_back(2.0);
//         for i in 0..15 {
//             deque.push_back(0.6 * deque[i] + 0.3 * deque[i + 1] + 1.0);
//         }
//         let deque = deque.iter().map(|x| Ok(*x)).collect();
//         let result = arima.train(&deque);
//         assert_eq!(result, true);
//         assert_relative_eq!(arima.l1_coef, 0.6, max_relative = 0.001);
//         assert_relative_eq!(arima.l2_coef, 0.3, max_relative = 0.001);
//         assert_relative_eq!(arima.constant, 1.0, max_relative = 0.001);
//         assert_eq!(arima.trained, true);
//     }

//     // Testing ARIMA with initial states 1,2 and equation x[i] = 0.6*x[i-1] + 0.3*x[i-2] + 1
//     #[test]
//     fn test_arima_with_errors() {
//         let mut arima = ARIMA::new(MIN_NUM_POINTS);
//         let mut deque: VecDeque<f64> = VecDeque::new();
//         deque.push_back(1.0);
//         deque.push_back(2.0);
//         for i in 0..100 {
//             deque.push_back(0.6 * deque[i] + 0.3 * deque[i + 1] + 1.0);
//         }
//         let deque = deque.iter().map(|x| {
//             let probability: f64 = rand::thread_rng().gen();
//             if probability < 0.3 { Err(OCTError::AcquisitionError { msg: "Acquisition error".to_string() }) } else { Ok(*x) }
//         }).collect();
//         let result = arima.train(&deque);
//         assert_eq!(result, true);
//         assert_relative_eq!(arima.l1_coef, 0.6, max_relative = 0.001);
//         assert_relative_eq!(arima.l2_coef, 0.3, max_relative = 0.001);
//         assert_relative_eq!(arima.constant, 1.0, max_relative = 0.001);
//         assert_eq!(arima.trained, true);
//     }

// }