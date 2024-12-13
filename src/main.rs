mod interface;
mod controller;
mod robot;
mod arima;
mod predictor;
use robot::RobotArm;
use std::{sync::Arc, thread};
use tokio::sync::Mutex;
use tokio::runtime::Builder;
use predictor::TaylorQuadraticApproximator;
use predictor::OraclePredictor;
use tokio::task::LocalSet;

fn main() {
    println!("Hello, world!");
    //Creates channels for communication between robot simulation 
    let (distance_tx, distance_rx) = tokio::sync::mpsc::channel(100);
    let (state_tx, state_rx) = tokio::sync::mpsc::channel(100);
    let (move_tx, move_rx) = tokio::sync::mpsc::channel(100);
    let (dead_tx, dead_rx) = tokio::sync::mpsc::channel(100);

    //Creates the robot simulation
    let robot = Arc::new(Mutex::new(RobotArm::new(0, false, false)));
    let robot_clone = Arc::clone(&robot);
    //Creates the controller simulation
    let controller = Arc::new(controller::Controller::new(distance_tx, state_tx, move_tx, dead_tx, TaylorQuadraticApproximator{}));
    let controller_clone = Arc::clone(&controller);
    //Commanded depth in nanometers
    let commands = vec![
        3_100_000, 3_200_000, 3_300_000, 3_400_000, 3_500_000,
        3_600_000, 3_700_000, 3_800_000, 3_900_000, 4_000_000,
        4_100_000, 4_200_000, 4_300_000, 4_400_000, 4_500_000,
        4_600_000, 4_700_000, 4_800_000, 4_900_000, 5_000_000,
        5_100_000, 5_200_000, 5_300_000, 5_400_000, 5_500_000,
        5_600_000, 5_700_000, 5_800_000, 5_900_000, 6_000_000];
    let commands_clone = commands.clone();

     // Create and run the controller on its own thread
    let handle_one = thread::spawn(move || {
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let local = LocalSet::new();
        local.block_on(&rt,async {
            controller::start(controller, &commands).await
        });
    });

    // Create and run the robot sim on its own thread
    let handle_two = std::thread::spawn(move || {
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let local = LocalSet::new();
        local.block_on(&rt,async move {
            robot::start(distance_rx, state_rx, move_rx, dead_rx,robot).await; 
        });
    });

    // Wait for both threads to finish
    handle_one.join().unwrap();
    handle_two.join().unwrap();

    //Filter indices with true value from controller_clone.outcomes
    let outcome_indices = controller_clone.get_outcomes().iter().enumerate().filter(|(_, &x)| x).map(|(i, _)| i).collect::<Vec<usize>>();
    assert!(outcome_indices.len() == robot_clone.blocking_lock().brain_distances.len());

    let mut abs_distances = Vec::new();
    //Print the commanded vs actual distance
    for (j, i) in outcome_indices.iter().enumerate() {
        let actual_distance = robot_clone.blocking_lock().brain_distances[j];
        let commanded_distance = commands_clone[*i];
        abs_distances.push(actual_distance.abs_diff(commanded_distance));
        print!("{}, {}, {} ", commanded_distance, actual_distance, *i);
        println!("");
    }

    println!("Average absolute distance: {}", abs_distances.iter().sum::<u64>() / abs_distances.len() as u64);
    println!("Max absolute distance: {}", abs_distances.iter().max().unwrap());
    println!("Std dev: {}", (abs_distances.iter().map(|x| (*x as f64 - abs_distances.iter().sum::<u64>() as f64 / abs_distances.len() as f64).powi(2)).sum::<f64>() / abs_distances.len() as f64).sqrt());
    println!("Num successes: {}", outcome_indices.len());

}