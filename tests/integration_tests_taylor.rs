use neuralink_final::robot;
use neuralink_final::robot::RobotArm;
use neuralink_final::controller;
use std::{sync::Arc, thread};
use tokio::sync::Mutex;
use tokio::runtime::Builder;
use tokio::task::LocalSet;
use neuralink_final::predictor::TaylorQuadraticApproximator;
use tokio::time::Instant;

const PRECISION: u64 = 300_000;

//This function creates the robot and controller and runs them on their own threads
//It then returns the controller and robot so that they can be checked in tests
//All tests rely on this function
fn make_state_taylor_predictor(commands: Vec<u64>,distance_errors: bool, move_errors: bool) -> (Arc<controller::Controller<TaylorQuadraticApproximator>>, Arc<Mutex<RobotArm>>) {
    let (distance_tx, distance_rx) = tokio::sync::mpsc::channel(100);
    let (state_tx, state_rx) = tokio::sync::mpsc::channel(100);
    let (move_tx, move_rx) = tokio::sync::mpsc::channel(100);
    let (dead_tx, dead_rx) = tokio::sync::mpsc::channel(100);

    //Creates the robot simulation
    let robot = Arc::new(Mutex::new(RobotArm::new(0, distance_errors, move_errors)));
    let robot_clone = Arc::clone(&robot);
    //Creates the controller simulation
    let controller = Arc::new(controller::Controller::new(distance_tx, state_tx, move_tx, dead_tx, TaylorQuadraticApproximator{}));
    let controller_clone = Arc::clone(&controller);
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

    return (controller_clone, robot_clone);
}


//Testing sim with no errors
//Testing with robot state errors are ignored in this testing suite
#[test]
fn test_controller_no_errors_taylor() {
    let distances = vec![3_100_000, 3_200_000, 3_300_000, 3_400_000, 3_500_000,
                                3_600_000, 3_700_000, 3_800_000, 3_900_000, 4_000_000,
                                4_100_000, 4_200_000, 4_300_000, 4_400_000, 4_500_000,
                                4_600_000, 4_700_000, 4_800_000, 4_900_000, 5_000_000,
                                5_100_000, 5_200_000, 5_300_000, 5_400_000, 5_500_000,
                                5_600_000, 5_700_000, 5_800_000, 5_900_000, 6_000_000];
    let time = Instant::now();
    let (controller, robot) = make_state_taylor_predictor(distances.clone(),false, false);
    //Assert that the surgery takes less than 10 seconds per thread
    assert!(time.elapsed().as_secs() < distances.len() as u64 * 20, "Test took longer than expected");
    let outcomes = controller.get_outcomes();
    let robot_distances = robot.blocking_lock().brain_distances.clone();
    //Assert that there were no fails
    //Asser thtat the commanded distances were close enough to the actual distances
    for (i, distance) in robot_distances.iter().enumerate() {
        assert!(outcomes[i], "Move failed in no error environment for move {} with outcome {}", i, outcomes[i]);
        assert!(distance.abs_diff(distances[i]) < PRECISION, "Expected {} but got {}", distances[i], distance);
    }
}

//Testing sim with only distance errors
#[test]
fn test_controller_distance_errors_taylor() {
    let distances = vec![3_100_000, 3_200_000, 3_300_000, 3_400_000, 3_500_000,
                                3_600_000, 3_700_000, 3_800_000, 3_900_000, 4_000_000,
                                4_100_000, 4_200_000, 4_300_000, 4_400_000, 4_500_000,
                                4_600_000, 4_700_000, 4_800_000, 4_900_000, 5_000_000,
                                5_100_000, 5_200_000, 5_300_000, 5_400_000, 5_500_000,
                                5_600_000, 5_700_000, 5_800_000, 5_900_000, 6_000_000];
    let time = Instant::now();
    let (controller, robot) = make_state_taylor_predictor(distances.clone(),true, false);
    //Assert that the surgery takes less than 10 seconds per thread
    assert!(time.elapsed().as_secs() < distances.len() as u64 * 20, "Test took longer than expected");
    let outcomes = controller.get_outcomes();
    let robot_distances = robot.blocking_lock().brain_distances.clone();
    //Assert that there were no fails
    //Asser thtat the commanded distances were close enough to the actual distances
    for (i, distance) in robot_distances.iter().enumerate() {
        assert!(outcomes[i], "Move didnt succeeded in distance error environment for move {} with outcome {}", i, outcomes[i]);
        assert!(distance.abs_diff(distances[i]) < PRECISION, "Expected {} but got {}", distances[i], distance);
    }
}

//Testing sim with only move errors
#[test]
fn test_controller_move_errors_taylor() {
    let distances = vec![3_100_000, 3_200_000, 3_300_000, 3_400_000, 3_500_000,
                                3_600_000, 3_700_000, 3_800_000, 3_900_000, 4_000_000,
                                4_100_000, 4_200_000, 4_300_000, 4_400_000, 4_500_000,
                                4_600_000, 4_700_000, 4_800_000, 4_900_000, 5_000_000,
                                5_100_000, 5_200_000, 5_300_000, 5_400_000, 5_500_000,
                                5_600_000, 5_700_000, 5_800_000, 5_900_000, 6_000_000];
    let time = Instant::now();
    let (controller, robot) = make_state_taylor_predictor(distances.clone(),false, true);
    //Assert that the surgery takes less than 10 seconds per thread
    assert!(time.elapsed().as_secs() < distances.len() as u64 * 20, "Test took longer than expected");
    let outcomes = controller.get_outcomes();
    let robot_distances = robot.blocking_lock().brain_distances.clone();
    //Find the indices of the moves that succeeded
    let outcome_indices = outcomes.iter().enumerate().filter(|(_, &x)| x).map(|(i, _)| i).collect::<Vec<usize>>();
    assert!(outcome_indices.len() == robot_distances.len());
    //Asser thtat the commanded distances were close enough to the actual distances on the successful moves
    for (j, i) in outcome_indices.iter().enumerate() {
        let actual_distance = robot_distances[j];
        let commanded_distance = distances[*i];
        print!("{}, {}, {} ", commanded_distance, actual_distance, *i);
        assert!(actual_distance.abs_diff(commanded_distance) < PRECISION, "Expected {} but got {}", commanded_distance, actual_distance);
    }

}