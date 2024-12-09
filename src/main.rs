mod interface;
mod controller;
//mod robot;
mod robot_chat;
mod arima;
use interface::RobotState;
use robot_chat::RobotArm;
use std::{os::macos::raw::stat, sync::Arc, thread};
use tokio::sync::Mutex;
use tokio::runtime::Builder;

fn main() {
    println!("Hello, world!");
    let (distance_tx, distance_rx) = tokio::sync::mpsc::channel(100);
    let (state_tx, state_rx) = tokio::sync::mpsc::channel(100);
    let (move_tx, move_rx) = tokio::sync::mpsc::channel(100);

    let mut robot = Arc::new(Mutex::new(RobotArm::new(0)));
    let controller = controller::Controller::new(distance_tx, state_tx, move_tx);

     // Create and run the first runtime on its own thread
    let handle_one = thread::spawn(|| {
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        
        rt.block_on(async {
            // These async functions run on "Thread 1"
            controller::start(Arc::new(controller), &vec![3_100_000, 3_200_000, 3_300_000, 3_400_000, 3_500_000,
                                                        3_600_000, 3_700_000, 3_800_000, 3_900_000, 4_000_000,
                                                        4_100_000, 4_200_000, 4_300_000, 4_400_000, 4_500_000,
                                                        4_600_000, 4_700_000, 4_800_000, 4_900_000, 5_000_000,
                                                        5_100_000, 5_200_000, 5_300_000, 5_400_000, 5_500_000,
                                                        5_600_000, 5_700_000, 5_800_000, 5_900_000, 6_000_000]).await
        });
    });

    let handle_two = std::thread::spawn(move || {
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
            
        rt.block_on(async move {
            // Because run now takes &mut self, 
            // we can borrow thread_two_context mutably here.
            robot_chat::start(distance_rx, state_rx, move_rx, robot).await; 

            // After `run()` completes, we could still use `thread_two_context` if we wanted to,
            // or call `run()` again.
        });
    });

    // Wait for both threads to finish
    handle_one.join().unwrap();
    handle_two.join().unwrap();

}