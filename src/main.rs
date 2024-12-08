mod interface;
mod controller;
//mod robot;
mod robot_chat;
mod arima;
use interface::RobotState;
use std::sync::Arc;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    println!("Hello, world!");
    let trial = RobotState{
        inserter_z: 0,
        needle_z: 0,
    };
    println!("{:?}", trial);
    let controller = controller::Controller::new();
    controller::start(Arc::new(controller), &vec![3_100_000, 3_200_000, 3_300_000, 3_400_000, 3_500_000,
                                                        3_600_000, 3_700_000, 3_800_000, 3_900_000, 4_000_000,
                                                        4_100_000, 4_200_000, 4_300_000, 4_400_000, 4_500_000,
                                                        4_600_000, 4_700_000, 4_800_000, 4_900_000, 5_000_000,
                                                        5_100_000, 5_200_000, 5_300_000, 5_400_000, 5_500_000,
                                                        5_600_000, 5_700_000, 5_800_000, 5_900_000, 6_000_000]).await;
}