use tokio::main;

mod interface;
mod controller;
mod robot;
mod arima;
use interface::RobotState;
use std::sync::Arc;

#[tokio::main]
async fn main() {
    println!("Hello, world!");
    let trial = RobotState{
        inserter_z: 0,
        needle_z: 0,
    };
    println!("{:?}", trial);
    let controller = controller::Controller::new();
    controller::start(Arc::new(controller)).await;
}
