use crate::interface::{Robot, Move, OCTService, RobotError, OCTError, RobotState};
use rand::Rng;
use tokio::time::{sleep, Duration};

pub struct RobotArm{
    pub arm: String
}

impl OCTService for RobotArm{
    async fn get_surface_distance(&self) -> Result<u64, OCTError> {
        sleep(Duration::from_millis(15)).await;
        let mut rng = rand::thread_rng();
        let probability: f64 = rng.gen(); // Generate a random float between 0.0 and 1.0

        if probability < 0.1 {
            Err(OCTError::CommunicationError { msg: "Connection error".to_string() }) // 10% chance to return an error
        } else {
            Ok(rng.gen_range(1..=100)) // 90% chance to return a random integer (1 to 100)
        }
    }
}

impl Robot for RobotArm{
    async fn command_grasp(&self) -> Result<(), RobotError> {
        Ok(())
    }
    async fn get_robot_state(&self) -> Result<RobotState, RobotError> {
        Ok(RobotState{
            inserter_z: 0,
            needle_z: 0
        })
    }
    async fn command_move(&self, _move_type: &Move) -> Result<(), RobotError> {
        Ok(())
    }
}