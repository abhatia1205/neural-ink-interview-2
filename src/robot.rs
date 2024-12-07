use crate::interface::{Robot, Move, OCTService, RobotError, OCTError, RobotState};
use rand::Rng;
use tokio::time::{sleep, Duration, Instant};

const NEEDLE_ACCELERATION_NM_MS: i64 = 250;
const NEEDLE_VELOCITY_NM_MS: u64 = 250_000;
const ROBOT_VELOCITY_NM_MS: u64 = 9_500;
pub struct RobotArmStateMutable{
    state: RobotState,
    is_moving: bool,
    last_move_time: Option<Instant>,
    last_move: Option<Move>,
}

pub struct RobotArm{
    pub distance_errors: bool,
    pub state_errors: bool,
    pub move_errors: bool,
    brain_location_fn: fn(u64) -> u64,
    init_time: Instant
}

impl RobotArm{
    pub fn new() -> RobotArm{
        let robot_mutable_state = RobotArmState{};
        RobotArm{
            distance_errors: false,
            state_errors: false,
            move_errors: false,
            init_time: Instant::now(),
            brain_location_fn: |x: u64|{
                return (7_000_000.0 + 2_000_000.0*(6.0 * x as f64).sin() + 3_000_0000.0*(x as f64).sin()) as u64;
            }
        }
    }
}

impl OCTService for RobotArm{
    async fn get_surface_distance(&self) -> Result<u64, OCTError>{
        let mut rng = rand::thread_rng();
        let probability: f64 = rng.gen(); // Generate a random float between 0.0 and 1.0
        let brain_position = (self.brain_location_fn)(self.init_time.elapsed().as_millis() as u64);
        sleep(Duration::from_millis(15)).await;
        if probability < 0.1 && self.distance_errors {
            Err(OCTError::CommunicationError { msg: "Connection error".to_string() }) // 10% chance to return an error
        } else {
            Ok(brain_position) // 90% chance to return a random integer (1 to 100)
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