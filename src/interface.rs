
#[derive(Debug, Clone)]
pub enum OCTError {
    // Failed to acquire data from the OCT laser
    AcquisitionError { msg: String },
    // Failed to communicate with the OCT driver
    CommunicationError { msg: String },
    // Timeout waiting for the OCT driver to respond
    TimeoutError { msg: String },

    PredictionError { msg: String },
}
    /// OCTService provides a high level interface with the OCT sensor.
    /// The only function defined here is get_surface_distance which returns
    /// the distance between `inserter_z` and the brain surface in nm.
    /// the function is async because communication time between the software
    /// and the OCT sensor is non-deterministic.
    ///
    /// The initial position of the brain relative to inserter_z is 7mm.
pub trait OCTService {
    // returns the distance between inserter_z and the brain surface in nm
    async fn get_surface_distance(&self) -> Result<u64, OCTError>;
}

#[derive(Debug)]
pub enum Move {
    InserterZ(u64), // desired absolute position in nm
    NeedleZ(u64),   // desired absolute position in nm
}

impl std::fmt::Display for Move {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Move::InserterZ(pos) => write!(f, "InserterZ({})", pos),
            Move::NeedleZ(pos) => write!(f, "NeedleZ({})", pos),
        }
    }
}

/// RobotState represents the current state of the robot where
/// each field represents an axis of our simplified robot.
///  - inserter_z: position of the tip of the needle cartridge which holds the needle
///  - needle_z: position of the needle tip
///
///  A increase in position indicates movement towards the brain surface (down),
///  a decrease in position indicates movement away from the brain surface (up).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotState {
    pub inserter_z: u64, // Absolute encoder position in nm
    pub needle_z: u64,   // Absolute encoder position in nm
}

#[derive(Debug)]
pub enum RobotError {
    // Failed to move the robot
    MoveError { msg: String },
    // lost connection to the robot
    ConnectionError { msg: String },
    // Position exceeds the limits of the robot,
    // can only be thrown by `command_move()`
    PositionError { msg: String },
}

/// Robot provides a high level interface with the robot
/// The simplified robot only has two axes, the tip of the needle cartridge
/// and the needle tip which comes out of the tip of the needle cartridge.
///
/// When a thread is grasped through a successful `command_grasp()` call,
/// the InserterZ axis can be moved in any direction but the NeedleZ axis can only move
/// in a positive direction.
pub trait Robot {
    async fn get_robot_state(&self) -> Result<RobotState, RobotError>;

    async fn command_move(&self, command: &Move) -> Result<(), RobotError>;
    async fn command_grasp(&self) -> Result<(), RobotError>;
}