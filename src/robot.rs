use crate::interface::{Move, RobotError, OCTError, RobotState};
use rand::Rng;
use tokio::time::{sleep, Duration, Instant};
use tokio::sync::Mutex;
use std::sync::Arc;
use tokio::sync::{oneshot,mpsc};

const NEEDLE_ACCELERATION_NM_MS: i64 = 250;     // nm/ms² (for needle)
const NEEDLE_VELOCITY_NM_MS: u64 = 250_000;     // nm/ms (for needle)
const INSERTER_VELOCITY_NM_MS: u64 = 9_500;    // nm/ms (for inserter arm)
const PROBABILITY_OF_ERROR: f64 = 0.1;

pub struct RobotArm {
    pub distance_errors: bool,
    pub state_errors: bool,
    pub move_errors: bool,
    pub brain_location_fn: fn(u64) -> u64,
    init_time: Instant,
    state: RobotState,
    is_moving: bool,
    last_move_time: Option<Instant>,
    last_move: Option<Move>,
    total_move_duration: Duration,
    start_z: u64,
    target_z: u64,
    is_inserter_move: bool,
    is_needle_move: bool,
    error_scheduled: bool,
    pub brain_distances: Vec<u64>,
}

impl RobotArm {
    pub fn new(initial_z: u64, distance_errors: bool, move_errors: bool) -> RobotArm {
        RobotArm {
            distance_errors,
            state_errors: false,
            move_errors,
            init_time: Instant::now(),
            brain_location_fn: |x: u64| {
                (7_000_000.0
                    + 500_000.0 * (6.0 * x as f64/1000.0).sin()
                    + 1_000_000.0 * (x as f64/1000.0).sin()) as u64
            },
            state: RobotState {
                inserter_z: initial_z,
                needle_z: 0,
            },
            is_moving: false,
            last_move_time: None,
            last_move: None,
            total_move_duration: Duration::from_millis(0),
            start_z: initial_z,
            target_z: initial_z,
            is_inserter_move: false,
            is_needle_move: false,
            error_scheduled: false,
            brain_distances: Vec::new(),
        }
    }

    /// Calculate total move time for needle moves using a trapezoidal profile.
    /// Inserter moves are handled separately.
    fn calculate_needlez_move_time(distance_nm: i64) -> Duration {
        let a = NEEDLE_ACCELERATION_NM_MS as f64;
        let v = NEEDLE_VELOCITY_NM_MS as f64;
        let d = distance_nm.abs() as f64;
        let d_min = v * v / a;

        let total_time_ms = if d < d_min {
            2.0*(d / a).sqrt()
        } else {
            let t_accel = v / a;
            let d_accel = 0.5 * a * t_accel * t_accel;
            let d_cruise = d - 2.0 * d_accel;
            let t_cruise = d_cruise / v;
            t_accel + t_cruise + t_accel
        };

        Duration::from_millis(total_time_ms as u64)
    }

    /// Interpolate needle moves using trapezoidal profile.
    fn interpolate_needlez_position(
        start_z: i64,
        target_z: i64,
        elapsed: Duration,
        total: Duration,
    ) -> i64 {
        let a = NEEDLE_ACCELERATION_NM_MS as f64;
        let v = NEEDLE_VELOCITY_NM_MS as f64;
        let d = (target_z - start_z) as f64;
        let direction = if target_z >= start_z { 1.0 } else { -1.0 };

        let t = elapsed.as_millis() as f64;
        let total_t = total.as_millis() as f64;
        let d_min = 2.0 * (v * v / a);

        if t >= total_t {
            return target_z;
        }

        if d.abs() < d_min {
            let half_t = total_t / 2.0;
            if t <= half_t {
                let s = 0.5 * a * t * t;
                (start_z as f64 + direction * s) as i64
            } else {
                let half_v = a * half_t;
                let dt = t - half_t;
                let s = (d / 2.0) + half_v * dt - 0.5 * a * dt * dt;
                (start_z as f64 + direction * s) as i64
            }
        } else {
            let t_accel = v / a;
            let d_accel = 0.5 * a * t_accel * t_accel;
            let t_cruise = total_t - 2.0 * t_accel;
            if t <= t_accel {
                let s = 0.5 * a * t * t;
                (start_z as f64 + direction * s) as i64
            } else if t <= t_accel + t_cruise {
                let dt = t - t_accel;
                let s = d_accel + v * dt;
                (start_z as f64 + direction * s) as i64
            } else {
                let dt = t - (t_accel + t_cruise);
                let d_cruise = v * t_cruise;
                let s = d_accel + d_cruise + v * dt - 0.5 * a * dt * dt;
                (start_z as f64 + direction * s) as i64
            }
        }
    }

    /// For inserter moves, we have constant velocity motion:
    /// total_time = distance / INSERTER_VELOCITY_NM_MS
    fn calculate_inserter_move_time(distance_nm: i64) -> Duration {
        let distance = distance_nm.abs() as f64;
        let time_ms = distance / INSERTER_VELOCITY_NM_MS as f64;
        Duration::from_millis(time_ms as u64)
    }

    /// Interpolate inserter moves linearly based on constant velocity.
    fn interpolate_inserter_position(
        start_z: i64,
        target_z: i64,
        elapsed: Duration,
        total: Duration,
    ) -> i64 {
        let total_t = total.as_millis() as f64;
        let t = elapsed.as_millis() as f64;
        let d = (target_z - start_z) as f64;
        let fraction = (t / total_t).min(1.0);
        (start_z as f64 + d * fraction) as i64
    }

    fn _get_state(&self) -> Result<RobotState, RobotError> {
        if self.is_moving {
            let elapsed = self.last_move_time.unwrap().elapsed();
            let mut state = self.state.clone();
            if self.is_inserter_move {
                // InserterZ move: interpolate inserter_z only, needle_z unchanged
                let pos = RobotArm::interpolate_inserter_position(
                    self.start_z as i64,
                    self.target_z as i64,
                    elapsed,
                    self.total_move_duration,
                );
                assert!(pos >= 0);
                assert!(self.total_move_duration <= Duration::from_secs(2));
                state.inserter_z = pos as u64;
            } else if self.is_needle_move {
                // NeedleZ move: interpolate needle_z only, inserter_z unchanged
                let pos = RobotArm::interpolate_needlez_position(
                    self.start_z as i64,
                    self.target_z as i64,
                    elapsed,
                    self.total_move_duration,
                );
                assert!(pos >= 0);
                state.needle_z = pos as u64;
            }
            return Ok(state.clone());
        } else {
            return Ok(self.state.clone());
        }
    }

}

async fn get_state(robot: Arc<Mutex<RobotArm>>, mut state_rx: mpsc::Receiver<((), oneshot::Sender<Result<RobotState, RobotError>>)>) -> () {
    println!("get_state");
    while let Some((_, tx)) = state_rx.recv().await {
        tx.send({
            robot.lock().await._get_state()}
        ).unwrap();
    }
}

async fn mv(robot: Arc<Mutex<RobotArm>>, mut move_rx: mpsc::Receiver<(Move, oneshot::Sender<Result<(), RobotError>>)>,) -> (){
    println!("mv");
    while let Some((move_cmd, tx)) = move_rx.recv().await {
        let (is_inserter_move, is_needle_move, start_z, target_z, total_move_duration, error_scheduled);

        {
            let mut guard = robot.lock().await;
            assert!(!guard.is_moving);
            // Decide if an error will occur now, before starting the move
            let mut rng = rand::thread_rng();
            let mut will_error = guard.move_errors && rng.gen_bool(PROBABILITY_OF_ERROR);

            match move_cmd {
                Move::InserterZ(z) => {
                    guard.is_inserter_move = true;
                    guard.is_needle_move = false;
                    guard.start_z = guard.state.inserter_z;
                    if will_error {
                        // Pick a partial error position
                        let partial_factor: f64 = rng.gen();
                        guard.target_z = (guard.start_z as i64 + ((z as i64 - guard.start_z as i64) as f64 * partial_factor) as i64) as u64;
                    } else {
                        guard.target_z = z;
                    }
                    let distance = (guard.target_z as i64 - guard.start_z as i64).abs();
                    guard.total_move_duration = RobotArm::calculate_inserter_move_time(distance);
                }
                Move::NeedleZ(z) => {
                    guard.is_inserter_move = false;
                    guard.is_needle_move = true;
                    guard.start_z = guard.state.needle_z;
                    // if(z == 0){
                    //     will_error = false;
                    // }
                    if(z != 0){
                        assert!(guard.state.needle_z == 0);
                    }
                    if will_error {
                        let partial_factor: f64 = rng.gen();
                        guard.target_z = (guard.start_z as i64 + ((z as i64 - guard.start_z as i64) as f64 * partial_factor) as i64) as u64;
                    } else {
                        guard.target_z = z;
                    }
                    let distance = (guard.target_z as i64 - guard.start_z as i64).abs();
                    guard.total_move_duration = RobotArm::calculate_needlez_move_time(distance);
                }
            }

            guard.is_moving = true;
            guard.last_move_time = Some(Instant::now());
            guard.last_move = Some(move_cmd.clone());
            guard.error_scheduled = will_error;

            // Extract fields for use outside lock (to avoid long lock time during sleep)
            is_inserter_move = guard.is_inserter_move;
            is_needle_move = guard.is_needle_move;
            start_z = guard.start_z;
            target_z = guard.target_z;
            total_move_duration = guard.total_move_duration;
            error_scheduled = guard.error_scheduled;
        }

        // Simulate the move duration
        match move_cmd {
            Move::NeedleZ(z) => {
                println!("InserterZ: {} -> {} with duration {}", start_z, z, total_move_duration.as_millis());
            }
            _ => {}
        }
        sleep(total_move_duration).await;
        {
            let mut guard = robot.lock().await;
            guard.is_moving = false;
            guard.last_move_time = None;
            guard.last_move = None;
            // At this point, the robot physically ends at target_z.
            if is_inserter_move {
                guard.state.inserter_z = target_z;
            } else if is_needle_move {
                let brain_position = (guard.brain_location_fn)(guard.init_time.elapsed().as_millis() as u64) - guard.state.inserter_z;
                if !error_scheduled && target_z != 0 {
                    assert!(guard.move_errors || brain_position < target_z, "brain position: {}, target position: {}", brain_position, target_z);
                    guard.brain_distances.push(if target_z < brain_position {0} else {target_z - brain_position});
                }
                guard.state.needle_z = target_z;
            }

            guard.is_inserter_move = false;
            guard.is_needle_move = false;

            if error_scheduled {
                guard.error_scheduled = false;
                tx.send(Err(RobotError::MoveError {
                    msg: "Random error occurred after move".to_string(),
                })).unwrap();
            } else{
                tx.send(Ok(())).unwrap();
            }
        }
    }
}

pub async fn start(distance_rx: mpsc::Receiver<((), oneshot::Sender<Result<u64, OCTError>>)>,
                    state_rx: mpsc::Receiver<((), oneshot::Sender<Result<RobotState, RobotError>>)>,
                    move_rx: mpsc::Receiver<(Move, oneshot::Sender<Result<(), RobotError>>)>,
                    mut dead_rx: mpsc::Receiver<()>,
                    robot: Arc<Mutex<RobotArm>>) {

    let r1 = Arc::clone(&robot);
    let r2 = Arc::clone(&robot);
    let r3 = Arc::clone(&robot);
    println!("Starting robot...");
    tokio::task::spawn(get_distance(r1, distance_rx));
    tokio::task::spawn(mv(r2, move_rx));
    tokio::task::spawn(get_state(r3, state_rx));
    dead_rx.recv().await;
}

async fn get_distance(robot: Arc<Mutex<RobotArm>>, mut distance_rx: mpsc::Receiver<((), oneshot::Sender<Result<u64, OCTError>>)>,) -> () {
    println!("get_distance");
    while let Some((_, tx)) = distance_rx.recv().await {
        let will_error = { rand::thread_rng().gen_bool(PROBABILITY_OF_ERROR)};
        let (diff, distance_errors) = 
        {
            let guard = robot.lock().await;
            let robot_position = guard._get_state().unwrap().inserter_z;
            let brain_position = (guard.brain_location_fn)(guard.init_time.elapsed().as_millis() as u64);
            assert!(brain_position > 0 && brain_position > robot_position, "brain position: {}, robot position: {}", brain_position, robot_position);
            (brain_position - robot_position, guard.distance_errors)
        };
        sleep(Duration::from_millis(15)).await;
        if will_error && distance_errors {
            tx.send(Err(OCTError::CommunicationError { msg: "Connection error".to_string() })).unwrap();
        } else {
            tx.send(Ok(diff)).unwrap();
        }
    }
}