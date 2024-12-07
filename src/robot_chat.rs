use crate::interface::{Robot, Move, OCTService, RobotError, OCTError, RobotState};
use rand::Rng;
use tokio::time::{sleep, Duration, Instant};
use tokio::sync::Mutex;

const NEEDLE_ACCELERATION_NM_MS: i64 = 250;     // nm/ms² (for needle)
const NEEDLE_VELOCITY_NM_MS: u64 = 250_000;     // nm/ms (for needle)
const INSERTER_VELOCITY_NM_MS: u64 = 9_500;    // nm/ms (for inserter arm)

pub struct RobotArmStateMutable {
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
}

impl RobotArmStateMutable {
    fn new(initial_z: u64) -> Self {
        Self {
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
        }
    }
}

pub struct RobotArm {
    pub distance_errors: bool,
    pub state_errors: bool,
    pub move_errors: bool,
    pub brain_location_fn: fn(u64) -> u64,
    init_time: Instant,
    inner: Mutex<RobotArmStateMutable>,
}

impl RobotArm {
    pub fn new() -> RobotArm {
        RobotArm {
            distance_errors: false,
            state_errors: false,
            move_errors: false,
            init_time: Instant::now(),
            brain_location_fn: |x: u64| {
                (7_000_000.0
                    + 2_000_000.0 * (6.0 * x as f64).sin()
                    + 3_000_000.0 * (x as f64).sin()) as u64
            },
            inner: Mutex::new(RobotArmStateMutable::new(0)),
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
}

impl OCTService for RobotArm {
    async fn get_surface_distance(&self) -> Result<u64, OCTError> {
        let mut rng = rand::thread_rng();
        let probability: f64 = rng.gen();
        let brain_position = (self.brain_location_fn)(self.init_time.elapsed().as_millis() as u64);
        sleep(Duration::from_millis(15)).await;
        if probability < 0.1 && self.distance_errors {
            Err(OCTError::CommunicationError {
                msg: "Connection error".to_string(),
            })
        } else {
            Ok(brain_position)
        }
    }
}

impl Robot for RobotArm {
    async fn command_grasp(&self) -> Result<(), RobotError> {
        Ok(())
    }

    async fn get_robot_state(&self) -> Result<RobotState, RobotError> {
        let guard = self.inner.lock().await;
        if guard.is_moving {
            let elapsed = guard.last_move_time.unwrap().elapsed();
            let mut state = guard.state.clone();

            if guard.is_inserter_move {
                // InserterZ move: interpolate inserter_z only, needle_z unchanged
                let pos = Self::interpolate_inserter_position(
                    guard.start_z as i64,
                    guard.target_z as i64,
                    elapsed,
                    guard.total_move_duration,
                );
                assert!(pos >= 0);
                assert!(guard.total_move_duration <= Duration::from_secs(2));
                state.inserter_z = pos as u64;
            } else if guard.is_needle_move {
                // NeedleZ move: interpolate needle_z only, inserter_z unchanged
                let pos = Self::interpolate_needlez_position(
                    guard.start_z as i64,
                    guard.target_z as i64,
                    elapsed,
                    guard.total_move_duration,
                );
                assert!(pos >= 0);
                state.needle_z = pos as u64;
            }

            Ok(state)
        } else {
            Ok(guard.state.clone())
        }
    }

    async fn command_move(&self, move_cmd: &Move) -> Result<(), RobotError> {
        let (is_inserter_move, is_needle_move, start_z, target_z, total_move_duration, error_scheduled);

        {
            let mut guard = self.inner.lock().await;
            assert!(!guard.is_moving);
            // Decide if an error will occur now, before starting the move
            let mut rng = rand::thread_rng();
            let will_error = self.move_errors && rng.gen_bool(0.1);

            match move_cmd {
                Move::InserterZ(z) => {
                    guard.is_inserter_move = true;
                    guard.is_needle_move = false;
                    guard.start_z = guard.state.inserter_z;
                    if will_error {
                        // Pick a partial error position
                        let partial_factor: f64 = rng.gen();
                        guard.target_z = (guard.start_z as i64 + ((*z as i64 - guard.start_z as i64) as f64 * partial_factor) as i64) as u64;
                    } else {
                        guard.target_z = *z;
                    }
                    let distance = (guard.target_z as i64 - guard.start_z as i64).abs();
                    guard.total_move_duration = Self::calculate_inserter_move_time(distance);
                }
                Move::NeedleZ(z) => {
                    guard.is_inserter_move = false;
                    guard.is_needle_move = true;
                    guard.start_z = guard.state.needle_z;
                    if will_error {
                        let partial_factor: f64 = rng.gen();
                        guard.target_z = (guard.start_z as i64 + ((*z as i64 - guard.start_z as i64) as f64 * partial_factor) as i64) as u64;
                    } else {
                        guard.target_z = *z;
                    }
                    let distance = (guard.target_z as i64 - guard.start_z as i64).abs();
                    guard.total_move_duration = Self::calculate_needlez_move_time(distance);
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
        sleep(total_move_duration).await;

        let mut guard = self.inner.lock().await;
        guard.is_moving = false;
        guard.last_move_time = None;
        guard.last_move = None;

        // At this point, the robot physically ends at target_z.
        if is_inserter_move {
            guard.state.inserter_z = target_z;
        } else if is_needle_move {
            guard.state.needle_z = target_z;
        }

        guard.is_inserter_move = false;
        guard.is_needle_move = false;

        if error_scheduled {
            guard.error_scheduled = false;
            return Err(RobotError::ConnectionError {
                msg: "Random error occurred after move (partial position)".to_string(),
            });
        }

        Ok(())
    }
}
