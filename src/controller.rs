use crate::interface::{RobotError, RobotState, OCTService, OCTError, Move, Robot}; //OCTError;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, Duration, Instant};
use std::cmp::max;
use std::sync::{Mutex,Arc}; //Arc;
use std::collections::VecDeque;
use crate::arima::ARIMA;
use roots::find_roots_quartic;
use roots::Roots;

const CALIBRATION_SAMPLES: u64 = 500;
const MIN_CALIBRATION_TRAINING_POINTS: u64 = CALIBRATION_SAMPLES/2;
const MIN_DISTANCE_BRAIN_TO_ARM_NM: u64 = 200_000;
const MAX_DISTANCES: u64 = 100;
const MAX_STATES: u64 = 100;
const MAX_IB_TIME: u64 = 30_000;
const MAX_CONSECUTIVE_PREDICTION_ERRORS: u64 = 5;
const MAX_PREDICTION_ERROR_NM: u64 = 50_000;
const MAX_BRAIN_VEL_NM_S: u64 = 10_000;
const MAX_TIME_TO_REACH_DEPTH: u64 = 150;
const MAX_LATENCY_MS: u64 = 30;
const MAX_DISTANCE_TO_BRAIN_MOVE: u64 = MIN_DISTANCE_BRAIN_TO_ARM_NM *2;
const TAYLOR_POLY_ORDER: u64 = 4; //must stay 4 unless get move location function is changed

const OCT_POLL_MILLIS: u64 = 5;
const OCT_LATENCY_MEAN: u64 = 15;
const ROBOT_STATE_POLL_MILLIS: u64 = 5;
const NEEDLE_ACCELERATION_NM_MS: i64 = 250;
//const NEEDLE_VELOCITY_NM_MS: u64 = 250_000;
const COMMANDED_DEPTH_MIN_NM: u64 = 3_000_000;
const COMMANDED_DEPTH_MAX_NM: u64 = 7_000_000;


#[derive(PartialEq, Clone, Copy)]
enum ControllerState {
    Dead,
    OutOfBrainUncalibrated,
    OutOfBrainCalibrated,
    InBrain,
    Panic
}

impl std::fmt::Display for ControllerState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            ControllerState::Dead => write!(f, "Dead"),
            ControllerState::OutOfBrainUncalibrated => write!(f, "OutOfBrainUncalibrated"),
            ControllerState::OutOfBrainCalibrated => write!(f, "OutOfBrainCalibrated"),
            ControllerState::InBrain => write!(f, "InBrain"),
            ControllerState::Panic => write!(f, "Panic")
        }
    }
}

pub struct Controller {
    current_state: Mutex<ControllerState>, //ControllerState,
    distance_queue: Mutex<VecDeque<Result<u64, OCTError>>>, //VecDeque<(Result<u64, OCTError>, Instant>>>,
    distance_time_queue: Mutex<VecDeque<Instant>>,
    robot_queue: Mutex<VecDeque<Result<RobotState, RobotError>>>, //VecDeque<(Result<RobotState, RobotError>, Instant>>>,
    robot_time_queue: Mutex<VecDeque<Instant>>,
    consecutive_errors: Mutex<u64>,
    pre_move_location: Mutex<Option<u64>>, //u64
    arima: Mutex<Option<ARIMA>>, //ARIMA>
    distance_tx: mpsc::Sender<((), oneshot::Sender<Result<u64, OCTError>>)>,
    state_tx: mpsc::Sender<((), oneshot::Sender<Result<RobotState, RobotError>>)>, //mpsc::Sender<()>,
    move_tx: mpsc::Sender<(Move, oneshot::Sender<Result<(), RobotError>>)>, //mpsc::Sender<Move>,
    dead_tx: mpsc::Sender<()>,
}

impl Controller{
    pub fn new(distance_tx: mpsc::Sender<((), oneshot::Sender<Result<u64, OCTError>>)>,
            state_tx: mpsc::Sender<((), oneshot::Sender<Result<RobotState, RobotError>>)>,
            move_tx: mpsc::Sender<(Move, oneshot::Sender<Result<(), RobotError>>)>,
            dead_tx: mpsc::Sender<()>) -> Controller {
        Controller {
            current_state: Mutex::new(ControllerState::Dead), //ControllerState::Dead,
            distance_queue: Mutex::new(VecDeque::new()), //VecDeque::new(),
            robot_queue: Mutex::new(VecDeque::new()), //VecDeque::new(),
            distance_time_queue: Mutex::new(VecDeque::new()), //VecDeque::new(),
            robot_time_queue: Mutex::new(VecDeque::new()),
            consecutive_errors: Mutex::new(0),
            pre_move_location: Mutex::new(None),
            arima: Mutex::new(None),
            distance_tx,
            state_tx,
            move_tx,
            dead_tx
        }
    }

    fn out_of_brain_uncalibrated(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::OutOfBrainUncalibrated
    }

    fn out_of_brain_calibrated(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::OutOfBrainCalibrated
    }

    fn in_panic(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::Panic
    }

    async fn clear_distance_queue(&self) {
        let mut distance_queue = self.distance_queue.lock().unwrap();
        let mut distance_time_queue = self.distance_time_queue.lock().unwrap();
        distance_queue.clear();
        distance_time_queue.clear();
    }

    //Prediction time is fine
    // fn get_move_location(&self, commanded_depth: u64) -> Option<u64> {
    //     let state_changer = self.robot_queue.lock().unwrap();
    //     let curr_front = state_changer.back();
    //     let Some(potential_location) = curr_front else{
    //         return None;
    //     };
    //     let Ok(location) = potential_location else{
    //         return None;
    //     };

    //     let brain_current = self.predict_brain_position(Duration::from_millis(OCT_LATENCY_MEAN));
    //     let brain_next = self.predict_brain_position(Duration::from_millis(OCT_LATENCY_MEAN + OCT_POLL_MILLIS));
    //     if brain_current.is_err() || brain_next.is_err() || brain_current.clone().unwrap() > MAX_DISTANCE_TO_BRAIN_MOVE{
    //         return None;
    //     }
    //     let latency ={(*self.distance_time_queue.lock().unwrap().back().unwrap() - *self.robot_time_queue.lock().unwrap().front().unwrap()).as_millis() as f64/self.distance_queue.lock().unwrap().len() as f64};
    //     if latency > MAX_LATENCY_MS as f64{
    //         return None;
    //     }
    //     let brain_current = brain_current.unwrap() + commanded_depth;
    //     let brain_next = brain_next.unwrap() + commanded_depth;
    //     let brain_v = (brain_next as f64 - brain_current as f64) / latency as f64; // in nm/ms
    //     if brain_v.abs() > MAX_BRAIN_VEL_NM_S as f64 {
    //         return None;
    //     }
    //     let position_indicator = if location.needle_z < brain_current {1.0} else {-1.0};
    //     let brain_acceleration = NEEDLE_ACCELERATION_NM_MS as f64 * position_indicator;
    //     let time_to_reach = (brain_v + position_indicator*(brain_v*brain_v - 4.0*brain_acceleration*(location.needle_z as f64 - brain_current as f64)).sqrt())/(2.0*brain_acceleration);
    //     assert!(time_to_reach < 1000.0 && time_to_reach > 0.0, "Time to reach: {} {} {} {} velocity: {}", time_to_reach, brain_current, location.needle_z, location.inserter_z, brain_v);
    //     if time_to_reach > MAX_TIME_TO_REACH_DEPTH as f64{
    //         return None;
    //     }
    //     let relative_position = brain_current as f64 + brain_v*time_to_reach;
    //     Some(relative_position as u64)
    // }

    fn _get_taylor_coefs(data: &Vec<u64>, n: u64, latency: f64) -> Vec<f64>{
        assert!(n > 0 && n <= data.len() as u64);
        let mut current = data.iter().map(|&x| x as i64).collect::<Vec<i64>>();
        let mut coefs = Vec::new();
        coefs.push(current[current.len() - 1] as f64);
        let mut factorial = 1;

        // Apply the backward difference n times
        for i in 0..n {
            // Compute the backward difference: Δf(x) = f(x) - f(x-1)
            factorial *= i+1;
            let next = current
                .windows(2)
                .map(|w| (w[1] - w[0]) / latency as i64) // (w[1] - w[0])
                .collect::<Vec<i64>>();
            coefs.push(next[next.len() - 1] as f64 / factorial as f64);
            current = next;
        }
        return coefs;
    }

    fn get_move_location(&self, commanded_depth: u64) -> Option<u64> {
        let queue = self.distance_queue.lock().unwrap();
        let time_queue = self.distance_time_queue.lock().unwrap();
        const data_len: usize = TAYLOR_POLY_ORDER as usize+1;
        if queue.len() < data_len{
            return None;
        }
        let latency = (*time_queue.back().unwrap() - *time_queue.front().unwrap()).as_millis() as f64/queue.len() as f64;
        if latency > MAX_LATENCY_MS as f64{
            return None;
        }
        let mut data = Vec::from(queue.clone());
        let Some(new_data) = data.last_chunk_mut::<data_len>() else{
            return None;
        };
        let mut data = Vec::new();
        for i in 0..new_data.len(){
            if let Ok(x) = new_data[i]{
                data.push(x);
            } else{
                return None;
            }
        }
        let coefs = Self::_get_taylor_coefs(&data, TAYLOR_POLY_ORDER, latency);
        let roots = find_roots_quartic(coefs[4], coefs[3], coefs[2] - NEEDLE_ACCELERATION_NM_MS as f64/2.0, coefs[1], coefs[0]);
        let pos = |x: f64| coefs[0] + coefs[1]*x + coefs[2]*x*x + coefs[3]*x*x*x + coefs[4]*x*x*x*x;
        match roots{
            Roots::No(_) => None,
            Roots::One(x) => {
                assert!(x[0] > 0.0);
                Some(pos(x[0]) as u64 + commanded_depth)
            },            
            Roots::Two(x) => {
                assert!(f64::max(x[0], x[1]) > 0.0);
                Some(pos(f64::max(x[0], x[1])) as u64 + commanded_depth)
            },
            Roots::Three(mut x) => {
                x.sort_by(|a,b| a.partial_cmp(b).unwrap());
                for i in 0..2{
                    if x[i] > 0.0{
                        return Some(pos(x[i]) as u64 + commanded_depth);
                    }
                }
                return None;
            },
            Roots::Four(mut x) => {
                x.sort_by(|a,b| a.partial_cmp(b).unwrap());
                for i in 0..2{
                    if x[i] > 0.0{
                        return Some(pos(x[i]) as u64 + commanded_depth);
                    }
                }
            return None;
            }   
        }
    }

    fn predict_brain_position_local(&self, future_duration: Duration) -> Result<u64, OCTError> {
        assert!(future_duration < Duration::from_millis(MAX_LATENCY_MS));
        let queue = self.distance_queue.lock().unwrap();
        let time_queue = self.distance_time_queue.lock().unwrap();
        if queue.len() < 2{
            return Err(OCTError::PredictionError { msg: "Queue too small".to_string() });
        }
        if future_duration > Duration::from_millis(MAX_LATENCY_MS){
            return Err(OCTError::PredictionError { msg: "Future duration too large".to_string() });
        }
        let length = queue.len();
        let current = queue.get(length-1).unwrap();
        let previous = queue.get(length-2).unwrap();
        if current.is_err() || previous.is_err() {
            return Err(OCTError::PredictionError { msg: "Queue error".to_string() });
        }
        let current_time = time_queue.get(length-1).unwrap();
        let previous_time = time_queue.get(length-2).unwrap();
        let time_diff = current_time.duration_since(*previous_time);
        let time_diff = time_diff.as_millis() as f64;
        let distance_diff = current.clone().unwrap() as f64 - previous.clone().unwrap() as f64;
        let velocity = distance_diff / time_diff;
        let elapsed_time = (current_time.elapsed() + future_duration).as_millis() as f64;
        let position = current.clone().unwrap() as f64 + velocity*elapsed_time;
        if position < 0.0{
            return Err(OCTError::PredictionError { msg: "Prediction went negative".to_string() });
        }
        Ok(position as u64)
    }

    fn is_abnormal_distance(&self, distance: u64) -> bool {
        let prediction = self.predict_brain_position_local(Duration::from_millis(OCT_POLL_MILLIS));
        if prediction.is_err(){
            return true;
        }
        let prediction = prediction.unwrap();
        let distance = distance as f64;
        let prediction = prediction as f64;
        let diff = distance - prediction;
        let diff = diff.abs();
        if diff > MAX_PREDICTION_ERROR_NM as f64{
            println!("ABNORMAL DISTANCE: {}", distance);
            {
                let distance_q = self.distance_queue.lock().unwrap();
                let time_q = self.distance_time_queue.lock().unwrap();
                println!("Previous State: {}, Recent State: {}, Prediction: {}, Time since recent state: {}, Actual distance: {} Current State: {}",
                    distance_q.get(distance_q.len() - 2).unwrap().clone().unwrap(),
                    distance_q.get(distance_q.len() - 1).unwrap().clone().unwrap(),
                    prediction,
                    time_q.get(time_q.len() - 1).unwrap().elapsed().as_millis(),
                    distance,
                    self.current_state.lock().unwrap()
                );
            }
        }
        return diff > MAX_PREDICTION_ERROR_NM as f64;
    }

}

fn die(control_state: Arc<Controller>) {
    let mut state_changer = control_state.current_state.lock().unwrap();
    *state_changer = ControllerState::Dead;
}

async fn poll_surface_distance(control_state: Arc<Controller>, tx: mpsc::Sender<Result<u64, OCTError>>) {
    loop {
        let tx_clone = tx.clone();
        // Spawn a task to fetch the distance without blocking the polling loop
        let distance = control_state.get_surface_distance().await;
        tokio::spawn({ 
            async move {
            let temp = distance.clone();
            if tx_clone.send(temp).await.is_err() {
                println!("Receiver dropped, stopping polling.");
            }
        }});
        // Wait for 5 seconds before polling again
        sleep(Duration::from_millis(OCT_POLL_MILLIS)).await;
    }
}
async fn process_distances(control_state: Arc<Controller>, mut rx: mpsc::Receiver<Result<u64, OCTError>>) {
    while let Some(distance_result) = rx.recv().await {
        // {
        //     let mut queue = control_state.distance_time_queue.lock().unwrap();
        //     if queue.len() > 0 && queue.back().unwrap().elapsed().as_millis() > MAX_LATENCY_MS as u128 {
        //         distance_result = Err(OCTError::TimeoutError{ msg: "Latency error".to_string() });
        //     }
        // }
        match distance_result {
            Ok(distance) => {
                //println!("Distance: {}", distance);
                let current_state = control_state.current_state.lock().unwrap().clone();
                let can_panic = current_state == ControllerState::OutOfBrainCalibrated || current_state == ControllerState::InBrain;
                // Check for abnormal distance
                let too_close_to_brain = distance < MIN_DISTANCE_BRAIN_TO_ARM_NM;
                if too_close_to_brain && can_panic {
                    println!("Too close to brain");
                    transition_state(control_state.clone(), ControllerState::Panic, false);
                }
                else if can_panic && control_state.is_abnormal_distance(distance) {
                    let mut consecutive_errors = control_state.consecutive_errors.lock().unwrap();
                    *consecutive_errors += 1;

                    let max_errors = MAX_CONSECUTIVE_PREDICTION_ERRORS.try_into().unwrap();

                    if *consecutive_errors > max_errors && can_panic
                    {
                        println!("Too many consecutive errors");
                        assert!(current_state != ControllerState::Panic);
                        transition_state(control_state.clone(), ControllerState::Panic, false);
                    }
                } else {
                    let mut consecutive_errors = control_state.consecutive_errors.lock().unwrap();
                    *consecutive_errors = 0;
                }
            }
            Err(_) => {
                println!("Received error in processing distance");
            }
        };

        // Update queues
        {
            let mut queue = control_state.distance_queue.lock().unwrap();
            queue.push_back(distance_result);
            let expected_length = if control_state.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_DISTANCES};
            while queue.len() > expected_length.try_into().unwrap() {
                queue.pop_front();
            }
        }
        {
            let mut time_queue = control_state.distance_time_queue.lock().unwrap();
            time_queue.push_back(Instant::now());
            let expected_length = if control_state.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_DISTANCES};
            while time_queue.len() > expected_length.try_into().unwrap() {
                time_queue.pop_front();
            }
        }
        tokio::task::yield_now().await;
    }
}
async fn process_robot_state(control_state: Arc<Controller>) {
    loop{
        let robot_state = {
            control_state.get_robot_state().await
        };
        match robot_state {
            Ok(_) => {
                //println!("Robot state: {:?}", r);
            }
            Err(RobotError::ConnectionError{..}) | Err(RobotError::MoveError{..}) => {
                println!("Received error in processing robot state");
            }
            Err(RobotError::PositionError{..}) => {
                die(control_state.clone());
            }
        };
        {
            let mut queue = control_state.robot_queue.lock().unwrap();
            queue.push_back(robot_state); //robot_state);
            let expected_length = if control_state.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_STATES};
            while queue.len() > expected_length.try_into().unwrap() {
                queue.pop_front();
            }

            let mut queue = control_state.robot_time_queue.lock().unwrap();
            queue.push_back(Instant::now()); //robot_state);
            while queue.len() > expected_length.try_into().unwrap() {
                queue.pop_front();
            }
        }
        sleep(Duration::from_millis(ROBOT_STATE_POLL_MILLIS)).await;
    }
}

async fn panic(control_state: Arc<Controller>) {
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::Panic, false).await;
    move_bot(control_state.clone(), &Move::InserterZ(0), ControllerState::Panic, false).await;
    transition_state(control_state,ControllerState::OutOfBrainUncalibrated, true);
}

async fn calibrate(control_state: Arc<Controller>) {
    assert!({
        control_state.get_robot_state().await.unwrap() == RobotState{inserter_z: 0, needle_z: 0} &&
        control_state.out_of_brain_uncalibrated()
    });
    println!("Out of assert in calibrate");
    let calibration_init = Instant::now();
    control_state.clear_distance_queue().await;
    {
        *control_state.consecutive_errors.lock().unwrap() = 0;
        *control_state.pre_move_location.lock().unwrap() = None;
        *control_state.arima.lock().unwrap() = None;
    }
    loop{
        {
            let distance_queue = control_state.distance_queue.lock().unwrap();
            let distance_time_queue = control_state.distance_time_queue.lock().unwrap();
            if distance_queue.len() >= CALIBRATION_SAMPLES.try_into().unwrap() && distance_queue.front().unwrap().is_ok() && *distance_time_queue.front().unwrap() >= calibration_init {
                let min_distance = distance_queue.iter().filter(|d| d.is_ok()).min_by_key(|d| d.as_ref().unwrap()).unwrap().as_ref().unwrap();
                let mut guard = control_state.pre_move_location.lock().unwrap();
                assert!(*min_distance > MIN_DISTANCE_BRAIN_TO_ARM_NM);
                *guard = Some(*min_distance - MIN_DISTANCE_BRAIN_TO_ARM_NM);
                break;
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    move_bot(control_state.clone(), &Move::InserterZ(control_state.pre_move_location.lock().unwrap().unwrap()), ControllerState::OutOfBrainUncalibrated, false).await;
    control_state.clear_distance_queue().await;
    loop{
        {
            let distance_queue = control_state.distance_queue.lock().unwrap();
            let distance_time_queue = control_state.distance_time_queue.lock().unwrap();
            let mut arima = control_state.arima.lock().unwrap();
            if distance_queue.len() >= CALIBRATION_SAMPLES.try_into().unwrap() && distance_queue.front().unwrap().is_ok() && *distance_time_queue.front().unwrap() >= calibration_init {
                let mut trained_arima = ARIMA::new(MIN_CALIBRATION_TRAINING_POINTS);
                println!("Training ARIMA");
                let success = trained_arima.train_u64(&distance_queue);
                if success {
                    *arima = Some(trained_arima);
                    break;   
                }
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
    control_state.clear_distance_queue().await;
    println!("---------------------------------------------------------------------------------------------------------------------------------------");
}

pub async fn start(control_state: Arc<Controller>, commanded_depth: &Vec<u64>) {
    println!("Starting controller...");
    assert!(OCT_LATENCY_MEAN/OCT_POLL_MILLIS > 1);
    let (tx, rx) = mpsc::channel::<Result<u64, OCTError>>(20);
    tokio::spawn({let me = Arc::clone(&control_state);
    async move {
        poll_surface_distance(me, tx).await;
    }});
    println!("Starting to process distances...");
    tokio::spawn({let me = Arc::clone(&control_state);
        async move {
            process_distances(me, rx).await;
        }});
    println!("Starting to process robot state...");
    tokio::spawn({let me = Arc::clone(&control_state);
        async move {
            process_robot_state(me).await;
        }});
    {
        let mut state_changer = control_state.current_state.lock().unwrap();
        *state_changer = ControllerState::OutOfBrainUncalibrated;
    }
    let mut outcomes = Vec::<bool>::new();
    for (_i, depth) in commanded_depth.iter().enumerate() {
        if control_state.in_panic(){
            panic(control_state.clone()).await;
        }
        if control_state.out_of_brain_uncalibrated(){
            calibrate(control_state.clone()).await;
            println!("Calibrated");
        }
        assert!(control_state.out_of_brain_calibrated());
        outcomes.push(insert_ib_open_loop(control_state.clone(), *depth).await);
    }
    transition_state(control_state.clone(), ControllerState::Dead, false);
    println!("Done");
    control_state.dead_tx.send(()).await.unwrap();
}

async fn retract_ib(control_state: Arc<Controller>) {
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
}

async fn insert_ib_open_loop(control_state: Arc<Controller>, commanded_depth: u64) -> bool {
    assert!(commanded_depth >= COMMANDED_DEPTH_MIN_NM && commanded_depth <= COMMANDED_DEPTH_MAX_NM);
    let pos = control_state.get_robot_state().await.unwrap();
    assert!(pos.needle_z == 0 && pos.inserter_z == control_state.pre_move_location.lock().unwrap().unwrap(), "Needle not at zero, instead at: {:?}", pos);
    assert!(control_state.arima.lock().unwrap().is_some() && control_state.arima.lock().unwrap().as_ref().unwrap().is_trained(), "ARIMA not trained {}", control_state.arima.lock().unwrap().is_some());
    let init_time = Instant::now();
    while !control_state.in_panic() && Instant::now().duration_since(init_time).as_millis() < MAX_IB_TIME.into() {
        let Some(relative_position) = control_state.get_move_location(commanded_depth) else{
            sleep(Duration::from_millis(5)).await;
            continue;
        };
        assert!(relative_position > 0 && relative_position < 10_000_000, "Invalid position: {}", relative_position);
        let response = {
            control_state.command_move(&Move::NeedleZ(relative_position)).await
        };
        match response {
            Ok(_) => {
                retract_ib(control_state.clone()).await;
                return true;
            }
            Err(RobotError::MoveError{..}) | Err(RobotError::ConnectionError{..}) => {
                println!("Error in moving to position: {}", relative_position);
            }
            Err(RobotError::PositionError{..}) => {
                die(control_state.clone());
            }
        }
        tokio::task::yield_now().await;
    }
    if control_state.in_panic(){
        panic(control_state.clone()).await;
    }else{
        retract_ib(control_state.clone()).await;
    }
    return false;
}

async fn move_bot(control_state: Arc<Controller>, command: &Move, next_state: ControllerState, from_panic: bool) -> () {
    loop {
        let response = 
            control_state.command_move(command).await;
        //print!("THERES NO DEADLOCK");
        match response {
            Ok(_) => {
                break;
            }
            Err(RobotError::MoveError{..}) | Err(RobotError::ConnectionError{..}) => {
                println!("Error in moving to position: {}", command);
            }
            Err(RobotError::PositionError{..}) => {
                die(control_state.clone());
            }
        }
        tokio::task::yield_now().await;
    }
    transition_state(control_state,next_state, from_panic);
}

fn transition_state(control_state: Arc<Controller>, next_state: ControllerState, from_panic: bool) {
    let mut state_changer = control_state.current_state.lock().unwrap();
    let mut  can_change = *state_changer != ControllerState::Panic || from_panic;
    can_change = can_change && *state_changer != ControllerState::Dead;
    if !can_change {
        println!("Cannot change state from {} to {}", state_changer, next_state);
        return;
    }
    *state_changer = next_state;
}

impl Robot for Controller{

    async fn command_grasp(& self) -> Result<(), RobotError> {
       return Ok(());
    }
    
    async fn command_move(& self, move_type: &Move) -> Result<(), RobotError> {
        loop{
            let (tx, rx) = oneshot::channel();
            match self.move_tx.send((move_type.clone(), tx)).await{
                Ok(_) => return rx.await.unwrap(),
                Err(_) => {}
            }
        };
    }
    async fn get_robot_state(& self) -> Result<RobotState, RobotError> {
        loop{
            let (tx, rx) = oneshot::channel();
            match self.state_tx.send(((), tx)).await{
                Ok(_) => return rx.await.unwrap(),
                Err(_) => {}
            }
        };
    }

}

impl OCTService for Controller{
    
    async fn get_surface_distance(& self) -> Result<u64, OCTError> {
        loop{
            let (tx, rx) = oneshot::channel();
            match self.distance_tx.send(((), tx)).await{
                Ok(_) => return rx.await.unwrap(),
                Err(_) => {}
            }
        };
    }
}