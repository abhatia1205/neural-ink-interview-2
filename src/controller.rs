use crate::interface::{RobotError, RobotState, OCTService, OCTError, Move, Robot}; //OCTError;
use tokio::sync::mpsc;
use tokio::time::{sleep, Duration, Instant};
use crate::robot_chat::RobotArm;
use std::f32::MIN;
// use core::num;
// use std::os::macos::raw::stat;
use std::sync::{Mutex,Arc}; //Arc;
use std::collections::VecDeque;
use crate::arima::ARIMA;

const CALIBRATION_SAMPLES: u64 = 500;
const MIN_CALIBRATION_TRAINING_POINTS: u64 = 600;
const MIN_DISTANCE_BRAIN_TO_ARM_NM: u64 = 200_000;
const MAX_DISTANCES: u64 = 100;
const MAX_STATES: u64 = 100;
// const MAX_OOB_STALL_TIME: u64 = 30_000;
const MAX_IB_TIME: u64 = 1_000;
// const NUM_POINTS_TILL_STATONARY: u64 = 30;
const MAX_CONSECUTIVE_PREDICTION_ERRORS: u64 = 5;
// const MAX_BRAIN_ACC_FOR_CONST_NM_S2: u64 = 1_000_000*100;
// const HARDCODED_PRE_MOVE_LOCATION: u64 = 3_000_000; //TODO
const MAX_PREDICTION_ERROR_NM: u64 = 1_000_000;

const OCT_POLL_MILLIS: u64 = 5;
const OCT_LATENCY_MEAN: u64 = 15;
const ROBOT_STATE_POLL_MILLIS: u64 = 5;
const NEEDLE_ACCELERATION_NM_MS: i64 = 250;
const NEEDLE_VELOCITY_NM_MS: u64 = 250_000;
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
    robot: tokio::sync::Mutex<RobotArm>, //Box<RobotArm>,
    consecutive_errors: Mutex<u64>,
    pre_move_location: Mutex<Option<u64>>, //u64
    arima: Mutex<Option<ARIMA>>, //ARIMA>
}

impl<'a> Controller{
    pub fn new() -> Controller {
        Controller {
            current_state: Mutex::new(ControllerState::Dead), //ControllerState::Dead,
            distance_queue: Mutex::new(VecDeque::new()), //VecDeque::new(),
            robot_queue: Mutex::new(VecDeque::new()), //VecDeque::new(),
            robot: tokio::sync::Mutex::new(RobotArm::new()),
            distance_time_queue: Mutex::new(VecDeque::new()), //VecDeque::new(),
            robot_time_queue: Mutex::new(VecDeque::new()),
            consecutive_errors: Mutex::new(0),
            pre_move_location: Mutex::new(None),
            arima: Mutex::new(None),
        }
    }

    fn reset(&mut self){
        *self.current_state.lock().unwrap() = ControllerState::OutOfBrainUncalibrated;
        *self.distance_queue.lock().unwrap() = VecDeque::new();
        *self.robot_queue.lock().unwrap() = VecDeque::new();
        *self.distance_time_queue.lock().unwrap() = VecDeque::new();
        *self.robot_time_queue.lock().unwrap() = VecDeque::new();
        *self.consecutive_errors.lock().unwrap() = 0;
        *self.pre_move_location.lock().unwrap() = None;
        *self.arima.lock().unwrap() = None;
    }

    fn in_brain(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::InBrain
    }

    fn out_of_brain(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::OutOfBrainCalibrated || *state_changer == ControllerState::OutOfBrainUncalibrated
    }

    fn out_of_brain_uncalibrated(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::OutOfBrainUncalibrated
    }

    fn out_of_brain_calibrated(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::OutOfBrainCalibrated
    }

    fn is_dead(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::Dead
    }

    fn in_panic(&self) -> bool {
        let state_changer = self.current_state.lock().unwrap();
        *state_changer == ControllerState::Panic
    }

    fn is_constant_speed(&self) -> bool {
        true
    }

    //Prediction time is fine
    fn get_move_location(&self, commanded_depth: u64) -> Option<u64> {
        let state_changer = self.robot_queue.lock().unwrap();
        let curr_front = state_changer.back();
        let Some(potential_location) = curr_front else{
            return None;
        };
        let Ok(location) = potential_location else{
            return None;
        };

        let brain_current = self.predict_brain_position(OCT_LATENCY_MEAN/OCT_POLL_MILLIS);
        let brain_next = self.predict_brain_position(OCT_LATENCY_MEAN/OCT_POLL_MILLIS + 1);
        if brain_current.is_err() || brain_next.is_err() {
            return None;
        }
        let latency ={(*self.distance_time_queue.lock().unwrap().back().unwrap() - *self.robot_time_queue.lock().unwrap().front().unwrap()).as_millis() as f64/self.distance_queue.lock().unwrap().len() as f64};
        //assert!(location.inserter_z < brain_current.clone().unwrap(), "{} {}", location.inserter_z, brain_current.clone().unwrap());
        //let needle_location = location.needle_z + location.inserter_z;
        let brain_current = brain_current.unwrap() + commanded_depth;
        let brain_next = brain_next.unwrap() + commanded_depth;
        let brain_v = (brain_next as f64 - brain_current as f64) / latency as f64; // in nm/ms
        let position_indicator = if location.needle_z < brain_current {1.0} else {-1.0};
        let brain_acceleration = NEEDLE_ACCELERATION_NM_MS as f64 * position_indicator;
        let time_to_reach = (brain_v + position_indicator*(brain_v*brain_v - 4.0*brain_acceleration*(location.needle_z as f64 - brain_current as f64)).sqrt())/(2.0*brain_acceleration);
        //println!( "Time to reach: {} {} {} {} {}", time_to_reach, brain_current, location.needle_z, location.inserter_z, brain_v);
        assert!(time_to_reach < 1000.0, "Time to reach: {} {} {} {} velocity: {}", time_to_reach, brain_current, location.needle_z, location.inserter_z, brain_v);
        let relative_position = brain_current as f64 + brain_v*time_to_reach;
        //println!("Relative position: {}", relative_position);
        Some(relative_position as u64)
    }

    fn predict_brain_position(&self, num_periods: u64) -> Result<u64, OCTError> {
        let queue = self.distance_queue.lock().unwrap();
        if queue.len() < 2{
            return Err(OCTError::PredictionError { msg: "Queue too small".to_string() });
        }
        let length = queue.len();
        return self._predict_brain_position(queue.get(length-1).unwrap(), queue.get(length-2).unwrap(), num_periods);
    }

    fn _predict_brain_position(&self, lag1: &Result<u64, OCTError>, lag2: &Result<u64, OCTError>, num_periods: u64) -> Result<u64, OCTError> {
        if lag1.is_err() {
            return Err(lag1.clone().unwrap_err());
        }
        if lag2.is_err() {
            return Err(lag2.clone().unwrap_err());
        }
        assert!(self.arima.lock().unwrap().as_ref().unwrap().is_trained());
        let prediction = self.arima.lock().unwrap().as_ref().unwrap().predict(lag1.clone().unwrap() as f64, lag2.clone().unwrap() as f64).unwrap(); //TODO
        let prediction = Ok(prediction as u64);
        if num_periods == 1{
            return prediction;
        }
        return self._predict_brain_position(&prediction, lag1, num_periods-1);
    }

    fn is_abnormal_distance(&self, distance: u64) -> bool {
        let prediction = self.predict_brain_position( 1);
        if prediction.is_err(){
            return true;
        }
        let prediction = prediction.unwrap();
        let distance = distance as f64;
        let prediction = prediction as f64;
        let diff = distance - prediction;
        let diff = diff.abs();
        if diff > MAX_PREDICTION_ERROR_NM as f64{
            //println!("ABNORMAL DISTANCE: {}", distance);
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
        let distance = {
            let guard: tokio::sync::MutexGuard<'_, RobotArm> = control_state.robot.lock().await;
            guard.get_surface_distance().await
        };
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
        match distance_result {
            Ok(distance) => {
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
            if(time_queue.len() > 0){
                println!("{}",time_queue.back().unwrap().elapsed().as_millis());   
            }
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
            control_state.robot.lock().await.get_robot_state().await
        };
        match robot_state {
            Ok(_) => {}
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
    //println!("SINCE PANICKED< MOVING BACK TO ORIGIN");
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::Panic, false).await;
    //println!("SINCE PANICKED CONTINUING MOVING BACK TO ORIGIN");
    move_bot(control_state.clone(), &Move::InserterZ(0), ControllerState::Panic, false).await;
    transition_state(control_state,ControllerState::OutOfBrainUncalibrated, true);
}

async fn calibrate(control_state: Arc<Controller>) {
    assert!({
        control_state.robot.lock().await.get_robot_state().await.unwrap() == RobotState{inserter_z: 0, needle_z: 0} &&
        control_state.out_of_brain_uncalibrated()
    });
    let calibration_init = Instant::now();
    {
        let mut queue = control_state.distance_queue.lock().unwrap();
        let mut time_queue = control_state.distance_time_queue.lock().unwrap();
        queue.clear();
        time_queue.clear();
        *control_state.consecutive_errors.lock().unwrap() = 0;
        *control_state.pre_move_location.lock().unwrap() = None;
        *control_state.arima.lock().unwrap() = None;
    }
    loop{
        //println!("Calibrating");
        {
            let distance_queue = control_state.distance_queue.lock().unwrap();
            let distance_time_queue = control_state.distance_time_queue.lock().unwrap();
            if distance_queue.len() >= CALIBRATION_SAMPLES.try_into().unwrap() && distance_queue.front().unwrap().is_ok() && *distance_time_queue.front().unwrap() >= calibration_init {
                break;
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    {
        let distance_queue = control_state.distance_queue.lock().unwrap();
        let min_distance = distance_queue.iter().filter(|d| d.is_ok()).min_by_key(|d| d.as_ref().unwrap()).unwrap().as_ref().unwrap();
        let mut guard = control_state.pre_move_location.lock().unwrap();
        assert!(*min_distance > MIN_DISTANCE_BRAIN_TO_ARM_NM);
        *guard = Some(*min_distance - MIN_DISTANCE_BRAIN_TO_ARM_NM);
    }
    move_bot(control_state.clone(), &Move::InserterZ(control_state.pre_move_location.lock().unwrap().unwrap()), ControllerState::OutOfBrainUncalibrated, false).await;
    {
        let mut queue = control_state.distance_queue.lock().unwrap();
        let mut time_queue = control_state.distance_time_queue.lock().unwrap();
        queue.clear();
        time_queue.clear();
    }
    loop{
        //println!("Calibrating");
        {
            let distance_queue = control_state.distance_queue.lock().unwrap();
            let distance_time_queue = control_state.distance_time_queue.lock().unwrap();
            if distance_queue.len() >= CALIBRATION_SAMPLES.try_into().unwrap() && distance_queue.front().unwrap().is_ok() && *distance_time_queue.front().unwrap() >= calibration_init {
                break;
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    {
        let mut queue = control_state.distance_queue.lock().unwrap();
        let mut arima = control_state.arima.lock().unwrap();
        let mut trained_arima = ARIMA::new(MIN_CALIBRATION_TRAINING_POINTS);
        trained_arima.train_u64(&queue);
        *arima = Some(trained_arima);
    }
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
    {
        let mut queue = control_state.distance_queue.lock().unwrap();
        let mut time_queue = control_state.distance_time_queue.lock().unwrap();
        queue.clear();
        time_queue.clear();
    }
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
        //println!("Looping back for next insert");
    }
    transition_state(control_state, ControllerState::Dead, false);
}

async fn retract_ib(control_state: Arc<Controller>) {
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
}

async fn insert_ib_open_loop(control_state: Arc<Controller>, commanded_depth: u64) -> bool {
    assert!(commanded_depth >= COMMANDED_DEPTH_MIN_NM && commanded_depth <= COMMANDED_DEPTH_MAX_NM);
    let needle_pos = {control_state.robot.lock().await.get_robot_state().await.unwrap().needle_z};
    assert!(needle_pos == 0, "Needle not at zero, instead at: {}", needle_pos);
    let init_time = Instant::now();
    while !control_state.in_panic() && Instant::now().duration_since(init_time).as_millis() < MAX_IB_TIME.into() {
        let Some(relative_position) = control_state.get_move_location(commanded_depth) else{
            tokio::task::yield_now().await;
            continue;
        };
        assert!(relative_position > 0 && relative_position < 10_000_000, "Invalid position: {}", relative_position);
        let response = {
            control_state.robot.lock().await.command_move(&Move::NeedleZ(relative_position)).await
        };
        match response {
            Ok(_) => {
                //println!("Moved to position: {}", relative_position);
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
        let response = {
            control_state.robot.lock().await.command_move(command).await
        };
        //print!("THERES NO DEADLOCK");
        match response {
            Ok(_) => {
                //println!("Moved to position: {}", command);
                break;
            }
            Err(RobotError::MoveError{..}) | Err(RobotError::ConnectionError{..}) => {
                println!("Error in moving to position: {}", command);
            }
            Err(RobotError::PositionError{..}) => {
                die(control_state.clone());
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    transition_state(control_state,next_state, from_panic);
}

fn transition_state(control_state: Arc<Controller>, next_state: ControllerState, from_panic: bool) {
    let mut state_changer = control_state.current_state.lock().unwrap();
    //println!("Coming from panic: {} {} {}=-----------------------------------------------", *state_changer, next_state, from_panic);
    let mut  can_change = *state_changer != ControllerState::Panic || from_panic;
    can_change = can_change && *state_changer != ControllerState::Dead;
    if !can_change {
        println!("Cannot change state from {} to {}", state_changer, next_state);
        return;
    }
    *state_changer = next_state;
}