use crate::controller;
use crate::interface::{RobotError, RobotState, OCTService, OCTError, Move, Robot};
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, Duration, Instant};
use std::cell::RefCell;
use std::sync::{Mutex,Arc};
use std::collections::VecDeque;
use roots::find_root_brent;
use roots::SimpleConvergency;
use crate::predictor::{self, BrainPredictor};
use crate::predictor::MIN_DISTANCE_BRAIN_TO_ARM_NM;

const CALIBRATION_SAMPLES: u64 = 500;
const MAX_DISTANCES: u64 = 100;
const MAX_STATES: u64 = 100;
const MAX_IB_TIME: u64 = 30_000; // HAS TO CHANGE
const MAX_CONSECUTIVE_PREDICTION_ERRORS: u64 = 20;
const MAX_PREDICTION_ERROR_NM: u64 = 50_000;

const OCT_POLL_MILLIS: u64 = 5;
const OCT_LATENCY_MS: u64 = 15;
const ROBOT_STATE_POLL_MILLIS: u64 = 5;
const NEEDLE_ACCELERATION_NM_MS: i64 = 250;
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

enum InBrainOutcome{
    Success,
    Failure,
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


pub struct ControllerInfo{
    current_state: ControllerState, //ControllerState,
    distance_queue: VecDeque<Result<u64, OCTError>>, //VecDeque<(Result<u64, OCTError>, Instant>>>,
    distance_time_queue: VecDeque<Instant>,
    robot_queue: VecDeque<Result<RobotState, RobotError>>, //VecDeque<(Result<RobotState, RobotError>, Instant>>>,
    robot_time_queue: VecDeque<Instant>,
    consecutive_errors: u64,
    pre_move_location: Option<u64>, //u64
    pub outcomes: Vec<bool>
}

impl ControllerInfo{
    fn clear_distance_queue(&mut self) {
        self.distance_queue.clear();
        self.distance_time_queue.clear();
    }
}

struct Controller<P: BrainPredictor>{
    info: RefCell<ControllerInfo>,
    distance_tx: mpsc::Sender<((), oneshot::Sender<Result<u64, OCTError>>)>,
    state_tx: mpsc::Sender<((), oneshot::Sender<Result<RobotState, RobotError>>)>,
    move_tx: mpsc::Sender<(Move, oneshot::Sender<Result<(), RobotError>>)>,
    dead_tx: mpsc::Sender<()>,
    predictor: P
}

impl<P: BrainPredictor> Controller<P>{

    fn new(distance_tx: mpsc::Sender<((), oneshot::Sender<Result<u64, OCTError>>)>,
    state_tx: mpsc::Sender<((), oneshot::Sender<Result<RobotState, RobotError>>)>,
    move_tx: mpsc::Sender<(Move, oneshot::Sender<Result<(), RobotError>>)>,
    dead_tx: mpsc::Sender<()>, predictor: P) -> Controller<P>{
        Controller{
            info: RefCell::new(ControllerInfo{
                current_state: ControllerState::Dead, //ControllerState::Dead,
                distance_queue: VecDeque::new(), //VecDeque::new(),
                robot_queue: VecDeque::new(), //VecDeque::new(),
                distance_time_queue: VecDeque::new(), //VecDeque::new(),
                robot_time_queue: VecDeque::new(),
                consecutive_errors: 0,
                pre_move_location: None,
                outcomes:Vec::new()
            }),
            distance_tx,
            state_tx,
            move_tx,
            dead_tx,
            predictor
        }
    }

    fn out_of_brain_uncalibrated(&self) -> bool {
        let info = self.info.borrow();
        info.current_state == ControllerState::OutOfBrainUncalibrated
    }

    fn out_of_brain_calibrated(&self) -> bool {
        let info = self.info.borrow();
        info.current_state == ControllerState::OutOfBrainCalibrated
    }

    fn in_panic(&self) -> bool {
        let info = self.info.borrow();
        info.current_state == ControllerState::Panic
    }

    fn clear_distance_queue(&self) {
        let mut info = self.info.borrow_mut();
        info.clear_distance_queue();
    }

    fn get_move_location(&self, commanded_depth: u64) -> Option<u64> {
        let info = self.info.borrow_mut();
        let distances = Vec::from(info.distance_queue.clone());
        let times = Vec::from(info.distance_time_queue.clone());
        let Some(brain_position_function) = self.predictor.predict(&distances, &times) else {
            return None;
        };
        let needle_pos = |x: f64| {NEEDLE_ACCELERATION_NM_MS as f64/4.0 * x * x};
        let intersection_fn = |x|{brain_position_function(x as f64) + commanded_depth as f64 - needle_pos(x as f64)};
        let furthest_needle_move = (2.0*COMMANDED_DEPTH_MAX_NM as f64/NEEDLE_ACCELERATION_NM_MS as f64).sqrt();
        let mut convergency = SimpleConvergency { eps:1e-15f64, max_iter:30 };
        let Ok(root) = find_root_brent(0.0, furthest_needle_move, &intersection_fn, &mut convergency) else{
            return None;
        };
        return Some(brain_position_function(root) as u64 + commanded_depth);
    }
    fn is_abnormal_distance(&self, distance: u64) -> bool {
        let info = self.info.borrow_mut();
        let distances = Vec::from(info.distance_queue.clone());
        let times = Vec::from(info.distance_time_queue.clone());
        let Some(brain_position_function) = self.predictor.predict(&distances, &times) else {
            return true;
        };
        let prediction = brain_position_function(info.distance_time_queue[info.distance_time_queue.len()-1].elapsed().as_millis() as f64);
        let diff = (distance as f64 - prediction).abs();
        if diff > MAX_PREDICTION_ERROR_NM as f64{
            println!("ABNORMAL DISTANCE:: Previous State: {}, Recent State: {}, Prediction: {}, Time since recent state: {}, Actual distance: {} Current State: {}",
                info.distance_queue.get(info.distance_queue.len() - 2).unwrap().clone().unwrap(),
                info.distance_queue.get(info.distance_queue.len() - 1).unwrap().clone().unwrap(),
                prediction,
                info.distance_time_queue.get(info.distance_time_queue.len() - 1).unwrap().elapsed().as_millis(),
                distance,
                info.current_state
            );
            return true;
        }
        return false;
    }

    fn get_recent_state(&self) -> Option<RobotState> {
        let info = self.info.borrow();
        let result = info.robot_queue.back();
        let Some(result) = result else {
            return None;
        };
        if result.is_err() {
            return None;
        }
        return Some(result.as_ref().unwrap().clone());
    }

    fn get_state(&self) -> ControllerState {
        let info = self.info.borrow();
        return info.current_state;
    }

    fn add_error(&self) {
        let mut info = self.info.borrow_mut();
        info.consecutive_errors += 1;
    }

    fn clear_error(&self) {
        let mut info = self.info.borrow_mut();
        info.consecutive_errors = 0;
    }

    fn get_consecutive_errors(&self) -> u64 {
        let info = self.info.borrow();
        return info.consecutive_errors;
    }

    fn get_pre_move_location(&self) -> Option<u64> {
        let info = self.info.borrow();
        return info.pre_move_location;
    }
    
}

fn die<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    let mut control_state = control_state.info.borrow_mut();
    control_state.current_state = ControllerState::Dead;

}

async fn poll_distance<P: BrainPredictor>(control_state: Arc<Controller<P>>, mut tx: mpsc::Sender<Result<u64, OCTError>>){
    loop{
        let tx_clone = tx.clone();
        let control_clone = control_state.clone();
        // Spawn a task to fetch the distance without blocking the polling loop
        tokio::spawn({ 
            async move {
            let distance = control_clone.get_surface_distance().await;
            let temp = distance.clone();
            if tx_clone.send(temp).await.is_err() {
                println!("Receiver dropped, stopping polling.");
            }
        }});
        // Wait for 5 seconds before polling again
        sleep(Duration::from_millis(OCT_POLL_MILLIS)).await;
    }
}

async fn process_distances<P: BrainPredictor>(control_state: Arc<Controller<P>>, mut rx: mpsc::Receiver<Result<u64, OCTError>>) {
    while let Some(distance_result) = rx.recv().await {
        match distance_result {
            Ok(distance) => {
                let current_state = control_state.get_state();
                let can_panic = current_state == ControllerState::OutOfBrainCalibrated || current_state == ControllerState::InBrain;
                // Check for abnormal distance
                let too_close_to_brain = distance < MIN_DISTANCE_BRAIN_TO_ARM_NM/2;
                if too_close_to_brain && can_panic {
                    println!("Too close to brain: {}", distance);
                    transition_state(control_state.clone(), ControllerState::Panic, false);
                }
                else if can_panic && control_state.is_abnormal_distance(distance) {
                    control_state.add_error();

                    if control_state.get_consecutive_errors() > MAX_CONSECUTIVE_PREDICTION_ERRORS && can_panic
                    {
                        println!("Too many consecutive errors");
                        assert!(control_state.get_state() != ControllerState::Panic);
                        transition_state(control_state.clone(), ControllerState::Panic, false);
                    }
                } else {
                    control_state.clear_error();
                }
            }
            Err(_) => {}
        };

        // Update queues
        {
            let expected_length = if control_state.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_DISTANCES};
            let mut controller = control_state.info.borrow_mut();
            controller.distance_queue.push_back(distance_result);
            while controller.distance_queue.len() > expected_length.try_into().unwrap() {
                controller.distance_queue.pop_front();
            }
            controller.distance_time_queue.push_back(Instant::now());
            while controller.distance_time_queue.len() > expected_length.try_into().unwrap() {
                controller.distance_time_queue.pop_front();
            }
        }
        tokio::task::yield_now().await;
    }
}
async fn process_robot_state<P: BrainPredictor>(control_state: Arc<Controller<P>>, mut rx: mpsc::Receiver<Result<RobotState, RobotError>>) {
    while let Some(robot_state) = rx.recv().await {
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
            let expected_length = if control_state.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_STATES};
            let mut controller = control_state.info.borrow_mut();
            controller.robot_queue.push_back(robot_state); //robot_state);
            while controller.robot_queue.len() > expected_length.try_into().unwrap() {
                controller.robot_queue.pop_front();
            }

            controller.robot_time_queue.push_back(Instant::now()); //robot_state);
            while controller.robot_time_queue.len() > expected_length.try_into().unwrap() {
                controller.robot_time_queue.pop_front();
            }
        }
        sleep(Duration::from_millis(ROBOT_STATE_POLL_MILLIS)).await;
    }
}

async fn panic<P: BrainPredictor>(control_state: &Controller<P>) {
    move_bot(control_state, &Move::NeedleZ(0), ControllerState::Panic, false).await;
    move_bot(control_state, &Move::InserterZ(0), ControllerState::Panic, false).await;
    transition_state(control_state,ControllerState::OutOfBrainUncalibrated, true);
}

async fn calibrate<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    loop{
        {
            let controller = control_state.info.borrow();
            if controller.robot_queue.back().is_some() {break;}
            sleep(Duration::from_millis(10));
        }
    }
    assert!(control_state.get_recent_state().unwrap() == RobotState{inserter_z: 0, needle_z: 0} && control_state.out_of_brain_uncalibrated());
    println!("Out of assert in calibrate");
    let calibration_init = Instant::now();
    {
        let mut controller = control_state.info.borrow_mut();
        control_state.clear_distance_queue();
        controller.consecutive_errors = 0;
        controller.pre_move_location = None;
    }
    loop{
        {
            let mut controller = control_state.info.borrow_mut();
            let distance_queue = &controller.distance_queue;
            let distance_time_queue = &controller.distance_time_queue;
            if distance_queue.len() >= CALIBRATION_SAMPLES.try_into().unwrap() && distance_queue.front().unwrap().is_ok() && *distance_time_queue.front().unwrap() >= calibration_init {
                let min_distance = distance_queue.iter().filter(|d| d.is_ok()).min_by_key(|d| d.as_ref().unwrap()).unwrap().as_ref().unwrap();
                assert!(*min_distance > MIN_DISTANCE_BRAIN_TO_ARM_NM);
                controller.pre_move_location = Some(*min_distance - MIN_DISTANCE_BRAIN_TO_ARM_NM);
                break;
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    let premove_location = control_state.info.borrow().pre_move_location.unwrap();
    move_bot(control_state.clone(), &Move::InserterZ(premove_location), ControllerState::OutOfBrainUncalibrated, false).await;
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
    {control_state.clone().clear_distance_queue()};
    println!("---------------------------------------------------------------------------------------------------------------------------------------");
}

pub async fn start<P: BrainPredictor>(control_state: Arc<RefCell<Controller<P>>>, commanded_depth: &Vec<u64>) {
    println!("Starting controller...");
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
    for (_i, depth) in commanded_depth.iter().enumerate() {
        if control_state.in_panic(){
            panic(control_state.clone()).await;
        }
        if control_state.out_of_brain_uncalibrated(){
            calibrate(control_state.clone()).await;
            println!("Calibrated");
        }
        assert!(control_state.out_of_brain_calibrated(), "Expected out of brain calibrated but was: {}", control_state.current_state.lock().unwrap());
        assert!(control_state.get_robot_state().await.unwrap().needle_z == 0);
        println!("Inserting {} thread", _i);
        let outcome = insert_ib_open_loop(control_state.clone(), *depth).await;
        match outcome {
            InBrainOutcome::Success => {
                control_state.outcomes.lock().unwrap().push(true);
            }
            InBrainOutcome::Failure => {
                control_state.outcomes.lock().unwrap().push(false);
                println!("Failure");
            }
            _ => {panic!("Unexpected outcome")}
        }
    }
    transition_state(control_state.clone(), ControllerState::Dead, false);
    println!("Done");
    control_state.dead_tx.send(()).await.unwrap();
}

async fn retract_ib<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
    assert!(control_state.get_recent_state().unwrap().needle_z == 0);
    assert!(control_state.get_state() == ControllerState::OutOfBrainCalibrated);
}

async fn insert_ib_open_loop<P: BrainPredictor>(control_state: Arc<Controller<P>>, commanded_depth: u64) -> InBrainOutcome {
    assert!(commanded_depth >= COMMANDED_DEPTH_MIN_NM && commanded_depth <= COMMANDED_DEPTH_MAX_NM);
    let pos = control_state.get_recent_state().unwrap();
    assert!(pos.needle_z == 0 && pos.inserter_z == control_state.get_pre_move_location().unwrap(), "Needle not at zero, instead at: {:?}", pos);
    let init_time = Instant::now();
    while !control_state.in_panic() && Instant::now().duration_since(init_time).as_millis() < MAX_IB_TIME.into() {
        let Some(relative_position) = control_state.get_move_location(commanded_depth) else{
            sleep(Duration::from_millis(15)).await;
            continue;
        };
        let response = {
            control_state.command_move(&Move::NeedleZ(relative_position)).await
        };
        match response {
            Ok(_) => {
                println!("Success full in brain move");
                retract_ib(control_state.clone()).await;
                return InBrainOutcome::Success;
            }
            Err(RobotError::MoveError{..}) | Err(RobotError::ConnectionError{..}) => {
                println!("Connection error in moving to position: {}", relative_position);
                retract_ib(control_state.clone()).await;
                return InBrainOutcome::Failure;
            }
            Err(RobotError::PositionError{..}) => {
                die(control_state.clone());
                break;
            }
        }
    }
    if control_state.in_panic() {
        panic(control_state.clone()).await;
    }else{
        retract_ib(control_state.clone()).await;
    }
    return InBrainOutcome::Failure;
}

async fn move_bot<P: BrainPredictor>(control_state: Arc<Controller<P>>, command: &Move, next_state: ControllerState, from_panic: bool) -> () {
    loop {
        let response = control_state.command_move(command).await;
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
    println!("Moved to position: {}", command);
    transition_state(control_state,next_state, from_panic);
}

fn transition_state<P: BrainPredictor>(control_state: &mut Controller<P>, next_state: ControllerState, from_panic: bool) {
    let mut state_changer = control_state.info.borrow_mut();
    let mut  can_change = state_changer.current_state!= ControllerState::Panic || from_panic;
    can_change = can_change && state_changer.current_state != ControllerState::Dead;
    if !can_change {
        println!("Cannot change state from {} to {}", state_changer, next_state);
        return;
    }
    state_changer.current_state = next_state;
}



impl<P: BrainPredictor> Robot for Controller<P>{

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

impl<P: BrainPredictor> OCTService for Controller<P>{
    
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