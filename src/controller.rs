use crate::interface::{RobotError, RobotState, OCTService, OCTError, Move, Robot};
use tokio::sync::{mpsc, oneshot, Notify};
use tokio::time::{sleep, Duration, Instant};
use std::collections::VecDeque;
use roots::find_root_brent;
use roots::SimpleConvergency;
use crate::predictor::BrainPredictor;
use std::sync::Arc;
use std::sync::Mutex;

//How close we allow our robot to get to the brain
const MIN_DISTANCE_BRAIN_TO_ARM_NM: u64 = 200_000;
//Number of samples we take during the calibration period
const CALIBRATION_SAMPLES: u64 = 1000;
//Max size of queues
const MAX_DISTANCES: u64 = 100;
const MAX_STATES: u64 = 100;
//Max time in brain before we panic
const MAX_IB_TIME: u64 = 30_000; // HAS TO CHANGE
//Max consecutive prediction errors before we panic
const MAX_CONSECUTIVE_PREDICTION_ERRORS: u64 = 20;
//Max prediction error before we actually count it
const MAX_PREDICTION_ERROR_NM: u64 = 50_000;
//Max distance from robot to brain before moving
const MAX_DIST_FROM_PREMOVE_TO_MOVE: u64 = MIN_DISTANCE_BRAIN_TO_ARM_NM + 3000;

//Polling rates
const OCT_POLL_MILLIS: u64 = 5;
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
    consecutive_errors: u64, //Local prediction errors
    pre_move_location: Option<u64>, //u64
    pub outcomes: Vec<bool>,
    notified_distances: Vec<Result<u64, OCTError>>,
    notified_distance_times: Vec<Instant>,
}

impl ControllerInfo{
    fn clear_distance_queue(&mut self) {
        self.distance_queue.clear();
        self.distance_time_queue.clear();
    }
}

pub struct Controller<P: BrainPredictor>{
    info: Mutex<ControllerInfo>,
    distance_tx: mpsc::Sender<((), oneshot::Sender<Result<u64, OCTError>>)>,
    state_tx: mpsc::Sender<((), oneshot::Sender<Result<RobotState, RobotError>>)>,
    move_tx: mpsc::Sender<(Move, oneshot::Sender<Result<(), RobotError>>)>,
    dead_tx: mpsc::Sender<()>,
    predictor: P,
    can_move: Notify,
}

impl<P: BrainPredictor> Controller<P>{

    /// Creates a new controller with the given parameters.
    ///
    /// The distance queue stores distances from the OCT service. Each element is a tuple
    /// containing the distance and the time at which it was received.
    ///
    /// The robot queue stores the state of the robot. Each element is a tuple containing the
    /// robot state and the time at which it was received.
    ///
    /// The distance time queue stores the times at which the distances were received.
    ///
    /// The robot time queue stores the times at which the robot states were received.
    ///
    /// The consecutive errors variable stores the number of consecutive prediction errors.
    ///
    /// The pre move location variable stores the location of the inserter z after calibration
    ///
    /// The outcomes vector stores the outcomes of the last few moves.
    ///
    /// The notified distances vector stores the distances that have been notified to the
    /// controller.
    ///
    /// The notified distance times vector stores the times at which the distances were
    /// notified.
    
    pub fn new(distance_tx: mpsc::Sender<((), oneshot::Sender<Result<u64, OCTError>>)>,
    state_tx: mpsc::Sender<((), oneshot::Sender<Result<RobotState, RobotError>>)>,
    move_tx: mpsc::Sender<(Move, oneshot::Sender<Result<(), RobotError>>)>,
    dead_tx: mpsc::Sender<()>, predictor: P) -> Controller<P>{
        Controller{
            info: Mutex::new(ControllerInfo{
                current_state: ControllerState::Dead, //ControllerState::Dead,
                distance_queue: VecDeque::new(), //VecDeque::new(),
                robot_queue: VecDeque::new(), //VecDeque::new(),
                distance_time_queue: VecDeque::new(), //VecDeque::new(),
                robot_time_queue: VecDeque::new(),
                consecutive_errors: 0,
                pre_move_location: None,
                outcomes:Vec::new(),
                notified_distances: Vec::new(),
                notified_distance_times: Vec::new(),
            }),
            distance_tx,
            state_tx,
            move_tx,
            dead_tx,
            predictor,
            can_move: Notify::new(),
        }
    }

    fn out_of_brain_uncalibrated(&self) -> bool {
        let info = self.info.lock().unwrap();
        info.current_state == ControllerState::OutOfBrainUncalibrated
    }

    fn out_of_brain_calibrated(&self) -> bool {
        let info = self.info.lock().unwrap();
        info.current_state == ControllerState::OutOfBrainCalibrated
    }

    fn in_panic(&self) -> bool {
        let info = self.info.lock().unwrap();
        info.current_state == ControllerState::Panic
    }

    fn in_brain(&self) -> bool {
        let info = self.info.lock().unwrap();
        info.current_state == ControllerState::InBrain
    }

    fn dead(&self) -> bool {
        let info = self.info.lock().unwrap();
        info.current_state == ControllerState::Dead
    }

    fn clear_distance_queue(&self) {
        let mut info = self.info.lock().unwrap();
        info.clear_distance_queue();
    }


    /// Calculates the location to move the robot based on the commanded depth.
    ///
    /// This function uses the predicted brain position and the commanded depth to 
    /// determine the optimal move location for the robot. It first checks if the 
    /// brain is close enough to the needle before proceeding. If the brain is too 
    /// far, the function returns `None`. It uses a function to calculate the 
    /// intersection of the brain's predicted path and the needle's path, and returns the position
    /// relative to the inserter z the needle should move based on the intersection. If a valid 
    /// root is found, it returns the calculated move location; otherwise, it returns 
    /// `None`.
    ///
    /// # Parameters
    /// - `commanded_depth`: The depth to which the robot is commanded to move.
    ///
    /// # Returns
    /// `Option<u64>`: The calculated move location if successful, otherwise `None`.
    fn get_move_location(&self, commanded_depth: u64) -> Option<u64> {
        let info = self.info.lock().unwrap();
        let Some(brain_position_function) = self.predictor.predict(&info.notified_distances, &info.notified_distance_times, true) else {
            println!("No brain position function");
            return None;
        };
        //We only move the robot if the brain is sufficiently close to the needle before moving
        if info.notified_distances.last().cloned().unwrap().unwrap() > MAX_DIST_FROM_PREMOVE_TO_MOVE {
            println!("We are too far away from the brain to move");
            return None;
        }
        //We calculate how far to move the robot based on where its path intersects the commanded location's path
        let needle_pos = |x: f64| {NEEDLE_ACCELERATION_NM_MS as f64/4.0 * x * x};
        let intersection_fn = |x|{brain_position_function(x as f64) + commanded_depth as f64 - needle_pos(x as f64)};
        let furthest_needle_move = (4.0*COMMANDED_DEPTH_MAX_NM as f64/NEEDLE_ACCELERATION_NM_MS as f64).sqrt()+100.0;
        let mut convergency = SimpleConvergency { eps:1e-15f64, max_iter:30 };
        let Ok(root) = find_root_brent(0.0, furthest_needle_move, &intersection_fn, &mut convergency) else{
            println!("Failed to find root with furthest needle move: {}", furthest_needle_move);
            return None;
        };
        return Some(brain_position_function(root) as u64 + commanded_depth);
    }
    
    //This function checks if the the brain has abnormal moving activity
    //The hyper local predictions allow us to check in real time whether the
    //brian is moving abnormally, or "siezing". In the case it is, we panic.
    fn is_abnormal_distance(&self, distance: u64) -> bool {
        let info = self.info.lock().unwrap();
        let distances = Vec::from(info.distance_queue.clone());
        let times = Vec::from(info.distance_time_queue.clone());
        let Some(brain_position_function) = self.predictor.predict(&distances, &times, false) else {
            println!("Len of distance queue is: {}", info.distance_queue.len());
            return true;
        };
        let prediction = brain_position_function(info.distance_time_queue[info.distance_time_queue.len()-1].elapsed().as_millis() as f64);
        let diff = (distance as f64 - prediction).abs();
        if diff > MAX_PREDICTION_ERROR_NM as f64{
            println!("Diff was: {}", diff);
            println!("ABNORMAL DISTANCE:: Previous State: {}, Recent State: {}, Prediction: {}, Time since recent state: {}, Actual distance: {} Current State: {}",
                info.distance_queue.get(info.distance_queue.len() - 2).unwrap().clone().unwrap(),
                info.distance_queue.get(info.distance_queue.len() - 1).unwrap().clone().unwrap(),
                prediction,
                info.distance_time_queue.get(info.distance_time_queue.len() - 1).unwrap().elapsed().as_millis(),
                distance,
                info.current_state
            );
        }
        return diff > MAX_PREDICTION_ERROR_NM as f64;
    }

    //We assume here that getting the robot state is instant
    async fn get_recent_robot_state(&self) -> Option<RobotState> {
        Some(self.get_robot_state().await.unwrap())
    }

    fn set_state(&self, state: ControllerState) {
        let mut info = self.info.lock().unwrap();
        info.current_state = state;
    }

    fn get_state(&self) -> ControllerState {
        let info = self.info.lock().unwrap();
        return info.current_state;
    }

    fn add_error(&self) {
        let mut info = self.info.lock().unwrap();
        info.consecutive_errors += 1;
    }

    fn clear_error(&self) {
        let mut info = self.info.lock().unwrap();
        info.consecutive_errors = 0;
    }

    fn get_consecutive_errors(&self) -> u64 {
        let info = self.info.lock().unwrap();
        return info.consecutive_errors;
    }

    fn get_pre_move_location(&self) -> Option<u64> {
        let info = self.info.lock().unwrap();
        return info.pre_move_location;
    }

    fn clear_pre_move_location(&self) {
        let mut info = self.info.lock().unwrap();
        info.pre_move_location = None;
    }

    fn add_outcome(&self, outcome: bool) {
        let mut info = self.info.lock().unwrap();
        info.outcomes.push(outcome);
    }

    fn add_distance(&self, distance: Result<u64, OCTError>) {
        let expected_length = if self.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_DISTANCES};
        let mut info = self.info.lock().unwrap();
        info.distance_queue.push_back(distance);
        while info.distance_queue.len() > expected_length.try_into().unwrap() {
            info.distance_queue.pop_front();
        }
    }

    fn add_distance_time(&self, time: Instant) {
        let expected_length = if self.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_DISTANCES};
        let mut info = self.info.lock().unwrap();
        info.distance_time_queue.push_back(time);
        while info.distance_time_queue.len() > expected_length.try_into().unwrap() {
            info.distance_time_queue.pop_front();
        }
    }

    fn add_robot_state(&self, state: Result<RobotState, RobotError>) {
        let expected_length = if self.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_STATES};
        let mut info = self.info.lock().unwrap();
        info.robot_queue.push_back(state);
        while info.robot_queue.len() > expected_length.try_into().unwrap() {
            info.robot_queue.pop_front();
        }
    }

    fn add_robot_state_time(&self, time: Instant) {
        let expected_length = if self.out_of_brain_uncalibrated() {CALIBRATION_SAMPLES} else {MAX_STATES};
        let mut info = self.info.lock().unwrap();
        info.robot_time_queue.push_back(time);
        while info.robot_time_queue.len() > expected_length.try_into().unwrap() {
            info.robot_time_queue.pop_front();
        }
    }

    pub fn get_outcomes(&self) -> Vec<bool> {
        let info = self.info.lock().unwrap();
        return info.outcomes.clone();
    }

    //The notificiation system works as follows: When the process_distances task
    //notices that the brain is close enough to the robot to move, it will notify
    // the move task.The move task will only move if it was already waiting for a
    //notificaiton (we dont want these notifications to persist because we might
    //move at the wrong time in the future).
    fn set_move_notification(& self) {
        let mut info = self.info.lock().unwrap();
        info.notified_distance_times = Vec::from(info.distance_time_queue.clone());
        info.notified_distances = Vec::from(info.distance_queue.clone());
        self.can_move.notify_waiters();
    }
    
}

fn die<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    control_state.set_state(ControllerState::Dead);
}

//This task is responsible for polling the robot for its distance from the surface
//Since polling is IO bound, a new task is spawned for each poll so that we get 
//values every 5ms instead of every 15ms as per the project description
async fn poll_distance<P: BrainPredictor + 'static>(
    control_state: Arc<Controller<P>>,
    tx: mpsc::Sender<Result<u64, OCTError>>
){
    loop {
        let tx_clone = tx.clone();
        let control_clone = control_state.clone();
        tokio::task::spawn_local({
            async move {
                let distance = control_clone.get_surface_distance().await;
                if tx_clone.send(distance).await.is_err() {
                    println!("Receiver dropped, stopping polling.");
                }
            }
        });

        // Wait for 5 seconds before polling again to keep under 20Hz
        sleep(Duration::from_millis(OCT_POLL_MILLIS)).await;
    }
}

async fn poll_state<P: BrainPredictor + 'static>(
    control_state: Arc<Controller<P>>,
    tx: mpsc::Sender<Result<RobotState, RobotError>>
){
    loop {
        let tx_clone = tx.clone();
        let control_clone = control_state.clone();

        // The future here must be 'static. Adding `+ 'static` to P helps.
        tokio::task::spawn_local({
            async move {
                let distance = control_clone.get_robot_state().await;
                if tx_clone.send(distance).await.is_err() {
                    println!("Receiver dropped, stopping polling.");
                }
            }
        });

        // Wait for 5 seconds before polling again
        sleep(Duration::from_millis(OCT_POLL_MILLIS)).await;
    }
}
//This task is responsible for processing the distance values from the robot
//Processing involves two steps: 1. Checking if the distance is abnormal 
//2. Checking if the distance is close enough to the brain to trigger a move
async fn process_distances<P: BrainPredictor>(control_state: Arc<Controller<P>>, mut rx: mpsc::Receiver<Result<u64, OCTError>>) {
    while let Some(distance_result) = rx.recv().await {
        match distance_result {
            Ok(distance) => {
                //We can only panic when OOBC or IB in the state machine
                let can_panic = control_state.out_of_brain_calibrated() || control_state.in_brain();
                // Check for abnormal distance
                let too_close_to_brain = distance < MIN_DISTANCE_BRAIN_TO_ARM_NM/2;
                if too_close_to_brain && can_panic {
                    println!("Too close to brain: {}", distance);
                    transition_state(control_state.clone(), ControllerState::Panic, false);
                }
                else if can_panic && control_state.is_abnormal_distance(distance) {
                    println!("Can panic {}", can_panic);
                    control_state.add_error();
                    if control_state.get_consecutive_errors() > MAX_CONSECUTIVE_PREDICTION_ERRORS && can_panic
                    {
                        println!("Too many consecutive errors");
                        assert!(!control_state.in_panic());
                        transition_state(control_state.clone(), ControllerState::Panic, false);
                    }
                } else {
                    //If we are not in panic, clear the error since they are non consecutive
                    control_state.clear_error();
                }
                //If we notice we can trigger a move, we trigger it
                if distance < MAX_DIST_FROM_PREMOVE_TO_MOVE {
                    println!("Found premove location");
                    control_state.set_move_notification();
                }
            }
            Err(_) => {}
        };

        // Update queues
        control_state.add_distance(distance_result);
        control_state.add_distance_time(Instant::now());
        
        //tokio::task::yield_now().await;
    }
}

//The code currently doesn;t utilize the robot state in any way aside from checking values for the state machine
async fn process_robot_state<P: BrainPredictor>(control_state: Arc<Controller<P>>, mut rx: mpsc::Receiver<Result<RobotState, RobotError>>) {
    while let Some(robot_state) = rx.recv().await {
        match robot_state {
            Ok(_) => {}
            Err(RobotError::ConnectionError{..}) | Err(RobotError::MoveError{..}) => {
                println!("Received error in processing robot state");
            }
            Err(RobotError::PositionError{..}) => {
                die(control_state.clone());
            }
        };
        control_state.add_robot_state(robot_state);
        control_state.add_robot_state_time(Instant::now());
        sleep(Duration::from_millis(ROBOT_STATE_POLL_MILLIS)).await;
    }
}

//When panicing, we move the needl to the origin first to potentially get out of the brain
//We then move the inserter to the origin and recalibrate our robot, since panics
//could have occured due to abnormal brain activity/bad motion predictions
async fn panic<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::Panic, false).await;
    move_bot(control_state.clone(), &Move::InserterZ(0), ControllerState::Panic, false).await;
    transition_state(control_state,ControllerState::OutOfBrainUncalibrated, true);
}

//The calibration sequence is very simple - we stare at the brain for CALIBRATION_SAMPLES OCT samples,
//calculate the closest the brain got to the robot, and move the inserter 200 microns above that location.
async fn calibrate<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    assert!(control_state.get_recent_robot_state().await.unwrap() == RobotState{inserter_z: 0, needle_z: 0} && control_state.out_of_brain_uncalibrated());
    println!("Out of assert in calibrate");
    //Reset the robots state to relearn all parameters
    let calibration_init = Instant::now();
    control_state.clear_error();
    control_state.clear_distance_queue();
    control_state.clear_pre_move_location();
    loop{
        {
            let mut controller = control_state.info.lock().unwrap();
            let distance_queue = &controller.distance_queue;
            let distance_time_queue = &controller.distance_time_queue;
            if distance_queue.len() >= CALIBRATION_SAMPLES.try_into().unwrap() && distance_queue.front().unwrap().is_ok() && *distance_time_queue.front().unwrap() >= calibration_init {
                let min_distance = distance_queue.iter().filter(|d| d.is_ok()).min_by_key(|d| d.as_ref().unwrap()).unwrap().as_ref().unwrap();
                assert!(*min_distance > MIN_DISTANCE_BRAIN_TO_ARM_NM);
                //Calculate our premove location by staring at the brain for a while
                controller.pre_move_location = Some(*min_distance - MIN_DISTANCE_BRAIN_TO_ARM_NM);
                break;
            }
        }
        sleep(Duration::from_millis(10)).await;
    }
    //Set our premove location and move the robot to the premove lcoation
    //By the state machine, we guarantee the robot will move to {premove_location, 0}
    let premove_location = control_state.get_pre_move_location().unwrap();
    move_bot(control_state.clone(), &Move::InserterZ(premove_location), ControllerState::OutOfBrainUncalibrated, false).await;
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
    control_state.clear_distance_queue();
    println!("---------------------------------------------------------------------------------------------------------------------------------------");
}

//We start our two polling tasks, one for distances and one for robot state
//We additionally start our two processing tasks, one for distances and one for robot state
//The fifth async task is our state machine, which is responsible for moving the robot 
//In every iteration, we start by guaranteeing that we move from panic --> OOBU, from OOBU --> OOBC
// and finally from OOBC --> IB, skipping any transitions if we are not in those states.

//The transition from panic -->OOBC is moving to the origin, from OOBU -->OOBC is calibration, and from OOBC --> IB
//is entering the brain
pub async fn start<P: BrainPredictor + 'static>(control_state: Arc<Controller<P>>, commanded_depth: &Vec<u64>) {
    println!("Starting controller...");
    //Make channels for communicating with robot simulation
    let (tx_distance, rx_distance) = mpsc::channel::<Result<u64, OCTError>>(20);
    let (tx_state, rx_state) = mpsc::channel::<Result<RobotState, RobotError>>(20);
    //Spawn our polling and processing tasks
    tokio::task::spawn_local({let me = Arc::clone(&control_state);
    async move {
        poll_distance(me, tx_distance).await;
    }});
    tokio::task::spawn_local({let me = Arc::clone(&control_state);
        async move {
            poll_state(me, tx_state).await;
        }});
    println!("Starting to process distances...");
    tokio::task::spawn_local({let me = Arc::clone(&control_state);
        async move {
            process_distances(me, rx_distance).await;
        }});
    println!("Starting to process robot state...");
    tokio::task::spawn_local({let me = Arc::clone(&control_state);
        async move {
            process_robot_state(me, rx_state).await;
        }});
    
    //Start the state machine
    control_state.set_state(ControllerState::OutOfBrainUncalibrated);
    for (_i, depth) in commanded_depth.iter().enumerate() {
        loop{
            if control_state.in_panic(){
                panic(control_state.clone()).await;
            }
            if control_state.out_of_brain_uncalibrated(){
                calibrate(control_state.clone()).await;
                println!("Calibrated");
            }
            assert!(control_state.out_of_brain_calibrated(), "Expected out of brain calibrated but was: {}", control_state.get_state());
            assert!(control_state.get_robot_state().await.unwrap().needle_z == 0);
            println!("Inserting {} thread", _i);
            let outcome = insert_ib_open_loop(control_state.clone(), *depth).await;
            match outcome {
                InBrainOutcome::Success => {
                    control_state.add_outcome(true);
                    break;
                }
                InBrainOutcome::Failure => {
                    control_state.add_outcome(false);
                    println!("Failure");
                    break;
                }
                _ => {}
            }
        }
    }
    transition_state(control_state.clone(), ControllerState::Dead, false);
    println!("Done");
    //Send a message to the robot to stop
    control_state.dead_tx.send(()).await.unwrap();
}

//Move the needle to the pre_move_location
async fn retract_ib<P: BrainPredictor>(control_state: Arc<Controller<P>>) {
    move_bot(control_state.clone(), &Move::NeedleZ(0), ControllerState::OutOfBrainCalibrated, false).await;
    assert!(control_state.get_recent_robot_state().await.unwrap().needle_z == 0);
    assert!(control_state.out_of_brain_calibrated());
}

//Moving the needle into the brain
async fn insert_ib_open_loop<P: BrainPredictor>(control_state: Arc<Controller<P>>, commanded_depth: u64) -> InBrainOutcome {
    assert!(commanded_depth >= COMMANDED_DEPTH_MIN_NM && commanded_depth <= COMMANDED_DEPTH_MAX_NM);
    let pos = control_state.get_recent_robot_state().await.unwrap();
    assert!(pos.needle_z == 0 && pos.inserter_z == control_state.get_pre_move_location().unwrap(), "Needle not at zero, instead at: {:?}", pos);
    let init_time = Instant::now();
    //Move the needle into the brain while we arent panicing or havent spent too long waiting
    while !control_state.in_panic() && Instant::now().duration_since(init_time).as_millis() < MAX_IB_TIME.into() {
        //Wait for the distance processor to tell us we can move
        control_state.can_move.notified().await;
        //If the move location is None, then we dont have a vlaid move on hand, based on the assumptions in predictor.rs
        let Some(relative_position) = control_state.get_move_location(commanded_depth) else{
            continue;
        };
        let response = {
            control_state.command_move(&Move::NeedleZ(relative_position)).await
        };
        //In all cases we break, either considering ourselves a success or a failure
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
    //If we panic, panic
    if control_state.in_panic() {
        panic(control_state.clone()).await;
    }else{
        //If we dont panic, then we exit the brain
        retract_ib(control_state.clone()).await;
    }
    return InBrainOutcome::Panic;
}

//This function is meant for moving outside of the brain and guarantees eventual consistency by looping until the move is successful
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

//This function transitions our state
//If we are ever in a panic state, we shouldn't let a successful move from prveious exit the panic
//Thus we check this with the from_panic flag
fn transition_state<P: BrainPredictor>(control_state: Arc<Controller<P>>, next_state: ControllerState, from_panic: bool) {
    let mut  can_change = !control_state.in_panic() || from_panic;
    can_change = can_change && !control_state.dead();
    if !can_change {
        println!("Cannot change state from {} to {}", control_state.get_state(), next_state);
        return;
    }
    control_state.set_state(next_state);
}

//This is the interface between the controller and the robot
//Command grasp is mocked as always succeeding
//Command move and get robot state ask to move until it receives a response from the robot
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