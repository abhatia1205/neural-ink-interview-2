# Brain Motion Data

This folder contains two kinds of brain motion data:
 1. A 10 second recording of brain motion `human_brain_motion.csv`
 2. A series of local recordings of brain motion found in `local_cycles`

 Your implementation may choose to use either or both of these datasets depending on your
 needs.

## `human_brain_motion.csv`
### Specification
`Time [s]` - Time in seconds
`Relative Position [mm]` - The relative position of the brain at the corresponding time


## `local_cycles`
### Specification
Each file contains a short recording of brain motion.
`timestamp_ns` - Timestamp exprerssed in nanoseconds
`height_inv_mm` - the position of the brain in the vertical plane expressed in millimeters

1) Are the values in human brain motion relative to the inserterZ position? What about in the local cycles? Can I assume all the data is from the same surgery, or are they from different surgeries?

2) Where does command_grasp come into play? Am I expected to grab threads from somewhere else before placing them in the brain, and if so, from where do I grasp them? command_grap also seems to imply that the needle can move in all 3 axes, but for insertion it can only move in 1, so I do i need to ensure it is on that axis before continuing to the insertion phase?

3) What is the general coordinate system? Command move seems to take an absolute position. What is this relative to? 

4) What can I assume about the variance in delays on the asynchronous functions calls? It seems like a fundamental problem in this controls problem is predicting the absolute location of the brain surface. However, this is only possible with accurate (time, location) tuples as ground truth. It seems fundamental to the problem to have some level of accuracy in having a time on received measurements - can I assume anything about that?

5) Can I assume that there is minimal/no error on the robot's position? 

6) What is a reasonable amount of time that the needle can stay inside of the brain for? Can I keep the needle in the brain long enough to ensure it has stopped moving? I ask because it seems impossible to accuratle place the thread in the brain without knowing the speed of the needle's movement and the position of the brain surface at a given time, especially in one motion. 

7) What is the intended location of the thread in the brain? Is it relative to the brain's surface, or is it an absolute position? Additionally, what can I assume about the dynamics of the brain movement. Intuitively, artery movements should cause expansion in the brain around some center, whereas respiratory movements should cause translations of the whole brain. Thus, the absolute position of where the thread should be placed should theoretically also be moving, unless im mistaken. Where exactly is the thread supposed to be placed?