# q_learning_project

## Implementation Plan

### Q-learning algorithm

* Executing the Q-learning algorithm
  * use formula given and translate it to code

* Determining when the Q-matrix has converged
  * 

* Determining robot actions
  *

### Robot perception

* Determining identities and locations of dumbbells
  * use the robot camera color sensor and LaserScan data (specifically for what's in front of it) to identify and move to the dumbbell

* Determining identities and locations of numbered blocks
  * Use the robot camera


### RObot manipulation & movement

* Picking up and putting down dumbbells
  * Once the robot is at the correct dumbbell it should have open grippers with the gripper at the handle of the dumbbell. To pick up the dumbbell it should close the grippers and move the end effector upwards until the dumbell is off the ground. Testing would involve placing the robot in the correct position and running the code to see if it successfully lifts the dumbbell

* Navigating to locations for pick up and put down of dumbbells
  * First I need to get the color dumbbell (robot_db) and block number (block_id) from q-learning algorithm. Then use the sensor data/functions from robot perception to navigate to the correct dumbbell, pick it up, carry it to the correct block, and place it in front of the block
  * I would test this by manually feeding the function(s) a dumbbell color and block number and see if it behaves correctly

### Preliminary Timeline

* Wed Feb 17: have some movement and/or perception working; start looking into q-learning and setting up code if haven't already

* Fri Feb 20: continue q-learning code and potentially work more on movement/perception code

* Mon Feb 22: have all of q-learning code working; continue working on robot perception

* Wed Feb 24: have robot perception working; continue/finish robot movement if haven't already

* Fri Feb 26: have code done and start writeup; tweak code if needed



## Writeup

### Objective

### Description

### The Code

#### Q-learning Algorithm

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  

#### Robot Perception

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  


#### Robot Manipulation and Movement

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  

* 
  * _Code location:_  
  * _Code description:_  

### Challenges

### Future Work

### Takeaways

* takeaway1
  * 

* takeaway1
  * 

### Gif of robot executing task

![robot executing task](robot_executing_task.gif)

