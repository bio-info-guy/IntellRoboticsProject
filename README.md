# Project Report

**Yangqi Su 800957989**

## SHOWCASE URL

https://sites.google.com/d/1EIyaS9tptFuEI33ad2DbSjbZd39qca0g/edit", "file_id": "1EIyaS9tptFuEI33ad2DbSjbZd39qca0g
---

## Context

Building a robot that is capable of catching objects can translate naturally into many areas of engineering and even entertainment. The goal of this project was to implement 2 robot arms that can pass a ball between one another and the user. Such a system can be utilized in areas such as packaging and delivery, where the robot can help load packages in place of humans while still allowing humans to work with more freedom since the robot does not require the packages to be placed in a fixed way. Furthermore, such a system can also be used specifically in athletic training for sports that involve inanimate moving objects (i.e. basketball, soccer, etc.). Using a robotic arm in these cases can give accurate readouts for the athletes involved and provide personalized feedback and training for athletes to improve their accuracy (i.e. shooting a ball, throwing a pitch, etc.). These are just a few of the areas that such a model can be of use.

---

## Task Completion

As of the current deadline, one 6 DOF robot arm has been implemented using kinematics only. The robot arm currently can only catch objects moving towards it at a constant velocity.

---

## Technical Description

- **Robot Arm Structure:**  
  The simulated robot arm contains 6 revolute joints as described in the accompanied URDF file. In order to acquire an analytical inverse kinematics solution for the joints, the last 3 revolute joints are at a single point connected to the end-effector (last 3 links are of length 0). However, if needed, the program can be modified to accommodate links of any length, but would only use numerical inverse kinematics to solve the joints. Both the numerical inverse kinematics and analytical kinematics solutions were adopted from [1].

- **Ball Visualization and Movement:**  
  A ball was drawn using the ROS `visualization_marker` package, and is currently programmed to move at a constant velocity from an arbitrary starting point.

- **Initialization File:**  
  The link lengths and initial joint angles for the robot are read from an initialization file. Furthermore, the velocity and starting point of the ball are also read from the same file. The file contains an additional alpha parameter indicating the type of trajectory for the robot arm to generate in order to catch the ball. The values for alpha are as follows:
  
  - `0`: Cartesian Straight-Line Trajectory  
  - `1`: Screw Trajectory  
  - `2`: Joint-space Straight-Line Trajectory

  The trajectories can utilize both quintic time-scaling and cubic time-scaling, and both are shown in the demos. All implementations were adopted from [1].

---

## Ball Catching Process

1. **Workspace Intersection Check:**  
   In order to catch the ball, the program first calculates whether the path of the ball intersects the workspace of the robot.  
   - If the ball is determined to not cross the workspace, the arm does not move and a message is displayed stating that the ball is uncatchable.

2. **Determining Catch Points:**  
   - If the ball does cross the workspace, the arm calculates the entry and exit point (points in which the ball enters and exits the workspace) of the ball.  
   - It then finds a point near the exit to catch the ball.

3. **Setting End-Effector Orientation:**  
   Upon catching the ball, the orientation of the end-effector is selected as follows:
   - The z-axis points in the direction opposite to the ball’s velocity.
   - The ball’s velocity vector resides in the z-y plane.

4. **Trajectory Generation and Movement:**  
   Once the orientation and position of the end-effector are determined:
   - A trajectory is generated from the robot arm’s current position to the final configuration using one of the above three trajectory generation methods.  
   - The joint configurations are calculated with analytical inverse kinematics.
   - The arm then attempts to move to the position and orientation in half the time it takes for the ball to arrive.

5. **Catching the Ball:**  
   After the arm arrives at the final configuration, it calculates the distance between the ball and itself.  
   - If the distance is under 1mm, the movement of the ball is stopped and the ball is caught.

- **Simulation Environment:**  
  All simulations were performed using RViz.

- **Project Demos:**  
  Demos of the project can be viewed on the webpage:  
  [https://sites.google.com/uncc.edu/ysu13-robotics-project/home](https://sites.google.com/uncc.edu/ysu13-robotics-project/home)

---

## File Locations & Commands

- **Initialization Parameters File:**  
  `/path/to/Robotics_Project/IR_catkin_ws/src/spatial6r_description/launch/init_config.yaml`

- **Commands to Run the Program:**

  ```bash
  $ cd /path/to/Robotics_Project/IR_catkin_ws
  $ catkin_make  
  $ source devel/setup.bash  
  $ roslaunch spatial6r_description rviz.launch  
  ```

---

## Future Work and Improvements

- **Trajectory Improvements:**  
  Currently, the second and third links of the arm are of the same length so that the end-effector can reach all points within the spherical workspace created by these links. However, if these links were of different lengths, there would exist a smaller sphere within the larger sphere that is unattainable. The non-joint trajectory generation methods do not take this into consideration. A definite improvement would be to plan several trajectories within the workspace to accommodate for the shortcomings of a single trajectory.

- **Gravity and Parabolic Curve for the Ball:**  
  Implementing gravity and a parabolic curve for the ball should be straightforward since the end-effector’s position and orientation requirements only depend on the ball’s final instantaneous velocity and position.

- **Throwing Mechanism:**  
  Generating a trajectory for throwing the ball should also be straightforward. Once the robot can both throw and catch, adding another robot with the same capabilities would be simple. The final part would involve constructing interfaces for the user to launch the ball from an arbitrary position and to receive the ball from the robot.

---

## Reference

[1] Lynch, Kevin M., and Frank C. Park. *Modern Robotics*. Cambridge University Press, 2017.

