# gazebo_robomaster_gym
This is a gym wrapper for the gazebo simulation of the ICRA Robomaster challenge made by the decision making team of Bear Dynamics.

## How to Use:
Clone the gazebo simulation from here: https://github.com/alexzhou0/RoboRTS-Berkeley-Perception. Clone this repository in ros_ws/src. 
Compile the ros workspace using catkin_make and source the setup file:

```bash
source <path_to_workspace>/devel/setup.bash
```  

Navigate to the directory of this repository and install robomaster_gym

```bash
pip install -e .
```  

Then, run the simulation using:

```bash
roslaunch roborts_sim multi_robot.launch
```

## Creating an environment:
The main robomaster environment is included in robomaster_gym/envs/robomaster_env.py  
To create the env write:

```
env = gym.make('robomaster-env-v0')._start_rospy()
```

## Files
There are currently 3 main directories in this package.  
robomaster_gym contains the gym environments  
scripts contains algorithms to test and run reinforcement learning on the environment  
strategies contains a few baseline strategies without rl
