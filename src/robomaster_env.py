#! /usr/bin/env python
import rospy

from roborts_msgs.msg import TwistAccel, GimbalAngle
from std_msgs.msg import Empty
from keyboard.msg import Key
from gazebo_connection import GazeboConnection
import time

class RobomasterEnv:
    def __init__(self):
        #Set up publishers for motion
        self.pub_cmd_vel = rospy.Publisher('cmd_vel_acc', TwistAccel, queue_size=1)
        self.cmd_vel_keyboard = TwistAccel();
        self.pub_gimbal_angle = rospy.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=1)
        self.cmd_gimbal_keyboard = GimbalAngle();
        
        #Set up gazebo connection
        self.gazebo = GazeboConnection()
        self.gazebo.pauseSim()

        #Amount of time running per timestep
        self.running_step = 0.1

        #Store number of timesteps
        self.timestep = 0
        self.max_timesteps = 1000 

    def step(self, action):
        #Unpause Simulation
        self.gazebo.unpauseSim()
        
        #Set action parameters for publisher
        #self.cmd_vel_keyboard.twist.linear.x = 1
        #self.cmd_vel_keyboard.twist.linear.y = 1
        self.cmd_vel_keyboard.twist.angular.z = -1
        
        #publish actions and execute for running_step time
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)
        time.sleep(self.running_step) 
        
        #pause Simulation
        self.gazebo.pauseSim()

        #calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, action)
        done = self.timestep >= self.max_timesteps

        return state, reward, done, {}
       

    def reset(self):
        self.timestep = 0
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        rospy.logdebug("reset_monoped_joint_controllers...")
        #self.controllers_object.reset_monoped_joint_controllers()

        self.gazebo.pauseSim()
        state = self.get_state()

        return state

    def get_state(self):
        return -1

    def get_reward(self, state, action):
        return 0

if __name__ == '__main__':  
    rospy.init_node('gym_env_node')
    env = RobomasterEnv()
    for i in range(1000):
        state, reward, done, info = env.step(1)
        time.sleep(0.01)
        if done:
            env.reset()
    rospy.spin()
