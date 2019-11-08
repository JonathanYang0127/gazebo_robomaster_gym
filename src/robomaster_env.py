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
        self.pub_cmd_vel = [rospy.Publisher('roborts_{0}/cmd_vel_acc'.format(i + 1), TwistAccel, queue_size=1) for i in range(4)]
        self.cmd_vel = [TwistAccel() for i in range(4)]
        self.pub_gimbal_angle = [rospy.Publisher('roborts_{0}/cmd_gimbal_angle'.format(i + 1), GimbalAngle, queue_size=1) for i in range(4)]
        self.cmd_gimbal = [GimbalAngle() for i in range(4)]
        
        #Set up gazebo connection
        self.gazebo = GazeboConnection()
        self.gazebo.pauseSim()

        #Amount of time running per timestep
        self._running_step = 0.1

        #Max number of timesteps
        self._timestep = 0
        self._max_timesteps = 1000 

        #Conversion from timestep to real time
        self._real_time_conversion = 1

        #Max and 

    def _timestep_to_real_time(self):
        return self._timestep * self._running_step * self._real_time_conversion 

    def step(self, action):
        #Unpause Simulation
        self.gazebo.unpauseSim()
        
        #Set action parameters for publisher
        #self.cmd_vel_keyboard.twist.linear.x = 1
        #self.cmd_vel_keyboard.twist.linear.y = 1
        self.cmd_vel[0].twist.angular.z = -1
        
        #Publish actions and execute for running_step time
        for i in range(4):
            self.pub_cmd_vel[i].publish(self.cmd_vel[i])
            self.pub_gimbal_angle[i].publish(self.cmd_gimbal[i])

        time.sleep(self._running_step) 
        
        #Pause simulation
        self.gazebo.pauseSim()

        #calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, action)
        done = self._timestep >= self._max_timesteps

        return state, reward, done, {}
       

    def reset(self):
        self._timestep = 0
        self.real_time = 0
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        rospy.logdebug("reset_monoped_joint_controllers...")
        #self.controllers_object.reset_monoped_joint_controllers()

        self.gazebo.pauseSim()
        state = self.get_state()

        return state

    def get_state(self):
        #xyz position, robot angle, gimbal angle, number of projectiles, temperature of shooter, real time?

        #xyz: roborts_x/pose/position
        #robot angle: roborts_x/pose/orientation
        #
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
