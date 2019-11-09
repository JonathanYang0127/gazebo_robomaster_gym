#! /usr/bin/env python
import rospy

from roborts_msgs.msg import TwistAccel, GimbalAngle
from std_msgs.msg import Empty
from keyboard.msg import Key
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_connection import GazeboConnection
import time

class RobomasterEnv:
    def __init__(self):
        #Set up publishers for motion
        self.pub_cmd_vel = [rospy.Publisher('roborts_{0}/cmd_vel_acc'.format(i + 1), TwistAccel, queue_size=1) for i in range(4)]
        self.cmd_vel = [TwistAccel() for i in range(4)]
        self.pub_gimbal_angle = [rospy.Publisher('roborts_{0}/cmd_gimbal_angle'.format(i + 1), GimbalAngle, queue_size=1) for i in range(4)]
        self.cmd_gimbal = [GimbalAngle() for i in range(4)]

        #Set up state parameters
        self._odom_info = [[None]] * 4 
        self._gimbal_angle_info = [[None]] * 4
        self._robot_hp = [[2000]] * 4
        self._num_projectiles = [[50]] * 4
        self._barrel_heat = [[0]] * 4
        self._launch_velocity_max = 25

        #Set up subscribers for state
        self.sub_odom = [rospy.Subscriber('roborts_{0}/ground_truth/state'.format(i+1), Odometry, self.odometry_callback) for i in range(4)]
        self.sub_gimbal_angle = [rospy.Subscriber('roborts_{0}/joint_states'.format(i+1), JointState, self.gimbal_angle_callback) for i in range(4)]
 
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

    def odometry_callback(self, msg):
        #print(self._odom_info)
        self._odom_info[int(msg._connection_header['topic'][9]) - 1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def gimbal_angle_callback(self, msg):
        self._gimbal_angle_info[int(msg._connection_header['topic'][9]) - 1] = msg.position 
        return

    def _timestep_to_real_time(self):
        return self._timestep * self._running_step * self._real_time_conversion 

    def step(self, action1, action2): 
        #position: x velocity, y velocity, twist angular; gimbal: pitch, yaw

        if len(action1) != 10 or len(action2) != 10:
            print("Invalid action!")
            return None, None, None, {} 

        #Unpause Simulation
        self.gazebo.unpauseSim()
        
        #Set action parameters for publisher
        self.cmd_vel[0].twist.linear.x = action1[0]
        self.cmd_vel[0].twist.linear.y = action1[1]
        self.cmd_vel[0].twist.angular.z = action1[2]
        self.cmd_gimbal[0].pitch_angle = action1[3]
        self.cmd_gimbal[0].yaw_angle = action1[4]
        self.cmd_vel[1].twist.linear.x = action1[5]
        self.cmd_vel[1].twist.linear.y = action1[6]
        self.cmd_vel[1].twist.angular.z = action1[7]
        self.cmd_gimbal[1].pitch_angle = action1[8]
        self.cmd_gimbal[1].yaw_angle = action1[9]

        self.cmd_vel[2].twist.linear.x = action2[0]
        self.cmd_vel[2].twist.linear.y = action2[1]
        self.cmd_vel[2].twist.angular.z = action2[2]
        self.cmd_gimbal[2].pitch_angle = action2[3]
        self.cmd_gimbal[2].yaw_angle = action2[4]
        self.cmd_vel[3].twist.linear.x = action2[5]
        self.cmd_vel[3].twist.linear.y = action2[6]
        self.cmd_vel[3].twist.angular.z = action2[7]
        self.cmd_gimbal[3].pitch_angle = action2[8]
        self.cmd_gimbal[3].yaw_angle = action2[9]

        #Publish actions and execute for running_step time
        for i in range(4):
            self.pub_cmd_vel[i].publish(self.cmd_vel[i])
            self.pub_gimbal_angle[i].publish(self.cmd_gimbal[i])

        time.sleep(self._running_step) 
        
        #Pause simulation
        self.gazebo.pauseSim()

        #calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, action1, action2)
        done = self._timestep >= self._max_timesteps

        return state, reward, done, {}
       

    def reset(self):
        self._timestep = 0
        self.real_time = 0
        self.gazebo.pauseSim()
        self.gazebo.resetSim()

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        #self.controllers_object.reset_monoped_joint_controllers()

        self.gazebo.pauseSim()
        state = self.get_state()

        return state

    def get_state(self):
        #xyz position, robot angle, gimbal angle, number of projectiles, barrel heat of shooter, robot hp
        #only use barrel heat for your robots

        #Sizes of parameters
        #xyz: 3
        #robot angle: 4
        #gimbal_angle: 2
        #number_of_projectiles: 1
        #barrel heat: 1 
        #robot hp: 1
        robot_state = [list(self._odom_info[i]) + list(self._gimbal_angle_info[i]) + self._num_projectiles[i] + self._barrel_heat[i] + self._robot_hp[i] for i in range(4)]
        return [robot_state[0] + robot_state[1] + robot_state[2] + robot_state[3], robot_state[0] + robot_state[1] + robot_state[2] + robot_state[3]]

    def get_reward(self, state, action1, action2):
        return 0

if __name__ == '__main__':  
    rospy.init_node('gym_env_node')
    env = RobomasterEnv()
    for i in range(1000):
        state, reward, done, info = env.step([0, 0, 1, 0, 0, 0, 0, -1, 0, 0], [0, 0, -1, 0, 0, 0, 0, 1, 0, 0])
        time.sleep(0.01)
        if done:
            env.reset()
    rospy.spin()
