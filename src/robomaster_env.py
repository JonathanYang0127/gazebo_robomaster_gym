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
        self.running_step = 0.01

    def step(self, action):
        self.gazebo.unpauseSim()
        #self.cmd_vel_keyboard.twist.linear.x = 1
        #self.cmd_vel_keyboard.twist.linear.y = 1
        self.cmd_vel_keyboard.twist.angular.z = -1
        
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)
        time.sleep(self.running_step)
        self.gazebo.pauseSim()
       

    def switch_robot(self, robot_num):
        print('Switching to robot ' + str(robot_num) + '!')
        self.cmd_vel_keyboard = TwistAccel();
        self.cmd_gimbal_keyboard = GimbalAngle();
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)

        robot_name = 'roborts_' + str(robot_num) if robot_num != 0 else ""
        self.pub_cmd_vel = rospy.Publisher(robot_name + '/cmd_vel_acc', TwistAccel, queue_size=1)
        self.pub_gimbal_angle = rospy.Publisher(robot_name + '/cmd_gimbal_angle', GimbalAngle, queue_size=1)

if __name__ == '__main__':  
    rospy.init_node('gym_env_node')
    env = RobomasterEnv()
    for i in range(1000):
        env.step(1)
        time.sleep(0.001)
    rospy.spin()
