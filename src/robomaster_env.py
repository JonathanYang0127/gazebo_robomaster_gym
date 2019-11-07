#! /usr/bin/env python
import rospy

from roborts_msgs.msg import TwistAccel, GimbalAngle
from std_msgs.msg import Empty
from keyboard.msg import Key

class RobomasterEnv:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel_acc', TwistAccel, queue_size=1)
        self.cmd_vel_keyboard = TwistAccel();
        self.pub_gimbal_angle = rospy.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=1)
        self.cmd_gimbal_keyboard = GimbalAngle();


    def step(self):
        self.cmd_vel_keyboard.twist.linear.x = 1
        self.cmd_vel_keyboard.twist.linear.y = 1
        
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)
       

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
    for i in range(100):
        env.step()
    rospy.spin()
