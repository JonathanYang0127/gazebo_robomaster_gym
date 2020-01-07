#! /usr/bin/env python
import rospy
from robomaster_env import RobomasterEnv

if __name__ == '__main__': 
    rospy.init_node('gym_env_node') 
    env = RobomasterEnv()
    for i in range(1000):
        state, reward, done, info = env.step([0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0])
        time.sleep(0.01)
        if done:
            env.reset()
    rospy.spin()
