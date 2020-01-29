import gym
from robomaster_gym.misc import *
import time

if __name__ == '__main__': 
    env = gym.make('robomaster-env-v0')._start_rospy()
    for i in range(1000):
        state, reward, done, info = env.step([[0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0]])
        time.sleep(0.01)
        if done:
            env.reset()
