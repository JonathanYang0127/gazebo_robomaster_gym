import gym
import robomaster_gym
import time 

env = gym.make('robomaster-env-v0')._start_rospy()

for i in range(1000):
        state, reward, done, info = env.step([0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0])
        time.sleep(0.01)
        if done:
            env.reset()
