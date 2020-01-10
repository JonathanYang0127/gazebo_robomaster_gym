import gym
import robomaster_gym

if __name__ == '__main__': 
    env = gym.make('closequarter-env-v0')._start_rospy()