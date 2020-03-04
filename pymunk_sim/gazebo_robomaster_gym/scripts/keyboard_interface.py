import robomaster_gym
import gym
import sys
import numpy as np
import pygame
from pygame.locals import QUIT, KEYDOWN, KEYUP
import time



# Dictionary mapping keyboard commands to actions
char_to_action = {
    'w': np.array([0, 1, 0, 0]),
    'a': np.array([-1, 0, 0, 0]),
    's': np.array([0, -1, 0, 0]),
    'd': np.array([1, 0, 0, 0]),
    'k': np.array([0, 0, 0, 1]),
    'j': np.array([0, 0, 0, -1])
}

# Dicionary storing whether each key was held
pressed_keys = {
    'w': False,
    'a': False,
    's': False,
    'd': False,
    'k': False,
    'j': False
}

env = gym.make('robomaster-env-v0', statistics_gui = True)._start_rospy()
obs = env.reset()

while True:
    dx = np.array([0, 0, 0, 0])

    # record key presses
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit()
        if event.type == KEYDOWN:
            pressed = chr(event.dict['key'])
            if pressed in pressed_keys.keys():
                pressed_keys[pressed] = True
            elif pressed == 'r':
                env.reset()
        if event.type == KEYUP:
            released = chr(event.dict['key'])
            if released in pressed_keys.keys():
                pressed_keys[released] = False

    # take actions corresponding to key presses
    for i in pressed_keys.items():
        if i[1]:
            new_action = char_to_action.get(i[0], None)
            dx += new_action
    action = np.append(dx, np.array([[0, 0, 0, 0]]))
    obs, reward, done, info = env.step(action, np.zeros(8))
    time.sleep(0.01)
