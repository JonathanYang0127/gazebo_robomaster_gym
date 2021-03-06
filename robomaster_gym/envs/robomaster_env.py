import sys
import rospy
import gym
import math
import time
import pygame
import numpy as np

from roborts_msgs.msg import TwistAccel, GimbalAngle
from std_msgs.msg import Empty
from keyboard.msg import Key
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from robomaster_gym.misc import *
from strategies import *

class RobomasterEnv(gym.Env):
    def __init__(self, statistics_gui=False):
        # Set up state parameters
        self._odom_info = [None] * 4
        self._gimbal_angle_info = [None] * 4
        self._shoot = [0, 0, 0, 0]

        self.damage_threshold = 6

        self.robot_coords = [None] * 4
        self.robot_plates_coords = [None] * 4
        self.best_plates = [None] * 4

        # Amount of time running per timestep
        self._running_step = 0.1

        # Max number of timesteps
        self._timestep = 0
        self._max_timesteps = 1800

        # Conversion from timestep to real time
        self._real_time_conversion = 1

        # initialize segments of each obstacle for blocking calculation
        # buffer is added
        self.shoot_obstacle_buffer = 0.015
        self.move_obstacle_buffer = robot_diag_length

        # SHOOT_SEGMENTS are near ground-truth
        # MOVE_SEGMENTS are helper for navigation purposes
        self.shoot_segments = generate_obstacle_segments(self.shoot_obstacle_buffer)
        self.move_segments = generate_obstacle_segments(self.move_obstacle_buffer, add_diag=True)
        self.navigator = CriticalPointNavigatorWithRotation(self)

        self.on_init()

        # Set up gazebo connection
        self.gazebo = GazeboConnection()
        self.gazebo.pauseSim()

        # Set up pygame
        self._statistics_gui = statistics_gui
        self._setup_pygame()

    def on_init(self):
        self._timestep = 0
        self._robot_hp = [robot_max_hp] * 4
        self._num_projectiles = [0, 50, 0, 50]
        self._barrel_heat = [0] * 4
        self.spawn_buff_zones()

        self.move_disable = [0, 0, 0, 0]
        self.shoot_disable = [0, 0, 0, 0]

    
    def _setup_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1500, 450), pygame.RESIZABLE)
        if self._statistics_gui:
            self.screen.fill(pygame.Color('Black'))
            self.font = pygame.font.SysFont('monospace', 40)
            self.update_statistics({'HP': self._robot_hp,
                                    'MD': self.move_disable,
                                    'SD': self.shoot_disable})


    def update_statistics(self, statistics):
        text = ''
        text += 'Time: {}\n'.format(self._timestep)
        text += '{:15s} {:15s} {:15s} {:15s}\n'.format('Robot 0', 'Robot 1', 'Robot 2', 'Robot 3')
        for key, values in statistics.items():
            for v in values:
                text += '{:5s} {:9s} '.format(key+':', str(v))
            text += '\n'
        self.screen.fill(pygame.Color('Black'))
        self.blit_text(self.screen, text, (20, 20), self.font)
        pygame.display.update()


    def blit_text(self, surface, text, pos, font, color=pygame.Color('white')):
        words = [word.split(' ') for word in text.splitlines()]  # 2D array where each row is a list of words.
        space = font.size(' ')[0]  # The width of a space.
        max_width, max_height = surface.get_size()
        x, y = pos
        index = 0
        for line in words:
            for word in line:
                word_surface = font.render(word, 0, color)
                word_width, word_height = word_surface.get_size()
                if index != 0:
                    word_height -= 10
                else:
                    word_height += 5
                if x + word_width >= max_width:
                    x = pos[0]  # Reset the x
                    y += word_height  # Start on new row.
                surface.blit(word_surface, (x, y))
                x += word_width + space
            x = pos[0]  # Reset the x.
            y += word_height 
            index += 1

    def get_enemy_team(self, robot_index):
        if robot_index < 2:
            return (2, 3)
        return (0, 1)

    def get_ally(self, robot_index):
        return 1 - robot_index if robot_index <= 1 else 5 - robot_index

    def update_robot_coords(self, _id):
        self.robot_coords[_id] = robot_coords_from_odom(self._odom_info[_id])
        self.robot_plates_coords[_id] = plate_coords_from_odom(self._odom_info[_id])

    # return whether the line (x1, y1) - (x2, y2) is blocked by anything
    # pass in from_robot_index so it doesn't check for the robot itself!
    def is_line_of_sight_blocked(self, x1, y1, x2, y2, from_robot_index=None, segment_type='shoot'):
        # uninitiated
        if not self.robot_coords:
            return True
        for robot_index in range(4):
            if robot_index != from_robot_index:
                _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 = self.robot_coords[robot_index]
                if lines_cross(x1, y1, x2, y2, _x1, _y1, _x3, _y3) or lines_cross(x1, y1, x2, y2, _x2, _y2, _x4, _y4):
                    return robot_index
        segments = self.shoot_segments if segment_type == 'shoot' else self.move_segments
        for xl, yl, xh, yh in segments:
            if lines_cross(x1, y1, x2, y2, xl, yl, xh, yh):
                return True
        return False

    # TODO: determine by experiment
    # should also return optimal shoot velocity
    def expected_damage_with_optimal_shoot_vel(self, damage, angle, distance):
        return self.estimate_plate_visible_size(angle, distance) * damage, 20

    def estimate_plate_visible_size(self, angle, distance):
        return math.sin(angle) / (distance / gimbal_range_dis) ** 2

    # return the index and optimal shoot velocity of best enemy plate visible
    # None if no index is visible
    # NOTE: there will be an additional input, size of plate viewed by Vision, that helps calibrate this calculation irl.
    def get_best_enemy_plate(self, robot_index):
        if not all(self._odom_info):
            return
        x, y, yaw = self._odom_info[robot_index]
        best_plate = None
        damage = self.damage_threshold
        for enemy in self.get_enemy_team(robot_index):
            for plate_index in range(4):
                x1, y1, x2, y2 = self.robot_plates_coords[enemy][plate_index]
                center_x, center_y = midpoint(x1, y1, x2, y2)
                dis = distance(x, y, center_x, center_y)
                angle_diff = angleDiff(angleTo(x, y, center_x, center_y), yaw)
                if dis < gimbal_range_dis and angle_diff <= gimbal_range_angle:
                    shoot_angle = angleDiff(angleTo(x, y, center_x, center_y), angleTo(x1, y1, x2, y2))
                    exp_damage, vel = self.expected_damage_with_optimal_shoot_vel(armor_plate_damage[plate_index], shoot_angle,
                                                                                  dis)
                    if exp_damage > damage:
                        if not self.is_line_of_sight_blocked(x, y, center_x, center_y, robot_index):
                            best_plate = (enemy, plate_index, exp_damage, vel)
                            damage = exp_damage
        return best_plate

    def get_largest_enemy_plate(self, robot_index):
        if not all(self._odom_info):
            return
        x, y, yaw = self._odom_info[robot_index]
        best_plate = None
        max_size = 0
        for enemy in self.get_enemy_team(robot_index):
            if self._robot_hp[enemy] <= 0:
                continue
            for plate_index in range(4):
                x1, y1, x2, y2 = self.robot_plates_coords[enemy][plate_index]
                center_x, center_y = midpoint(x1, y1, x2, y2)
                dis = distance(x, y, center_x, center_y)
                angle_diff = angleDiff(angleTo(x, y, center_x, center_y), yaw)
                if dis < gimbal_range_dis and angle_diff <= gimbal_range_angle:
                    shoot_angle = angleDiff(angleTo(x, y, center_x, center_y), angleTo(x1, y1, x2, y2))
                    _size = self.estimate_plate_visible_size(shoot_angle, dis)
                    exp_damage, vel = self.expected_damage_with_optimal_shoot_vel(armor_plate_damage[plate_index], shoot_angle,
                                                                                  dis)
                    if exp_damage > self.damage_threshold and _size > max_size:
                        if not self.is_line_of_sight_blocked(x, y, center_x, center_y, robot_index):
                            best_plate = (enemy, plate_index, exp_damage, vel)
                            max_size = _size
        return best_plate

    def odometry_callback(self, msg):
        _id = int(msg._connection_header['topic'][9]) - 1
        self._odom_info[_id] = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                                quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]]
        self.update_robot_coords(_id)

    def gimbal_angle_callback(self, msg):
        self._gimbal_angle_info[int(msg._connection_header['topic'][9]) - 1] = msg.position

    def _timestep_to_real_time(self):
        # converts timestep to real time
        return self._timestep * self._running_step * self._real_time_conversion

    def _check_in_zone(self, robot_index, odom_info):
        # Input: x y z position
        # Checks if robot are in a buff/debuff zone, returns the zone
        x, y, yaw = odom_info
        for i in range(6):
            _type = self._zone_types[i]
            edge_buffer = 0.05 if abs(_type) >= 2 and (robot_index <= 1) == (_type > 0) else -0.05 # enlarge debuffs, shrink buffs
            left, right, bottom, top = zones[i]
            if x >= left + edge_buffer and x <= right - edge_buffer and y >= bottom + edge_buffer and y <= top - edge_buffer:
                return i

    def waypoint_to_cmd(self, robot_index, waypoint_index):
        cmd = None
        if self._timestep % 2 == 0:
            self.best_plates[robot_index] = self.get_largest_enemy_plate(robot_index) if self._robot_hp[robot_index] > 0 else None
        if waypoint_index is not None:
            if type(waypoint_index) is not tuple:
                waypoint = self.navigator.get_point(waypoint_index)
                cmd = self.navigator.navigate(robot_index, waypoint)
            else:
                cmd = self.navigator.navigate(robot_index, waypoint_index)
        else:
            cmd = [0, 0, -1, 0]
        if not cmd:
            return [0, 0, -1, 0]
        shoot_flag = 0
        cmd[-1] = shoot_flag
        return cmd

    def apply_heal(self, is_team_01):
        index1, index2 = (0, 1) if is_team_01 else (2, 3)
        self._robot_hp[index1] = min(robot_max_hp, self._robot_hp[index1] + 200)
        self._robot_hp[index2] = min(robot_max_hp, self._robot_hp[index2] + 200)

    def apply_bullet_refill(self, is_team_01):
        index1, index2 = (0, 1) if is_team_01 else (2, 3)
        self._num_projectiles[index1] += 100
        self._num_projectiles[index2] += 100

    # Call at each minute mark
    def spawn_buff_zones(self):
        self._zones_active = [True] * 6
        self._zone_types = generate_random_zone_config()

    def step(self, actions): 
        # [[x velocity, y velocity, twist angular, shoot]] * 4

        if len(actions) != 4:
            print("Invalid action!")
            return None, None, None, {}

        for robot_index in range(4):
            if self._robot_hp[robot_index] <= 0:
                actions[robot_index] = no_op

        #Check for/update debuffs to robots if in zone
        for robot_index in range(4):
            if self._odom_info[robot_index] is not None:
                if self._robot_hp[robot_index] <= 0:
                    continue
                zone_index = self._check_in_zone(robot_index, self._odom_info[robot_index])
                if zone_index is None or not self._zones_active[zone_index]:
                    continue
                self._zones_active[zone_index] = False
                zone_type = self._zone_types[zone_index]

                if zone_type % 2 == 0:
                    self.apply_heal(zone_type > 0)
                elif zone_type % 3 == 0:
                    self.apply_bullet_refill(zone_type > 0)
                elif zone_type == 1:
                    self.move_disable[robot_index] = 10
                else:
                    self.shoot_disable[robot_index] = 10

        #Apply debuffs
        for robot_index, _time in enumerate(self.move_disable):
            if _time > 0:
                actions[robot_index] = no_op
                self.move_disable[robot_index] = round(self.move_disable[robot_index] - self._running_step, 1)

        for robot_index, _time in enumerate(self.shoot_disable):
            if _time > 0:
                actions[robot_index][3] = 0
                self.shoot_disable[robot_index] = round(self.shoot_disable[robot_index] - self._running_step, 1)

        best_plates = [self.best_plates[i] if self.best_plates[i] or self._robot_hp[i] <= 0 else self.get_largest_enemy_plate(i) for i in range(4)]

        for robot_index, entry in enumerate(best_plates):
            if not entry or self._num_projectiles[robot_index] <= 0 or self._robot_hp[robot_index] <= 0:
                continue
            enemy, plate_index, exp_damage, vel = entry
            self._num_projectiles[robot_index] -= 1
            dmg = armor_plate_damage[plate_index]
            p = exp_damage / (dmg * 1.0)
            if chance(p):
                print('DAMAGE dealt! {}'.format(dmg))
                self._robot_hp[enemy] -= dmg

        for i in range(4):
            self.cmd_vel[i].twist.linear.x, self.cmd_vel[i].twist.linear.y, self.cmd_vel[i].twist.angular.z, self._shoot[i] = actions[i]
            self.pub_cmd_vel[i].publish(self.cmd_vel[i])
            self.pub_gimbal_angle[i].publish(self.cmd_gimbal[i])

        # Unpause Simulation
        self.gazebo.unpauseSim()

        time.sleep(self._running_step)

        # Pause simulation
        self.gazebo.pauseSim()

        # calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, actions)
        done = self._timestep >= self._max_timesteps

        if self._statistics_gui:
            self.update_statistics({'HP': self._robot_hp,
                                    'MD': self.move_disable,
                                    'SD': self.shoot_disable,
                                    'CE': [None if k is None else (k[0], k[1]) for k in best_plates],
                                    'BL': self._num_projectiles})
        return state, reward, done, {}

    def reset(self):
        self.on_init()
        self.step([no_op] * 4)
        self.gazebo.pauseSim()
        self.gazebo.resetSim()

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        #self.controllers_object.reset_monoped_joint_controllers()

        self.gazebo.pauseSim()
        state = self.get_state()

        return state

    def get_state(self):
        # xyz position, robot angle, gimbal angle, number of projectiles, barrel heat of shooter, robot hp
        # only use barrel heat for your robots

        # Sizes of parameters
        # xyz: 3
        # robot angle: 1
        # number_of_projectiles: 1
        # barrel heat: 1
        # robot hp: 1

        if not all(self._odom_info):
            return [None], [None]
        robot_state = [self._odom_info[i][:] + [self._num_projectiles[i], self._barrel_heat[i], self._robot_hp[i]]
                       for i in range(4)]
        return robot_state[:], robot_state[:]

    def get_reward(self, state, actions):
        return 0

    def _start_rospy(self):
        # Bring up node
        rospy.init_node('gym_env_node')

        # Set up publishers for motion
        self.pub_cmd_vel = [rospy.Publisher('roborts_{0}/cmd_vel_acc'.format(i + 1), TwistAccel, queue_size=1) for i in
                            range(4)]
        self.cmd_vel = [TwistAccel() for i in range(4)]
        self.pub_gimbal_angle = [
            rospy.Publisher('roborts_{0}/cmd_gimbal_angle'.format(i + 1), GimbalAngle, queue_size=1) for i in range(4)]
        self.cmd_gimbal = [GimbalAngle() for i in range(4)]

        # Set up subscribers for state
        self.sub_odom = [
            rospy.Subscriber('roborts_{0}/ground_truth/state'.format(i + 1), Odometry, self.odometry_callback) for i in
            range(4)]
        self.sub_gimbal_angle = [
            rospy.Subscriber('roborts_{0}/joint_states'.format(i + 1), JointState, self.gimbal_angle_callback) for i in
            range(4)]
        return self

if __name__ == '__main__':
    run_ros = True
    if len(sys.argv) > 1:
        run_ros = not sys.argv[1] == 'no_ros=True'
    # This section runs roslaunch from script.
    if run_ros:
        ros_from_python = rosFromPython()

    env = RobomasterEnv(True)._start_rospy()
    patrol_strat = PatrolStrategy([6, 7, 15, 14])
    get_buff_strat = GetBuffStrategy()
    team_attack_strat = StartOfGameGetBulletStrategy()
    # team_attack_strat = TeamAttackStrategy()
    # team_attack_strat = StartOfGameWrapper(team_attack_strat)

    def dummy_strategy(robot_index):
        if robot_index == 0:
            return patrol_strat.pick(env, robot_index)
        if robot_index == 1:
            return get_buff_strat.pick(env, robot_index)

    for _ in range(1800):
        env._timestep += 1
        if env._timestep % 600 == 0:
            env.spawn_buff_zones()
        for team in [(0, 1), (2, 3)]:
            index1, index2 = team
            if env._robot_hp[index1] <= 0 and env._robot_hp[index2] <= 0:
                print('GAME OVER! Team {} died.'.format({team}))
                exit()
        test_cmds = [env.waypoint_to_cmd(j, dummy_strategy(j)) for j in range(2)] + \
            [env.waypoint_to_cmd(j + 2, team_attack_strat.pick(env, 2, 3)[j]) for j in range(2)]
        state, reward, done, info = env.step(test_cmds)
        time.sleep(0.01)
        if done:
            env.reset()

    if run_ros:
        ros_from_python.shutdown()
