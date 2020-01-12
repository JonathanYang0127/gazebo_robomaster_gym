import rospy
import gym
import math
import time

from roborts_msgs.msg import TwistAccel, GimbalAngle
from std_msgs.msg import Empty
# from keyboard.msg import Key
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from robomaster_gym.misc.gazebo_connection import GazeboConnection
from robomaster_gym.misc.geometry import *
from robomaster_gym.misc.constants import *


class RobomasterEnv(gym.Env):
    def __init__(self, statistics_gui=False):
        # Set up state parameters
        self._odom_info = [[None]] * 4
        self._gimbal_angle_info = [[None]] * 4
        self._robot_hp = [robot_max_hp] * 4
        self._num_projectiles = [50, 0, 50, 0]
        self._barrel_heat = [0] * 4
        self._shoot = [0, 0, 0, 0]

        self.damage_threshold = 6

        self.robot_coords = [None] * 4
        self.robot_plates_coords = [None] * 4

        # Amount of time running per timestep
        self._running_step = 0.1

        # Max number of timesteps
        self._timestep = 0
        self._max_timesteps = 1000

        # Conversion from timestep to real time
        self._real_time_conversion = 1

        # initialize segments of each obstacle for blocking calculation
        # buffer is added
        self.obstacle_buffer = 0.015
        self.segments = generate_obstacle_segments(self.obstacle_buffer)

        self._zones_active = [False] * 6

        # Zone Types:
        # 0: refill
        # 1: hp recovery
        # 2: no movement debuff
        # 3: no shoot debuff
        self._zone_types = [0, 3, 2]

        # Effects per robot
        self._robot_effects = [dict(), dict(), dict(), dict()]

        # Set up gazebo connection
        self.gazebo = GazeboConnection()
        self.gazebo.pauseSim()

    def check_collision_with_obstacle(robot1, robot2):
        pass

    def get_enemy_team(self, robot_index):
        if robot_index < 2:
            return (2, 3)
        return (0, 1)

    def update_robot_coords(self, _id):
        self.robot_coords[_id] = robot_coords_from_odom(self._odom_info[_id])
        self.robot_plates_coords[_id] = plate_coords_from_odom(self._odom_info[_id])

    # return whether the line (x1, y1) - (x2, y2) is blocked by anything
    # pass in from_robot_index so it doesn't check for the robot itself!
    def is_line_of_sight_blocked(self, x1, y1, x2, y2, from_robot_index=None):
        # uninitiated
        if not self.robot_coords:
            return True
        for robot_index in range(4):
            if robot_index != from_robot_index:
                _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 = self.robot_coords[robot_index]
                if lines_cross(x1, y1, x2, y2, _x1, _y1, _x3, _y3) or lines_cross(x1, y1, x2, y2, _x2, _y2, _x4, _y4):
                    return True
        for xl, yl, xh, yh in self.segments:
            if lines_cross(x1, y1, x2, y2, xl, yl, xh, yh):
                return True
        return False

    # TODO: determine by experiment
    # should also return optimal shoot velocity
    def expected_damage_with_optimal_shoot_vel(self, damage, angle, distance):
        return math.sin(angle) * damage, damage

    # return the index and optimal shoot velocity of best enemy plate visible
    # None if no index is visible
    def get_best_enemy_plate(self, robot_index):
        x, y, z, yaw = self._odom_info[robot_index]
        best_plate = None
        damage = self.damage_threshold
        for enemy in self.get_enemy_team(robot_index):
            for index in range(4):
                x1, y1, x2, y2 = self.robot_plates_coords[enemy][index]
                center_x, center_y = midpoint(x1, y1, x2, y2)
                dis = distance(x, y, center_x, center_y)
                angle_diff = angleDiff(angleTo(x, y, center_x, center_y), yaw)
                if dis < gimbal_range_dis and angle_diff <= gimbal_range_angle:
                    exp_damage, vel = self.expected_damage_with_optimal_shoot_vel(armor_plate_damage[index], angle_diff,
                                                                                  dis)
                    if exp_damage > damage:
                        if not self.is_line_of_sight_blocked(x, y, center_x, center_y, robot_index):
                            best_plate = (enemy, index, vel)
                            damage = exp_damage
        return best_plate

    def odometry_callback(self, msg):
        _id = int(msg._connection_header['topic'][9]) - 1
        self._odom_info[_id] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                                quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]]
        self.update_robot_coords(_id)

    def gimbal_angle_callback(self, msg):
        self._gimbal_angle_info[int(msg._connection_header['topic'][9]) - 1] = msg.position
        return

    def _timestep_to_real_time(self):
        # converts timestep to real time
        return self._timestep * self._running_step * self._real_time_conversion

    def _check_in_zone(self, position):
        # Input: x y z position
        # Checks if robot are in a buff/debuff zone, returns the zone
        for i in range(len(zones)):
            edge_buffer = 0.05 if self._zone_types[i % 3] == 0 or self._zone_types[i % 3] == 1 else -0.05
            if position[0] >= zones[i][0] + edge_buffer and position[0] <= zones[i][1] - edge_buffer and position[1] >= \
                    zones[i][2] + edge_buffer and position[1] <= zones[i][3] - edge_buffer:
                return i

    def step(self, action1, action2):
        # x velocity, y velocity, twist angular, shoot

        if len(action1) != 8 or len(action2) != 8:
            print("Invalid action!")
            return None, None, None, {}

            # Check for/update debuffs to robots if in zone
        for position in range(len(self._odom_info)):
            if self._odom_info[position][0] is not None:
                zone = self._check_in_zone(self._odom_info[position])
                if zone is None:
                    continue
                if not self._zones_active[zone]:
                    self._zones_active[zone] = True
                else:
                    continue
                if position == 0:
                    print(zone)
                    print(self._zone_types[zone % 3])
                    # print(zone, self._odom_info[position])
                if zone is None:
                    continue
                elif self._zone_types[zone % 3] == 0 or self._zone_types[zone % 3] == 1:
                    if zone < 3:
                        self._robot_effects[0][self._zone_types[zone % 3]] = True
                        self._robot_effects[1][self._zone_types[zone % 3]] = True
                    else:
                        self._robot_effects[2][self._zone_types[zone % 3]] = True
                        self._robot_effects[3][self._zone_types[zone % 3]] = True
                elif self._zone_types[zone % 3] == 2:
                    self._robot_effects[position][self._zone_types[zone % 3]] = 10

        #print(self._robot_effects[0].get(2, 0))
        # Apply buffs/debuffs
        for robot_number in range(len(self._robot_effects)):
            if self._robot_effects[robot_number].get(0, False):
                self._num_projectiles[robot_number] += 100
                self._robot_effects[robot_number][0] = False
            if self._robot_effects[robot_number].get(1, False):
                self._robot_hp[robot_number] = min(2000, self.robot_hp[robot_number] + 200)
                self._robot_effects[robot_number][1] = False
            if self._robot_effects[robot_number].get(2, 0) > 0:
                if robot_number == 0:
                    action1[0] = 0
                    action1[1] = 0
                    action1[2] = 0
                elif robot_number == 1:
                    action1[4] = 0
                    action1[5] = 0
                    action1[6] = 0
                elif robot_number == 2:
                    action2[0] = 0
                    action2[1] = 0
                    action2[2] = 0
                else:
                    action2[4] = 0
                    action2[5] = 0
                    action2[6] = 0
                self._robot_effects[robot_number][2] -= self._running_step
            if self._robot_effects[robot_number].get(3, 0) > 0:
                if robot_number == 0:
                    action1[3] = 0
                elif robot_number == 1:
                    action1[7] = 0
                elif robot_number == 2:
                    action2[3] = 0
                elif robot_number == 3:
                    action2[7] = 0
                self._robot_effects[robot_number][3] -= self._running_step

        # Set action parameters for publisher
        self.cmd_vel[0].twist.linear.x = action1[0]
        self.cmd_vel[0].twist.linear.y = action1[1]
        self.cmd_vel[0].twist.angular.z = action1[2]
        self._shoot[0] = action1[3]
        self.cmd_vel[1].twist.linear.x = action1[4]
        self.cmd_vel[1].twist.linear.y = action1[5]
        self.cmd_vel[1].twist.angular.z = action1[6]
        self._shoot[1] = action1[7]

        self.cmd_vel[2].twist.linear.x = action2[0]
        self.cmd_vel[2].twist.linear.y = action2[1]
        self.cmd_vel[2].twist.angular.z = action2[2]
        self._shoot[2] = action2[3]
        self.cmd_vel[3].twist.linear.x = action2[4]
        self.cmd_vel[3].twist.linear.y = action2[5]
        self.cmd_vel[3].twist.angular.z = action2[6]
        self._shoot[3] = action2[7]

        # Publish actions and execute for running_step time
        for i in range(4):
            self.pub_cmd_vel[i].publish(self.cmd_vel[i])
            self.pub_gimbal_angle[i].publish(self.cmd_gimbal[i])

        # Unpause Simulation
        self.gazebo.unpauseSim()

        time.sleep(self._running_step)

        # Pause simulation
        self.gazebo.pauseSim()

        # calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, action1, action2)
        done = self._timestep >= self._max_timesteps

        return state, reward, done, {}

    def reset(self):
        self._zones_active = [False] * 6
        self._zone_types = [0, 3, 2]
        self._robot_debuffs = [[], [], [], []]
        self._timestep = 0
        self.real_time = 0
        self._robot_hp = [robot_max_hp] * 4
        self._num_projectiles = [50, 0, 50, 0]
        self._barrel_heat = [0] * 4
        self.gazebo.pauseSim()
        self.gazebo.resetSim()

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        # self.controllers_object.reset_monoped_joint_controllers()

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

        robot_state = [list(self._odom_info[i]) + [self._num_projectiles[i], self._barrel_heat[i], self._robot_hp[i]]
                       for i in range(4)]
        return [robot_state[0] + robot_state[1] + robot_state[2] + robot_state[3],
                robot_state[0] + robot_state[1] + robot_state[2] + robot_state[3]]

    def get_reward(self, state, action1, action2):
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
    env = RobomasterEnv()._start_rospy()
    for i in range(1000):
        state, reward, done, info = env.step([0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0])
        time.sleep(0.01)
        if done:
            env.reset()
