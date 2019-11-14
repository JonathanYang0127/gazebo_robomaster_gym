#! /usr/bin/env python
import rospy

from roborts_msgs.msg import TwistAccel, GimbalAngle
from std_msgs.msg import Empty
from keyboard.msg import Key
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_connection import GazeboConnection
import math
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
        self._robot_hp = [[2000], [2000], [2000], [2000]]
        self._num_projectiles = [[50], [50], [50], [50]]
        self._barrel_heat = [[0], [0], [0], [0]]
        self._launch_velocity_max = 25
        self._shoot = [0, 0, 0, 0]

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

        #Gives coordinates for the obstacles:
        #x-start, x-end, y-start, y-end
        #4 lines per obstacle
        parallel_obstacles = [
        [1.500, 1.750, 0, 0], [1.500, 1.750, 1.000, 1.000], [1.500, 1.500, 0, 1.000], [1.750, 1.750, 0, 1.000],
        [3.600, 4.600, 1.000, 1.000], [3.600, 4.600, 1.250, 1.250], [3.600, 3.600, 1.000, 1.250], [4.600, 4.600, 1.000, 1.250],
        [7.100, 8.100, 1.000, 1.000], [7.100, 8.100, 1.250, 1.250], [7.100, 7.100, 1.000, 1.250], [8.100, 8.100, 1.000, 1.250],
        [1.500, 2.300, 2.425, 2.425], [1.500, 2.300, 2.675, 2.675], [1.500, 1.500, 2.425, 2.675], [2.300, 2.300, 2.425, 2.675],
        [5.800, 6.600, 2.425, 2.425], [5.800, 6.600, 2.675, 2.675], [5.800, 5.800, 2.425, 2.675], [6.600, 6.600, 2.425, 2.675],
        [0.000, 1.000, 3.850, 3.850], [0.000, 1.000, 4.100, 4.100], [0.000, 0.000, 3.850, 4.100], [1.000, 1.000, 4.100, 4.100],
        [3.500, 4.500, 3.850, 3.850], [3.500, 4.500, 4.100, 4.100], [3.500, 3.500, 3.850, 4.100], [4.500, 4.500, 3.850, 4.100],
        [6.350, 6.600, 4.100, 4.100], [6.350, 6.600, 5.100, 5.100], [6.350, 6.350, 4.100, 5.100], [6.600, 6.600, 4.100, 5.100]]
        #points [x_1, y_1, x_2, y_2]
        center_obstacle = [[3600-300 / math.sqrt(2), 2550, 3600, 2550 + 300 / math.sqrt(2)],
            [3600-300 / math.sqrt(2), 2550, 3600, 2550 - 300 / math.sqrt(2)],
            [3600, 2550 + 300 / math.sqrt(2), 3600 + 300/ math.sqrt(2), 2550],
            [3600, 2550 - 300 / math.sqrt(2), 3600 + 300/ math.sqrt(2), 2550]]

        #These are the coordinates of the debuff zone:
        self._zones = [
        [3.830, 4.370, 0.270, 0.750],
        [1.630, 2.170, 1.695, 2.175],
        [0.230, 0.770, 3.120, 3.600],
        [4.350, 4.830, 4.500, 5.040], 
        [5.330, 7.870, 1.500, 1.980],
        [5.930, 6.470, 2.925, 3.405]]
        self._zones_active = [False] * 6

        #Zone Types:
        #0: refill
        #1: hp recovery
        #2: no movement debuff
        #3: no shoot debuff
        self._zone_types = [0, 3, 2]

        #Effects per robot
        self._robot_effects = [dict(), dict(), dict(), dict()]

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z 

    def odometry_callback(self, msg):
        #print(self._odom_info)
        self._odom_info[int(msg._connection_header['topic'][9]) - 1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, self.quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]]

    def gimbal_angle_callback(self, msg):
        self._gimbal_angle_info[int(msg._connection_header['topic'][9]) - 1] = msg.position 
        return

    def _timestep_to_real_time(self):
        #converts timestep to real time
        return self._timestep * self._running_step * self._real_time_conversion 

    def _check_in_zone(self, position):
        #Input: x y z position
        #Checks if robot are in a buff/debuff zone, returns the zone
        for i in range(len(self._zones)):
            edge_buffer = 0.05 if self._zone_types[i%3] == 0 or self._zone_types[i%3] == 1 else -0.05
            if position[0] >= self._zones[i][0] + edge_buffer and position[0] <= self._zones[i][1] - edge_buffer and position[1] >= self._zones[i][2] + edge_buffer and position[1] <= self._zones[i][3] - edge_buffer:
                return i
        return None

    def step(self, action1, action2): 
        #x velocity, y velocity, twist angular, shoot

        if len(action1) != 8 or len(action2) != 8:
            print("Invalid action!")
            return None, None, None, {} 

        #Check for/update debuffs to robots if in zone
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
                    #print(zone, self._odom_info[position])
                if zone is None:
                    continue
                elif self._zone_types[zone%3] == 0 or self._zone_types[zone%3] == 1:
                    if zone < 3:
                        self._robot_effects[0][self._zone_types[zone%3]] = True
                        self._robot_effects[1][self._zone_types[zone%3]] = True
                    else:
                        self._robot_effects[2][self._zone_types[zone%3]] = True
                        self._robot_effects[3][self._zone_types[zone%3]] = True
                else:
                    self._robot_effects[position][self._zone_types[zone%3]] = 10
         
        #Apply buffs/debuffs
        for robot_number in range(len(self._robot_effects)):
            if self._robot_effects[robot_number].get(0, False):
                self._num_projectiles[robot_number] += 100
                self._robot_effects[robot_number][0] = False
            if self._robot_effects[robot_number].get(1, False):
                self._robot_hp[robot_number] = min(2000, self.robot_hp[robot_number] + 200)
                self._robot_effects[robot_number][1] = False
            if  self._robot_effects[robot_number].get(2, 0) > 0:
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
                self._robot_effects[robot_number] -= self._running_step
 
        #Set action parameters for publisher
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

        #Publish actions and execute for running_step time
        for i in range(4):
            self.pub_cmd_vel[i].publish(self.cmd_vel[i])
            self.pub_gimbal_angle[i].publish(self.cmd_gimbal[i])

        #Unpause Simulation
        self.gazebo.unpauseSim()
    
        time.sleep(self._running_step) 
        
        #Pause simulation
        self.gazebo.pauseSim()

        #calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, action1, action2)
        done = self._timestep >= self._max_timesteps

        return state, reward, done, {}
       

    def reset(self):
        self._zones_active = [False] * 6
        self._zone_types = [0, 2, 3]
        self._robot_debuffs = [[], [], [], []]
        self._timestep = 0
        self.real_time = 0
        self._robot_hp = [[2000], [2000], [2000], [2000]]
        self._num_projectiles = [[50], [50], [50], [50]]
        self._barrel_heat = [[0], [0], [0], [0]]
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
        #robot angle: 1
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
        state, reward, done, info = env.step([0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0])
        time.sleep(0.01)
        if done:
            env.reset()
    rospy.spin()
