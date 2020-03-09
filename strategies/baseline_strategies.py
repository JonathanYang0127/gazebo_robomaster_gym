from robomaster_gym.misc.navigation import *

def doNothing(env, robot_index):
    if not all(env._odom_info):
        return
    x, y, yaw = env._odom_info[robot_index]
    return x, y

class PatrolStrategy:

    def __init__(self, path):
        self.path = path
        self.proximity_threshold = 0.05

    def pick(self, env, robot_index):
        if not all(env._odom_info):
            return
        path = self.path
        x, y, _ = env._odom_info[robot_index]
        to_x, to_y = env.navigator.get_point(path[0])
        if distance(x, y, to_x, to_y) < self.proximity_threshold:
            self.path = path[1:] + [path[0]]
        return self.path[0]


class GetBuffStrategy():

    def pick(self, env, robot_index):
        is_team_01 = robot_index <= 1
        target_zones = [2, 3] if is_team_01 else [-2, -3]
        for target_zone in target_zones:
            _index = env._zone_types.index(target_zone)
            if env._zones_active[_index]:
                return env.navigator.get_point_at_zone(_index)


class TeamAttackStrategy():

    def __init__(self):
        self.last_strategy = None
        self.high_health_threshold, self.low_health_threshold = robot_max_hp * 2.0 / 3.0, robot_max_hp * 1.0 / 3.0

    def pick(self, env, robot_index1, robot_index2):
        if not all(env._odom_info):
            return None, None
        robot1_hp = env._robot_hp[robot_index1]
        robot2_hp = env._robot_hp[robot_index2]

        robot1_bullet = env._num_projectiles[robot_index1]
        robot2_bullet = env._num_projectiles[robot_index2]

        enemy_team = env.get_enemy_team(robot_index1)

        if robot1_hp < self.low_health_threshold and robot2_hp >= self.low_health_threshold:
            if env._robot_hp[enemy_team[0]] <= 0 or env._robot_hp[enemy_team[1]] <= 0:
                alive_enemy_robot_index = 0 if env._robot_hp[enemy_team[0]] > 0 else 1
                if robot1_hp + robot2_hp >= env._robot_hp[enemy_team[alive_enemy_robot_index]] + self.low_health_threshold:
                    return get_bullet_refill_if_available(env, robot_index1), \
                        simple_attack_sub_strategy(env, robot_index2, env.get_enemy_team[alive_enemy_robot_index].index) if robot2_bullet > 0 else get_bullet_refill_if_available(env, robot_index2)
                # else:
                #     return simple_attack_sub_strategy(env, robot_index1, get_closest_enemy(env, robot_index1)) if robot1_bullet > 0 else get_bullet_refill_if_available(env, robot_index1), \
                #         simple_attack_sub_strategy(env, robot_index2, get_closest_enemy(env, robot_index2)) if robot2_bullet > 0 else get_bullet_refill_if_available(env, robot_index2)
            #else:
                #need a (or maybe 2-3) helper function(s) to implement the action of (my robot w/ higher HP) take the damage
                #while (both of my robots) attack (closest enemy) without exposure to (both enemies)
        elif robot1_hp >= self.low_health_threshold and robot2_hp < self.low_health_threshold:
            if env._robot_hp[enemy_team[0]] <= 0 or env._robot_hp[enemy_team[1]] <= 0:
                alive_enemy_robot_index = 0 if env._robot_hp[enemy_team[0]] > 0 else 1
                if robot1_hp + robot2_hp >= env._robot_hp[enemy_team[alive_enemy_robot_index]] + self.low_health_threshold:
                    return simple_attack_sub_strategy(env, robot_index1, env.get_enemy_team[alive_enemy_robot_index].index) if robot1_bullet > 0 else get_bullet_refill_if_available(env, robot_index1), \
                        get_bullet_refill_if_available(env, robot_index2)
                # else:
                #     return simple_attack_sub_strategy(env, robot_index1, get_closest_enemy(env, robot_index1)) if robot1_bullet > 0 else get_bullet_refill_if_available(env, robot_index1), \
                #         simple_attack_sub_strategy(env, robot_index2, get_closest_enemy(env, robot_index2)) if robot2_bullet > 0 else get_bullet_refill_if_available(env, robot_index2)
            #else:
                #need a (or maybe 2-3) helper function(s) to implement the action of (my robot w/ higher HP) take the damage
                #while (both of my robots) attack (closest enemy) without exposure to (both enemies)
        # if robot1_hp > self.high_health_threshold and robot2_hp > self.high_health_threshold:
        return simple_attack_sub_strategy(env, robot_index1, get_closest_enemy(env, robot_index1)) if robot1_bullet > 0 else get_bullet_refill_if_available(env, robot_index1), \
            simple_attack_sub_strategy(env, robot_index2, get_closest_enemy(env, robot_index2)) if robot2_bullet > 0 else get_bullet_refill_if_available(env, robot_index2)
        

        # print("SHOULDN'T GET HERE")
        # self.last_strategy = strat
        # return strat.pick(env, robot_index)

def get_closest_enemy(env, robot_index):
    enemy_team = env.get_enemy_team(robot_index)
    enemy_coords = [(index, env._odom_info[index]) for index in enemy_team]
    dis = float('inf')
    x, y, _ = env._odom_info[robot_index]
    for (index, (e_x, e_y, _)) in enemy_coords:
        if distance(e_x, e_y, x, y) < dis:
            dis = distance(e_x, e_y, x, y)
            closest = index
    return closest

def simple_attack_sub_strategy(env, robot_index, enemy_index):
    x, y, yaw = env._odom_info[robot_index]
    if env.best_plates[robot_index] and env.best_plates[robot_index][0] == enemy_index:
        return (x, y)
    enemy_x, enemy_y, enemy_yaw = env._odom_info[enemy_index]
    return (enemy_x, enemy_y)

def get_bullet_refill_if_available(env, robot_index):
    is_team_01 = robot_index <= 1
    target_zone = 3 if is_team_01 else -3
    _index = env._zone_types.index(target_zone)
    if env._zones_active[_index]:
        return env.navigator.get_point_at_zone(_index)
