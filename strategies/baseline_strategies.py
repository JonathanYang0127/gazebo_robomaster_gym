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

        # if robot1_hp < self.low_health_threshold and robot2_hp >= self.low_health_threshold:
        #     if env._robot_hp[enemy_team[0]] <= 0 or env._robot_hp[enemy_team[1]] <= 0:
        #         alive_enemy_robot_index = 0 if env._robot_hp[enemy_team[0]] > 0 else 1
        #         if robot1_hp + robot2_hp >= env._robot_hp[enemy_team[alive_enemy_robot_index]] + self.low_health_threshold:
        #             return get_bullet_refill_if_available(env, robot_index1), \
        #                 simple_attack_sub_strategy(env, robot_index2, env.get_enemy_team[alive_enemy_robot_index].index) if robot2_bullet > 0 else get_bullet_refill_if_available(env, robot_index2)
                # else:
                #     return simple_attack_sub_strategy(env, robot_index1, get_closest_enemy(env, robot_index1)) if robot1_bullet > 0 else get_bullet_refill_if_available(env, robot_index1), \
                #         simple_attack_sub_strategy(env, robot_index2, get_closest_enemy(env, robot_index2)) if robot2_bullet > 0 else get_bullet_refill_if_available(env, robot_index2)
            #else:
                #need a (or maybe 2-3) helper function(s) to implement the action of (my robot w/ higher HP) take the damage
                #while (both of my robots) attack (closest enemy) without exposure to (both enemies)
        # elif robot1_hp >= self.low_health_threshold and robot2_hp < self.low_health_threshold:
        #     if env._robot_hp[enemy_team[0]] <= 0 or env._robot_hp[enemy_team[1]] <= 0:
        #         alive_enemy_robot_index = 0 if env._robot_hp[enemy_team[0]] > 0 else 1
        #         if robot1_hp + robot2_hp >= env._robot_hp[enemy_team[alive_enemy_robot_index]] + self.low_health_threshold:
        #             return simple_attack_sub_strategy(env, robot_index1, env.get_enemy_team[alive_enemy_robot_index].index) if robot1_bullet > 0 else get_bullet_refill_if_available(env, robot_index1), \
        #                 get_bullet_refill_if_available(env, robot_index2)
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

class StartOfGameGetBulletStrategy():

    zone_rush_plan_team01 = {
        2: 0,
        8: 0,
        10: 0,
        11: 1,
    }
    defense_points_team01 = [1, 5]

    def __init__(self):
        self.zone_rush_plan = [self.zone_rush_plan_team01, { 21 - k: v for k, v in self.zone_rush_plan_team01.items() }]
        self.defense_points = [self.defense_points_team01, [ 21 - v for v in self.defense_points_team01 ]]
        print(self.zone_rush_plan)

    def get_rush_plan(self, robot_index):
        return self.zone_rush_plan[robot_index // 2]

    def get_defense_points(self, robot_index):
        return self.defense_points[robot_index // 2]

    def pick(self, env, robot_index1, robot_index2):
        plan = self.get_rush_plan(robot_index1)
        dpoints = self.get_defense_points(robot_index1)
        _zones = env._zone_types
        zones_active = env._zones_active
        goals = [None, None]
        indices = [robot_index1, robot_index2]

        bullet_refill_index = _zones.index(bullet_refill_zone_rep(robot_index1))
        bullet_refill_node = env.navigator.get_point_at_zone(bullet_refill_index)
        if bullet_refill_node in plan:
            print(zones[bullet_refill_index])
            print(bullet_refill_node)
            i = plan[bullet_refill_node]
            if zones_active[bullet_refill_index]:
                goals[i] = env.navigator.get_point_at_zone(bullet_refill_index)
            else:
                goals[i] = dpoints[i]

            enemy_refill_index = _zones.index(hp_refill_zone_rep(3 - robot_index1))
            enemy_hp_refill_node = env.navigator.get_point_at_zone(enemy_refill_index)
            if zones_active[enemy_refill_index] and enemy_hp_refill_node in plan and \
                sum([env._robot_hp[i] for i in env.get_enemy_team(robot_index1)]) > robot_max_hp * 2 - 60:
                print(zones[enemy_refill_index])
                goals[1 - i] = env.navigator.get_point_at_zone(enemy_refill_index)
            else:
                goals[1 - i] = dpoints[1 - i]
            return goals
        print("Other strategy. Not implemented")
        return None, None


class StartOfGameWrapper():

    time_cap = 100

    def __init__(self, pre_strat, post_strat):
        self.pre_strat = pre_strat
        self.post_strat = post_strat

    def pick(self, env, robot_index1, robot_index2):
        if env._timestep < self.time_cap:
            return self.pre_strat.pick(env, robot_index1, robot_index2)
        return self.post_strat.pick(env, robot_index1, robot_index2)


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
    target_zone = bullet_refill_zone_rep(robot_index)
    _index = env._zone_types.index(target_zone)
    if env._zones_active[_index]:
        return env.navigator.get_point_at_zone(_index)
