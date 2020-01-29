import robomaster_gym.misc.geometry as geometry
import strategies.optimizers.constants as CONSTANTS

class RotationalPlanner:
    def __init__(self, get_game_state):
        """
        :param get_game_state: a function that returns the state of the game, consists of
                [friendly_state_1, friendly_state_2, enemy_state_1, enemy_state_2]
                state = [x, y, yaw, amo, heat, health]
        """
        self.get_game_state = get_game_state

    def rotation_penalty(self, robot_id):
        """
        TODO: finish the logic
        :param robot_id: the robot to control
        :return: a penalty for rotation
        logics:
            assuming rotation is to attack, therefore
            1. if amo < threshold, add penalty
            2. if rotation does will not let us face any enemy, add penalty
            3. if number of friendly robots < number of enemy robots PRESENT AT THE SCENE, add penalty
        """
        assert robot_id <= 1   # cannot operate enemy robots, obviously
        robot_state = self.get_game_state()[robot_id]
        penalty = 0
        """logic 1"""
        if robot_state[3] <= CONSTANTS.AMO_THRESHOLD:
            penalty += CONSTANTS.AMO_THRESHOLD_VIOLATION_PENALTY

        """logic 2"""
        if self.enemies_around() == 0:
            penalty += CONSTANTS.EMPTY_ROTATION_PENALTY

        """logic 3"""
        if self.friendlies_around() < self.enemies_around():
            penalty += CONSTANTS.DISADVANTAGEOUS_ENGAGEMENT_PENALTY
        return penalty

    def enemies_around(self):
        return 0

    def friendlies_around(self):
        return 1


## angle is followed by the convention of complex field
def compute_optimal_angle(self_position, self_angle, enemy_position):
    x_self, y_self = self_position
    x_enemy, y_enemy = enemy_position
    raw_angle = geometry.angleTo(x_self, y_self, x_enemy, y_enemy)
    return raw_angle - self_angle

