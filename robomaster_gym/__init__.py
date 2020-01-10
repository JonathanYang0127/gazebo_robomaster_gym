from gym.envs.registration import register

register(id='robomaster-env-v0',
        entry_point='robomaster_gym.envs:RobomasterEnv',
        )
register(id='closequarter-env-v0',
        entry_point='robomaster_gym.envs:CloseQuarter',
        )
