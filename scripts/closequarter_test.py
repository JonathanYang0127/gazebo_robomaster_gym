import gym
import robomaster_gym
import roslaunch
import time

if __name__ == '__main__': 
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args1 = ['roborts_sim','multi_robot.launch']
    # cli_args2 = ['roborts_bringup','roborts_gazebo.launch','gui:=true']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
    # roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
    # roslaunch_args2 = cli_args2[2:]
    # launch_files = [roslaunch_file1,(roslaunch_file2,roslaunch_args2)]
    launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_file1])
    launch.start()

    # env = gym.make('closequarter-env-v0')._start_rospy()
    env = gym.make('robomaster-env-v0')._start_rospy()

    # for i in range(1000):
    #     state, reward, done, info = env.step([0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0])
    #     print(env._odom_info)
    #     print(env.robot_coords)
    #     time.sleep(0.01)
    #     if done:
    #         env.reset()