import gym
import robomaster_gym
import roslaunch

if __name__ == '__main__': 
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args1 = ['roborts_sim','multi_robot.launch']
    cli_args2 = ['roborts_bringup','roborts_gazebo.launch','gui:=true']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
    roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
    roslaunch_args2 = cli_args2[2:]
    launch_files = [roslaunch_file1,(roslaunch_file2,roslaunch_args2)]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()

    env = gym.make('closequarter-env-v0')._start_rospy()