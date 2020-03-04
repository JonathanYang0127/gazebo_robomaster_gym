#!/bin/bash

# turn on bash's job control
set -m

# Start the primary process and put it in the background
python3 robomaster_gym/envs/robomaster_env.py &
env_pid = $!

# Start the helper process'
sleep 60
rostopic roborts_1/ground_truth/state >> output.txt
rostopic roborts_2/ground_truth/state >> output.txt
rostopic roborts_3/ground_truth/state >> output.txt
rostopic roborts_4/ground_truth/state >> output.txt
