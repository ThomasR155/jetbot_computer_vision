#!/bin/bash
#set -e
cd /workspace/src
git clone https://github.com/CreedyNZ/rplidar_ros2
cd /workspace

printf "Building workspace...\n\n"
colcon build --symlink-install --event-handlers console_direct+

printf "\nSourcing install/local_setup.bash\n"
source install/local_setup.bash
#printf "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH\n"
