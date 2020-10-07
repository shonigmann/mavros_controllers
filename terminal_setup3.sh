#Check environment setup
cd ~/src/Firmware
DONT_RUN=1 make px4_sitl_default gazebo

#Source PX4 environment
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

#Run and quit PX4 (not sure if necessary)
#roslaunch px4 posix_sitl.launch
# Get its PID
#PID=$!
#sleep 2
#kill -INT $PID

#Return to mavros controller directory (not sure if necessary)
cd ~/catkin_ws/src/
#roslaunch geometric_controller sitl_trajectory_track_circle.launch
#roslaunch trajectory_publisher sitl_trajectory_setpointraw.launch
roslaunch geometric_controller sitl_trajectory_track_manual_2.launch


