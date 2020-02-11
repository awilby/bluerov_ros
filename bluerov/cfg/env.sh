#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# source setup.sh from same directory as this file
. "/opt/ros/melodic/setup.sh"
. "/home/bluerov/ros_ws/devel/setup.bash"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/bluerov/ros_ws/src

exec "$@"
