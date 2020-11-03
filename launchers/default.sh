#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
rm -f /tmp/.X1-lock
dt-exec Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset
export DISPLAY=:1
roslaunch duckiegym-ros-wrapper duckiegym_ros_wrapper_node.launch

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
