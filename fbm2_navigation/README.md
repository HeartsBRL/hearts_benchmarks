

# Running Turtlebot:

## Bring up:

roslaunch turtlebot_bringup minimal.launch

## Make a map:

roslaunch turtlebot_navigation gmapping_demo.launch

### Save a map:

rosrun map_server map_saver -f <file_name>

## Load a map:
roslaunch turtlebot_navigation amcl_demo.launch map_file:=<file_name>

### eg:
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/tb_ws/maps/map_assistedLiving_02.yaml

roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/tb_ws/maps/run1/map.yaml
# Running ControlBox:

## RVIZ with config
roslaunch turtlebot_rviz_launchers view_navigation.launch


# FBM2 Controller:

## Make a map:

roslaunch hearts_navigation map_creator.launch

## fbm2_controller:

roslaunch fbm2_navigation fbm2_controller.launch


# Running Simulation:

## Bring up:

roslaunch turtlebot_gazebo turtlebot_world.launch

roslaunch turtlebot_teleop keyboard_teleop.launch

roslaunch turtlebot_rviz_launchers view_robot.launch

## Make a map:

roslaunch turtlebot_gazebo gmapping_demo.launch

## Save a map:

rosrun map_server map_saver -f <your map name>

## Load a map:

roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<full path to your map YAML file>




########### Running

roslaunch fbm2_navigation test_controller.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/tb_ws/maps/run2/map.yaml
rosrun hearts_navigation avoider.py
