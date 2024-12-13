#!/bin/bash
# Set the number of robots
declare -i j

# Get the number of robots from a ROS 2 parameter
robots_num=$(ros2 param get /control_terminal robots_num)

# Spawn robots
for ((i=0; i<robots_num; i++))
do
    j=$i

    # Spawn robot task allocation node for each robot
    ros2 run task_allocation task_allocation_node Robot${j} --ros-args -r __name:=Robot${j}_task_allocation &

    # Spawn robot model in Ignition Gazebo
    # Assuming you have the SDF model path correctly set
    ros2 run ros_ign_gazebo spawn_entity -file $(ros2 pkg prefix gazebo_description)/models/Robot/model.sdf -entity Robot${j} -x 0.0 -y 0.0 -z 0.0 &
done
