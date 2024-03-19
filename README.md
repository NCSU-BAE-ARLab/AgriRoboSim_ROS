# ROS-UE5 Agricultural Robot Simulation (ROS)
This repo host the ROS part for the simulation, there is a seperate [UE5 repo](https://github.com/XingjianL/AgriRobSim_UE5) that work along side this simulation.

`benchbot_multi_eef_moveit_configs` demos the `2-robot_plant_vis` movement, image capturing, and visualization.

`tomato_moveit_configs` demos the `ur5e_tomato` movement, image capturing, and visualization.

ROS noetic and moveit is tested, ur_description from [this repo (melodic-devel branch)](https://github.com/ros-industrial/universal_robot.git), and rosbridge from [rosbridge (ros1 branch)](https://github.com/RobotWebTools/rosbridge_suite/tree/ros1).

## Usage
1. clone repo
2. clone ur_description and rosbridge
3. build and source
4. `roslaunch rosbridge_server rosbridge_websocket.launch bson_only_mode:=True websocket_external_port:=80`
5. `roslaunch benchbot_multi_eef_moveit_configs startup.launch` or `roslaunch tomato_moveit_configs startup.launch` depending on the UE5 simulation
6. Run [UE5 simulation](https://github.com/XingjianL/AgriRobSim_UE5)
7. Run `ROS_UE_comm.py`