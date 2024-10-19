# custom_path_follow_with_obstacle_handling
This project involves a custom navigation setup where a robot is tasked with following a predetermined global path using the Regulated Pure Pursuit Controller as a local planner. The primary goal is to ensure that the robot strictly follows the global path without triggering any replanning mechanisms. Additionally, the system is integrated with the Nav2 Collision Monitor to ensure safe navigation, stopping the robot when obstacles are detected and resuming movement once the path is cleared.

Key Features
Global Path Restriction:
The default NavToPose behavior tree in ROS 2's navigation stack is modified to disable any replanning of the global path. This ensures that once a global path is defined, the robot will strictly adhere to it, without deviations or recalculations, even if obstacles appear on the route.

Local Planner - Regulated Pure Pursuit:
The Regulated Pure Pursuit Controller is utilized as the local planner. This controller ensures that the robot follows the provided global plan accurately while regulating speed and maintaining a safe distance from potential obstacles. The controller efficiently navigates the robot along the global path while maintaining adherence to the planned trajectory.

Collision Monitoring:
Integrated with the Nav2 Collision Monitor, the system continuously scans the environment for potential collisions. If any obstacles are detected within the robot's path, the robot will stop immediately, ensuring safety. Once the obstacle is removed, the robot resumes its journey towards the goal, without requiring any global path replanning.

Custom Global Path:
A custom straight-line path of (5 meters) is defined and passed to the navigation system as the global path from cpp_node . This is a fixed path that the robot must follow using the Regulated Pure Pursuit local planner.


Steps to recreate
1. git clone Repo
2. colcon build
3. cd nav2_ws && source install/setup.bash && ros2 launch nav2_simple_commander route_example_launch.py 
4. cd nav2_ws && source install/setup.bash && ros2 launch nav2_bringup localization_launch.py 
5. cd nav2_ws && source install/setup.bash && ros2 launch nav2_bringup navigation_launch.py 
6. cd nav2_ws && source install/setup.bash && ros2 launch nav2_collision_monitor collision_monitor_node.launch.py
7. cd nav2_ws/src && python3 robot_pose.py
8. cd nav2_ws && source install/setup.bash && ros2 run cpp_node send_custom_path_node

Working video can be seen [Screencast from 10-12-2024 05_50_29 PM_trimmed.webm](https://github.com/user-attachments/assets/e648d06b-ac64-4ccb-aec0-8a127be798cd)

