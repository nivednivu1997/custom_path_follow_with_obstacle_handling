# custom_path_follow_with_obstacle_handling
Task:
Global path can be passed to controller, then using Regulated pursuit as a local planner to follow 
While the robot is following the path it should not replan at any cost but should stop if any obstacles appear, once the obstacle is moved then keeping moving towards the goal

1.Default NavtoPose Behaviour tree is modified to restrict any replanning of global path.
2.Regulated Purepursuit is used as local planner which follows the global plan.
3.Nav2_Collision_Monitor monitors for any possible collisions and stop robot accordingly.

Steps to recreate
1. git clone Repo
2. colcon build
3. cd nav2_ws && source install/setup.bash && ros2 launch nav2_simple_commander route_example_launch.py 
4. cd nav2_ws && source install/setup.bash && ros2 launch nav2_bringup localization_launch.py 
5. cd nav2_ws && source install/setup.bash && ros2 launch nav2_bringup navigation_launch.py 
6. cd nav2_ws && source install/setup.bash && ros2 launch nav2_collision_monitor collision_monitor_node.launch.py 



