#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from nav2_msgs.msg import NodeInfo
import logging
import math
from nav_msgs.msg import Path  # Added import

"""
Basic navigation demo to using the route server.
"""

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def toPoseStamped(pt, header):
    pose = PoseStamped()
    pose.pose.position.x = pt.x
    pose.pose.position.y = pt.y
    pose.header = header
    return pose

def yaw_to_quaternion(yaw):
    half_yaw = math.radians(yaw/2)
    quaternion_z = math.sin(half_yaw)
    quaternion_w = math.cos(half_yaw)
    return quaternion_z, quaternion_w


def main():
    rclpy.init()

    navigator = BasicNavigator()
    node_info_publisher = navigator.create_publisher(NodeInfo, 'node_info', 10)


    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x =  31.59405726405553
    goal_pose.pose.position.y =  -76.05323961914061
    # goal_pose.pose.position.z = 0.0

    yaw_angle =90.0
    quaternion_z , quaternion_w = yaw_to_quaternion(yaw_angle)

    goal_pose.pose.orientation.z = quaternion_z
    goal_pose.pose.orientation.w = quaternion_w

    # Sanity check a valid route exists using PoseStamped.
    # May also use NodeIDs on the graph if they are known by passing them instead as `int`
    # [path, route] = navigator.getRoute(initial_pose, goal_pose)

    # May also use NodeIDs on the graph if they are known by passing them instead as `int`
    navigator.getAndTrackRoute(initial_pose, goal_pose)

    # Note for the route server, we have a special route argument in the API b/c it may be
    # providing feedback messages simultaneously to others (e.g. controller or WPF as below)


    isTrackingRoute = True
    task_canceled = False
    last_feedback = None
    while not navigator.isTaskComplete(trackingRoute=isTrackingRoute):
        # Do something with the feedback, which contains the route / path if tracking
        feedback = navigator.getFeedback(trackingRoute=isTrackingRoute)

        # Check if feedback is not None before accessing its attributes
        if feedback is not None:
            node_info_msg = NodeInfo()
            node_info_msg.unfinished_node_id = feedback.next_node_id
            node_info_msg.finished_node_id = feedback.last_node_id
            node_info_msg.present_edge_id = feedback.current_edge_id  # Replace with your logic
            node_info_msg.target_node_id = feedback.next_node_id

            logger.info(f"Publishing NodeInfo: {node_info_msg}")

            while feedback is not None:
                if not last_feedback or \
                    (feedback.last_node_id != last_feedback.last_node_id or \
                    feedback.next_node_id != last_feedback.next_node_id):
                    print('Passed node ' + str(feedback.last_node_id) +
                          ' to next node ' + str(feedback.next_node_id) +
                          ' along edge ' + str(feedback.current_edge_id) + '.')

                last_feedback = feedback
                if feedback.rerouted:
                   
                    closest_pt_on_edge_x = feedback.closest_pt_on_edge_x
                    closest_pt_on_edge_y = feedback.closest_pt_on_edge_y
                    closest_pt_on_edge = PoseStamped()
                    closest_pt_on_edge.header.frame_id = 'map'
                    closest_pt_on_edge.header.stamp = navigator.get_clock().now().to_msg()
                    closest_pt_on_edge.pose.position.x = closest_pt_on_edge_x
                    closest_pt_on_edge.pose.position.y = closest_pt_on_edge_y

                    print(f"Closest point on the edge: x={closest_pt_on_edge_x}, y={closest_pt_on_edge_y}")
                    pathh = navigator.getPath(initial_pose, closest_pt_on_edge)

                    # if pathh is not None:
                    #     smoothed_path = navigator.smoothPath(pathh)
                    #     closest_pt_index = None
                    #     for i, point in enumerate(feedback.path.poses):
                    #         if point.pose.position.x == closest_pt_on_edge_x and point.pose.position.y == closest_pt_on_edge_y:
                    #             closest_pt_index = i
                    #             break

                    #     if closest_pt_index is not None:
                    #         del feedback.path.poses[:closest_pt_index]
                    #         for pose in feedback.path.poses[closest_pt_index:]:
                    #             # smoothed_path.poses.extend(pose)
                    #             smoothed_path.poses.extend(feedback.path.poses[closest_pt_index:])

                    #     else:
                    #         print("Failed to find the closest point on the edge in the feedback path.")
                    if pathh is not None:
                        smoothed_path = navigator.smoothPath(pathh)
                        if smoothed_path is not None:  # Check if the smoothed path is not None
                            closest_pt_index = None
                            min_dist = float('inf')  # Initialize minimum distance to a large value

                            # Find the closest point on the edge to the robot's current position
                            for i, point in enumerate(feedback.path.poses):
                                dist = math.sqrt((point.pose.position.x - closest_pt_on_edge_x) ** 2 +
                                                (point.pose.position.y - closest_pt_on_edge_y) ** 2)
                                if dist < min_dist:
                                    closest_pt_index = i
                                    min_dist = dist

                            if closest_pt_index is not None:
                                # Append the path from the closest point to the end
                                smoothed_path.poses.extend(feedback.path.poses[closest_pt_index:])
                            else:
                                print("Failed to find the closest point on the edge in the feedback path.")

                            # Follow the combined path
                            print("Smoothed path:", smoothed_path)
                            navigator.followPath(smoothed_path)
                        else:
                            navigator.followPath(feedback.path)
                    else:
                        print("Failed to get a valid path.")



                                        

                    #     # Follow the combined path
                    #     navigator.followPath(smoothed_path)
                    #     # navigator.followPath(feedback.path)
                    # else:
                    #     print("Failed to get a valid path.")

                feedback = navigator.getFeedback(trackingRoute=isTrackingRoute)
        # Check if followPath or WPF task is done (or failed), will cancel all
                
       # Check if followPath or WPF task is done (or failed), will cancel all current tasks, including route
        if navigator.isTaskComplete():
            print("Controller or waypoint follower server completed its task!")
            navigator.cancelTask()
            task_canceled = True



    

    # Route server will return completed status before the controller / WPF server
    # so wait for the actual robot task processing server to complete
    while not navigator.isTaskComplete() and not task_canceled:
        pass

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
