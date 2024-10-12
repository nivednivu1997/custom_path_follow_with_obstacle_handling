import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose  # Import the action message type

def cancel_goal():
    # Initialize ROS node
    rclpy.init()
    node = rclpy.create_node('goal_canceler')

    # Create an action client for the NavigateToPose action
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Wait for the action server to be available
    while not action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info('Action server not available, waiting...')

    # Cancel the goal
    goal_handle = action_client.action_server.wait_for_goal_handle_async()
    rclpy.spin_until_future_complete(node, goal_handle)
    if goal_handle.result():
        future = goal_handle.result().cancel_goal_async()
        rclpy.spin_until_future_complete(node, future)
        if future.result():
            node.get_logger().info('Goal canceled successfully!')
        else:
            node.get_logger().info('Failed to cancel goal.')
    else:
        node.get_logger().info('Failed to get goal handle.')

    # Cleanup
    action_client.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    cancel_goal()
