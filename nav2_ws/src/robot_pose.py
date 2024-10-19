import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("motion_minimal_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "current_pose", 10)

        timer_period = 0.016  # seconds
        self.create_timer(timer_period, self.timer_callback)

        # Initialize a TransformListener and Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            msg.pose.position.x = trans.transform.translation.x
            msg.pose.position.y = trans.transform.translation.y
            msg.pose.orientation = trans.transform.rotation  # Directly use the quaternion

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published pose: x={msg.pose.position.x}, y={msg.pose.position.y}")

        except TransformException as e:
            self.get_logger().warn(f"Transform error: {e}")

def main(args=None):
    rclpy.init(args=args)
    motion_minimal_publisher = MinimalPublisher()
    rclpy.spin(motion_minimal_publisher)
    motion_minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
