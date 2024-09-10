import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Nova5ControlNode(Node):
    def __init__(self):
        super().__init__('nova5_control_node')
        self.publisher_ = self.create_publisher(Twist, 'nova5/control', 10)
        self.get_logger().info("Nova5 Control Node has started.")

    def send_control_command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: Linear[{linear_x}, {linear_y}, {linear_z}], Angular[{angular_x}, {angular_y}, {angular_z}]")
