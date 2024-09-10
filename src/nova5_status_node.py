import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PySide6.QtCore import Signal, QObject

class Nova5StatusNode(Node):
    status_update = Signal(str)

    def __init__(self):
        super().__init__('nova5_status_node')
        self.subscription_ = self.create_subscription(
            JointState,
            'nova5/status',
            self.listener_callback,
            10
        )
        self.get_logger().info("Nova5 Status Node has started.")

    def listener_callback(self, msg):
        joint_positions = ', '.join([f"{name}: {pos:.2f}" for name, pos in zip(msg.name, msg.position)])
        status_message = f"Joint Positions: {joint_positions}"
        self.get_logger().info(status_message)
        self.status_update.emit(status_message)
