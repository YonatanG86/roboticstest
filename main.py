import sys
from pathlib import Path
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide6.QtCore import Signal, QObject
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine


class RosPublisherNode(Node):
    def __init__(self):
        super().__init__('qt_ros_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from ROS2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


class RosWorker(QObject):
    new_message = Signal(str)

    def __init__(self):
        super().__init__()
        self.node = RosPublisherNode()

    def start(self):
        rclpy.spin(self.node)

    def stop(self):
        rclpy.shutdown()


if __name__ == "__main__":
    # Initialize ROS 2
    rclpy.init(args=None)

    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    # Set up the ROS worker
    ros_worker = RosWorker()

    # Start the ROS node in a separate thread
    ros_thread = Thread(target=ros_worker.start)
    ros_thread.start()

    # Load the QML file
    qml_file = Path(__file__).resolve().parent / "main.qml"
    engine.load(qml_file)

    if not engine.rootObjects():
        sys.exit(-1)

    # Connect the ROS worker's signal to a slot in the QML
    root_object = engine.rootObjects()[0]
    ros_worker.new_message.connect(root_object.updateMessage)

    try:
        sys.exit(app.exec())
    finally:
        ros_worker.stop()
        ros_thread.join()
