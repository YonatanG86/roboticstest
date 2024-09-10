import sys
from threading import Thread
import rclpy
from rclpy.node import Node
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from nova5_control_node import Nova5ControlNode
from nova5_status_node import Nova5StatusNode

class RosWorker:
    def __init__(self, node: Node):
        self.node = node

    def start(self):
        rclpy.spin(self.node)

    def stop(self):
        rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init(args=None)

    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    control_node = Nova5ControlNode()
    status_node = Nova5StatusNode()

    ros_control_worker = RosWorker(control_node)
    ros_status_worker = RosWorker(status_node)

    control_thread = Thread(target=ros_control_worker.start)
    status_thread = Thread(target=ros_status_worker.start)

    control_thread.start()
    status_thread.start()

    qml_file = Path(__file__).resolve().parent / "gui.qml"
    engine.load(qml_file)

    if not engine.rootObjects():
        sys.exit(-1)

    root_object = engine.rootObjects()[0]
    status_node.status_update.connect(root_object.updateStatus)

    timer = QTimer()
    timer.timeout.connect(lambda: control_node.send_control_command(1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    timer.start(1000)

    try:
        sys.exit(app.exec())
    finally:
        ros_control_worker.stop()
        ros_status_worker.stop()
        control_thread.join()
        status_thread.join()
