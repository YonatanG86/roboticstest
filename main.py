import sys
from pathlib import Path
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide6.QtCore import Signal, QObject, Qt
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtWidgets import QApplication, QMainWindow, QTextEdit, QVBoxLayout, QWidget, QSplitter


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


class ConsoleWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)

        layout = QVBoxLayout()
        layout.addWidget(self.text_edit)
        self.setLayout(layout)

    def write(self, message):
        self.text_edit.append(message)

    def flush(self):
        pass


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS and QML Split View")
        self.setGeometry(100, 100, 1200, 800)

        # Create a QSplitter to split the window into two areas
        splitter = QSplitter(Qt.Horizontal)

        # Left side: Console window
        self.console_window = ConsoleWindow()

        # Right side: QML application view
        self.qml_widget = QWidget()
        self.qml_layout = QVBoxLayout()
        self.qml_widget.setLayout(self.qml_layout)

        splitter.addWidget(self.console_window)
        splitter.addWidget(self.qml_widget)

        # Set initial sizes of the two areas
        splitter.setSizes([400, 800])

        self.setCentralWidget(splitter)

    def set_qml_engine(self, engine):
        # Add QML view to the right side
        self.qml_layout.addWidget(engine)


if __name__ == "__main__":
    # Initialize ROS 2
    rclpy.init(args=None)

    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()

    # Redirect standard output and error to the console window
    sys.stdout = main_window.console_window
    sys.stderr = main_window.console_window

    # Set up the ROS worker
    ros_worker = RosWorker()

    # Start the ROS node in a separate thread
    ros_thread = Thread(target=ros_worker.start)
    ros_thread.start()

    # Load the QML file
    gui_app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    qml_file = Path(__file__).resolve().parent / "main.qml"
    engine.load(qml_file)

    if not engine.rootObjects():
        sys.exit(-1)

    # Connect the ROS worker's signal to a slot in the QML
    root_object = engine.rootObjects()[0]
    ros_worker.new_message.connect(root_object.updateMessage)

    # Add the QML engine view to the main window
    main_window.set_qml_engine(engine)

    try:
        sys.exit(gui_app.exec())
    finally:
        ros_worker.stop()
        ros_thread.join()
