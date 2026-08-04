import sys

import rclpy
from PyQt5.QtWidgets import QApplication

from yeyu_gui.main_window import MainWindow
from yeyu_gui.ros_bridge import DashboardRosNode, RosSpinThread


def main(args=None):
    rclpy.init(args=args)
    node = DashboardRosNode()

    spin_thread = RosSpinThread(node)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        spin_thread.stop()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
