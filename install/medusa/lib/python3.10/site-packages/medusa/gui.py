#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QGridLayout
from PyQt5.QtCore import QTimer


class IMUGUI(Node):
    def __init__(self):
        super().__init__('imu_gui')

        # Variables para guardar datos
        self.orientation = [0.0, 0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]
        self.linear_acc = [0.0, 0.0, 0.0]

        # Subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # ----- GUI -----
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Interfaz IMU - ROS2 Humble")

        layout = QGridLayout()

        # Etiquetas
        self.labels = {
            "ori_x": QLabel("Ori X: 0"),
            "ori_y": QLabel("Ori Y: 0"),
            "ori_z": QLabel("Ori Z: 0"),
            "ori_w": QLabel("Ori W: 0"),

            "ang_x": QLabel("Vel Ang X: 0"),
            "ang_y": QLabel("Vel Ang Y: 0"),
            "ang_z": QLabel("Vel Ang Z: 0"),

            "acc_x": QLabel("Acc Lin X: 0"),
            "acc_y": QLabel("Acc Lin Y: 0"),
            "acc_z": QLabel("Acc Lin Z: 0"),
        }

        row = 0
        for label in self.labels.values():
            layout.addWidget(label, row, 0)
            row += 1

        self.window.setLayout(layout)
        self.window.resize(290, 300)

        # Timer para actualizar la GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(50)  # 20 Hz

    def imu_callback(self, msg: Imu):
        self.orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        self.angular_vel = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

        self.linear_acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

    def update_gui(self):
        self.labels["ori_x"].setText(f"Ori X: {self.orientation[0]:.3f}")
        self.labels["ori_y"].setText(f"Ori Y: {self.orientation[1]:.3f}")
        self.labels["ori_z"].setText(f"Ori Z: {self.orientation[2]:.3f}")
        self.labels["ori_w"].setText(f"Ori W: {self.orientation[3]:.3f}")

        self.labels["ang_x"].setText(f"Vel Ang X: {self.angular_vel[0]:.3f}")
        self.labels["ang_y"].setText(f"Vel Ang Y: {self.angular_vel[1]:.3f}")
        self.labels["ang_z"].setText(f"Vel Ang Z: {self.angular_vel[2]:.3f}")

        self.labels["acc_x"].setText(f"Acc Lin X: {self.linear_acc[0]:.3f}")
        self.labels["acc_y"].setText(f"Acc Lin Y: {self.linear_acc[1]:.3f}")
        self.labels["acc_z"].setText(f"Acc Lin Z: {self.linear_acc[2]:.3f}")

    def start(self):
        self.window.show()
        sys.exit(self.app.exec_())


def main(args=None):
    rclpy.init(args=args)
    gui_node = IMUGUI()
    gui_node.start()
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

