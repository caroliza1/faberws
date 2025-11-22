#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class MPU6050SerialNode(Node):
    def __init__(self):
        super().__init__('mpu6050_serial_node')

        # --- AJUSTA ESTOS VALORES ---
        self.serial_port = "/dev/ttyUSB0"
        self.baud_rate = 115200
        # ----------------------------

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Conectado al puerto serial: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Error abriendo el puerto serial: {e}")
            exit(1)

        self.publisher_ = self.create_publisher(Float32MultiArray, '/imu/data', 10)

        # Lee 50 veces por segundo
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            # Acepta valores separados por espacio o coma
            parts = line.replace(",", " ").split()

            if len(parts) != 9:
                self.get_logger().warn(f"Datos inválidos: {line}")
                return

            values = [float(x) for x in parts]

            msg = Float32MultiArray()
            msg.data = values

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error leyendo serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

