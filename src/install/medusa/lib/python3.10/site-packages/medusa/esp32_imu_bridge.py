#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header

import socket
import struct
import threading
import math

# ===================== CRC-8 Dallas/Maxim =====================
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 0x01) else (crc >> 1)
    return crc & 0xFF


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    Convierte RPY (rad) a quaternion (x,y,z,w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class Esp32ImuBridge(Node):
    def __init__(self):
        super().__init__('esp32_imu_bridge')

        p = self.declare_parameter
        # CAMBIO: usamos 9050 como puerto por defecto para evitar conflicto con otros nodos
        self.udp_port = p('udp_port', 9050).value
        self.frame_id = p('frame_id', 'medusa/imu_link').value

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_mag = self.create_publisher(Vector3Stamped, 'imu/mag', 10)
        self.pub_pos = self.create_publisher(Vector3Stamped, 'imu/position', 10)
        self.pub_vel = self.create_publisher(Vector3Stamped, 'imu/velocity', 10)
        self.pub_rpy = self.create_publisher(Vector3Stamped, 'imu/rpy', 10)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Permitir reutilizar el puerto si el proceso anterior no lo soltó bien
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.udp_port))
        self.get_logger().info(f"[ESP32 IMU] Escuchando UDP en puerto {self.udp_port}")

        # Thread de recepción
        self.running = True
        self.rx_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.rx_thread.start()

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()

    def recv_loop(self):
        # struct ImuPacket: <4B18fB  => 4 bytes header, 18 floats, 1 byte crc
        PACKET_SIZE = 4 + 18 * 4 + 1  # = 81
        fmt = '<4B18fB'

        while self.running:
            try:
                data, addr = self.sock.recvfrom(256)
            except OSError:
                break

            if len(data) != PACKET_SIZE:
                continue

            try:
                (start1, start2, seq, reserved,
                 ax, ay, az,
                 gx, gy, gz,
                 mx, my, mz,
                 px, py, pz,
                 vx, vy, vz,
                 roll, pitch, yaw,
                 crc_rx) = struct.unpack(fmt, data)
            except struct.error:
                continue

            if start1 != 0xAA or start2 != 0x55:
                continue

            crc_calc = crc8_maxim(data[2:-1])  # desde seq hasta último float
            if crc_calc != crc_rx:
                self.get_logger().warn(
                    f"CRC inválido (seq={seq}) calc=0x{crc_calc:02X} rx=0x{crc_rx:02X}"
                )
                continue

            now = self.get_clock().now().to_msg()

            # ===================== IMU =====================
            msg_imu = Imu()
            msg_imu.header = Header()
            msg_imu.header.stamp = now
            msg_imu.header.frame_id = self.frame_id

            # aceleración lineal
            msg_imu.linear_acceleration.x = ax
            msg_imu.linear_acceleration.y = ay
            msg_imu.linear_acceleration.z = az

            # gyro (rad/s)
            msg_imu.angular_velocity.x = gx
            msg_imu.angular_velocity.y = gy
            msg_imu.angular_velocity.z = gz

            # orientación a partir de roll/pitch/yaw
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            msg_imu.orientation.x = qx
            msg_imu.orientation.y = qy
            msg_imu.orientation.z = qz
            msg_imu.orientation.w = qw

            # ===================== Magnetómetro =====================
            msg_mag = Vector3Stamped()
            msg_mag.header.stamp = now
            msg_mag.header.frame_id = self.frame_id
            msg_mag.vector.x = mx
            msg_mag.vector.y = my
            msg_mag.vector.z = mz

            # ===================== Posición =====================
            msg_pos = Vector3Stamped()
            msg_pos.header.stamp = now
            msg_pos.header.frame_id = self.frame_id  # o "map"/"odom"
            msg_pos.vector.x = px
            msg_pos.vector.y = py
            msg_pos.vector.z = pz

            # ===================== Velocidad =====================
            msg_vel = Vector3Stamped()
            msg_vel.header.stamp = now
            msg_vel.header.frame_id = self.frame_id
            msg_vel.vector.x = vx
            msg_vel.vector.y = vy
            msg_vel.vector.z = vz

            # ===================== RPY =====================
            msg_rpy = Vector3Stamped()
            msg_rpy.header.stamp = now
            msg_rpy.header.frame_id = self.frame_id
            msg_rpy.vector.x = roll
            msg_rpy.vector.y = pitch
            msg_rpy.vector.z = yaw

            # Publicar
            self.pub_imu.publish(msg_imu)
            self.pub_mag.publish(msg_mag)
            self.pub_pos.publish(msg_pos)
            self.pub_vel.publish(msg_vel)
            self.pub_rpy.publish(msg_rpy)

            # Debug opcional
            # self.get_logger().info(
            #     f"[RX seq={seq}] px={px:.2f} py={py:.2f} pz={pz:.2f}"
            # )


def main(args=None):
    rclpy.init(args=args)
    node = Esp32ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

