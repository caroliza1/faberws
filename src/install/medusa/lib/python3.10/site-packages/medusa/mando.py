import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import sys
import tty
import termios
import select


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_state_sim')
        
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Movimiento principal
        self.min_angle = -3.142
        self.max_angle =  3.142
        self.current_angle = 0.0
        self.step_motor = 0.01   # movimiento por tecla

        # PAN / TILT
        self.pan = 0.0
        self.tilt = 0.0
        self.step_pan_tilt = 0.02

        # --- Interpolación para des1_joint y des2_joint ---
        self.key_points_des = [
            (-3.142, 0.07,   0.06),
            (-1.57,  0.0444, 0.0216),
            (0.0,    0.0,    0.0),
            (1.57,   0.0444, 0.0216),
            (3.142,  0.07,   0.06)
        ]

        # --- Interpolación para var1_joint y vari1_joint ---
        self.key_points_var = [
            (-3.142, -0.093, -0.093),
            (-1.57, -0.246, -0.246),
            (0.0,   0.0, 0.0),
            (1.57, -0.246, -0.246),
            (3.142, -0.093, -0.093)
        ]

        # --- Interpolación para trans1_joint y trans2_joint ---
        self.key_points_trans_pos = [
            (0.0,    0.0,    0.0),
            (1.57,  -2.15,  -1.05),
            (3.142, -3.142, -3.142)
        ]

        self.key_points_trans_neg = [
            (-3.142, 3.142, 3.142),
            (-1.57,  2.15,  1.05),
            (0.0,     0.0,  0.0)
        ]

        # Configurar teclado
        self.old_settings = termios.tcgetattr(sys.stdin)

        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Nodo joint_state_sim con CONTROL DE TECLADO iniciado")
        self.get_logger().info("Controles: W/S = Tilt, A/D = Pan, Q/E = Motor joint")


    # ---------- Lectura NO BLOQUEANTE del teclado ----------
    def read_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


    # -------------------- FUNCIÓN INTERPOLACIÓN BASE --------------------
    def interpolate_generic(self, angle, table):
        if angle <= table[0][0]:
            return table[0][1], table[0][2]
        if angle >= table[-1][0]:
            return table[-1][1], table[-1][2]

        for i in range(len(table) - 1):
            a0, v1_0, v2_0 = table[i]
            a1, v1_1, v2_1 = table[i + 1]

            if a0 <= angle <= a1:
                t = (angle - a0) / (a1 - a0)
                v1 = v1_0 + (v1_1 - v1_0) * t
                v2 = v2_0 + (v2_1 - v2_0) * t
                return v1, v2

        return 0.0, 0.0

    def interpolate_des(self, angle):
        return self.interpolate_generic(angle, self.key_points_des)

    def interpolate_var(self, angle):
        return self.interpolate_generic(angle, self.key_points_var)

    def interpolate_trans(self, angle):
        if angle >= 0:
            return self.interpolate_generic(angle, self.key_points_trans_pos)
        else:
            return self.interpolate_generic(angle, self.key_points_trans_neg)


    # -------------------- CALLBACK --------------------
    def timer_callback(self):

        # ---- LEER TECLADO ----
        key = self.read_key()
        if key:

            # TILT (W / S)
            if key == 'w':
                self.tilt += self.step_pan_tilt
            elif key == 's':
                self.tilt -= self.step_pan_tilt

            # PAN (A / D)
            elif key == 'a':
                self.pan -= self.step_pan_tilt
            elif key == 'd':
                self.pan += self.step_pan_tilt

            # MOTOR (Q / E) con WRAP
            elif key == 'q':
                self.current_angle += self.step_motor
                if self.current_angle > self.max_angle:
                    self.current_angle = self.min_angle

            elif key == 'e':
                self.current_angle -= self.step_motor
                if self.current_angle < self.min_angle:
                    self.current_angle = self.max_angle

        # --- Obtener valores interpolados ---
        des1, des2 = self.interpolate_des(self.current_angle)
        var1, vari1 = self.interpolate_var(self.current_angle)
        trans1, trans2 = self.interpolate_trans(self.current_angle)

        # --- Publicar ---
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'motor_joint',
            'des1_joint', 'des2_joint',
            'var1_joint', 'vari1_joint',
            'trans1_joint', 'trans2_joint',
            'pan_joint', 'tilt_joint'
        ]

        msg.position = [
            self.current_angle,
            des1, des2,
            var1, vari1,
            trans1, trans2,
            self.pan, self.tilt
        ]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

