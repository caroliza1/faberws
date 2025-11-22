import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
        self.step = 0.01

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
        # Rango positivo
        self.key_points_trans_pos = [
            (0.0,    0.0,    0.0),
            (1.57,  -2.15,  -1.05),
            (3.142, -3.142, -3.142)
        ]

        # Rango negativo (invertido)
        self.key_points_trans_neg = [
            (-3.142, 3.142, 3.142),
            (-1.57,  2.15,  1.05),
            (0.0,     0.0,    0.0)
        ]

        self.get_logger().info("Nodo joint_state_sim COMPLETO inicializado")


    # -------------------- FUNCIÓN INTERPOLACIÓN BASE --------------------
    def interpolate_generic(self, angle, table):
        """Función genérica para interpolación lineal."""
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


    # -------------------- des1 / des2 --------------------
    def interpolate_des(self, angle):
        return self.interpolate_generic(angle, self.key_points_des)

    # -------------------- var1 / vari1 --------------------
    def interpolate_var(self, angle):
        return self.interpolate_generic(angle, self.key_points_var)

    # -------------------- trans1 / trans2 --------------------
    def interpolate_trans(self, angle):
        if angle >= 0:
            return self.interpolate_generic(angle, self.key_points_trans_pos)
        else:
            return self.interpolate_generic(angle, self.key_points_trans_neg)


    # -------------------- CALLBACK --------------------
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Obtener valores interpolados
        des1, des2 = self.interpolate_des(self.current_angle)
        var1, vari1 = self.interpolate_var(self.current_angle)
        trans1, trans2 = self.interpolate_trans(self.current_angle)

        # PUBLICAR TODOS LOS JOINTS
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
            0.0,0.0
        ]

        self.publisher_.publish(msg)

        # Actualizar motor_joint
        self.current_angle += self.step
        if self.current_angle > self.max_angle:
            self.current_angle = self.min_angle


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

