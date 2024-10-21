import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros

class CinematicaDirecta(Node):
    def __init__(self):
        super().__init__('cinematica_directa')
        self.L = 0.125  # Distancia desde el centro del robot hasta la rueda (en metros)
        self.Rw = 0.03  # Radio de la rueda (en metros)

        # Suscripción a los valores de los encoders
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'encoder_values',
            self.encoder_callback,
            10
        )
        self.subscription  # evitar advertencias de variable no utilizada

        # Publicador para la odometría
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def encoder_callback(self, msg):
        # Leer las velocidades de las ruedas desde los valores de los encoders
        wheel_velocities = np.array(msg.data)
        if len(wheel_velocities) != 4:
            self.get_logger().error('Se esperaban 4 valores de velocidad de los encoders')
            return

        # Calcular la velocidad del robot
        vel_x, vel_y, vel_w = self.cinematica_directa(wheel_velocities)

        # Calcular la odometría
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # Actualizar la posición del robot
        self.x += vel_x * dt * math.cos(self.theta) - vel_y * dt * math.sin(self.theta)
        self.y += vel_x * dt * math.sin(self.theta) + vel_y * dt * math.cos(self.theta)
        self.theta += vel_w * dt

        # Crear y publicar el mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Posición
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quaternion = self.create_quaternion_from_yaw(self.theta)
        odom_msg.pose.pose.orientation = quaternion

        # Velocidad
        odom_msg.twist.twist.linear.x = vel_x
        odom_msg.twist.twist.linear.y = vel_y
        odom_msg.twist.twist.angular.z = vel_w

        # Publicar el mensaje de odometría
        self.odom_publisher.publish(odom_msg)

        # Publicar la transformación TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion

        self.tf_broadcaster.sendTransform(t)

    def cinematica_directa(self, wheel_velocities):
        """
        Calcula la velocidad lineal y angular del robot a partir de las velocidades de las ruedas.

        Parámetros:
        wheel_velocities: np.array
            Array de velocidades de las cuatro ruedas [v1, v2, v3, v4].

        Retorna:
        vel_x: float
            Velocidad lineal en el eje x.
        vel_y: float
            Velocidad lineal en el eje y.
        vel_w: float
            Velocidad angular alrededor del eje vertical.
        """
        # Matriz de transformación de velocidades de las ruedas a velocidades del robot
        T_inv = (self.Rw / 4) * np.array([
            [math.sin(math.pi / 4), math.sin(math.pi / 4 + math.pi / 2), math.sin(math.pi / 4 - math.pi), math.sin(math.pi / 4 - math.pi / 2)],
            [math.cos(math.pi / 4), math.cos(math.pi / 4 + math.pi / 2), math.cos(math.pi / 4 - math.pi), math.cos(math.pi / 4 - math.pi / 2)],
            [1 / self.L, 1 / self.L, 1 / self.L, 1 / self.L]
        ])

        # Calcula las velocidades del robot (vel_x, vel_y, vel_w)
        velocities = np.dot(T_inv, wheel_velocities)

        vel_x = velocities[0]
        vel_y = velocities[1]
        vel_w = velocities[2]

        return vel_x, vel_y, vel_w

    def create_quaternion_from_yaw(self, yaw):
        """
        Crea un cuaternión a partir de un ángulo yaw.

        Parámetros:
        yaw: float
            Ángulo de orientación en radianes.

        Retorna:
        Quaternion: geometry_msgs.msg.Quaternion
            Cuaternión correspondiente al ángulo yaw dado.
        """
        q = Quaternion()
        q.x, q.y, q.z, q.w = tf2_ros.transformations.quaternion_from_euler(0, 0, yaw)
        return q

# Ejemplo de uso con ROS2
def main(args=None):
    rclpy.init(args=args)
    cinematica_directa_node = CinematicaDirecta()

    try:
        rclpy.spin(cinematica_directa_node)
    except KeyboardInterrupt:
        pass

    cinematica_directa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
