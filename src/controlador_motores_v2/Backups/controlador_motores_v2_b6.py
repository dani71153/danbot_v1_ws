#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_srvs.srv import SetBool
import serial
import time

class MotorController(Node):
    """
    Clase que controla los motores de un sistema a través de comandos ROS2 y comunica con el Arduino y ESP32
    para enviar comandos de velocidad y leer los valores de los encoders. Además, permite detener el nodo mediante
    un servicio ROS2.
    """

    def __init__(self):
        super().__init__('motor_controller')
        # Suscripción al tópico 'motor_commands' para recibir comandos de velocidad para los motores
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'motor_commands',
            self.listener_callback,
            10
        )
        self.subscription  # evitar advertencias de variable no utilizada

        # Servicio para detener el nodo
        self.srv = self.create_service(SetBool, 'stop_motor_controller', self.stop_callback)

        # Configuración de los puertos seriales
        self.arduino_port = '/dev/ttyUSB0'  # Ajustar según sea necesario
        self.esp32_port = '/dev/ttyUSB1'    # Ajustar según sea necesario
        self.baud_rate = 115200             # Ajustar según sea necesario

        # Límites de velocidad
        self.max_speed = 0.65
        self.min_speed = -0.65

        # Factor de corrección para los encoders (ajustar según la relación de engranajes)
        self.arduino_correction_factor = 1.0  # Ajustar según sea necesario para el Arduino
        self.esp32_correction_factor = 1.83    # Ajustar según sea necesario para el ESP32

        # Inicializar la comunicación serial para Arduino
        try:
            self.arduino_ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Arduino serial port {self.arduino_port} opened successfully')
            # Enviar comando para resetear los encoders al iniciar el programa
            self.arduino_ser.write(b'r\n')
            self.get_logger().info('Sent reset command to Arduino')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open Arduino serial port {self.arduino_port}: {e}')
            self.arduino_ser = None

        # Inicializar la comunicación serial para ESP32
        try:
            self.esp32_ser = serial.Serial(self.esp32_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'ESP32 serial port {self.esp32_port} opened successfully')
            # Enviar comando para resetear los encoders al iniciar el programa
            self.esp32_ser.write(b'r\n')
            self.get_logger().info('Sent reset command to ESP32')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open ESP32 serial port {self.esp32_port}: {e}')
            self.esp32_ser = None

        # Temporizador para leer los encoders periódicamente (cada 1 segundo)
        self.create_timer(0.5, self.read_encoders)

        # Publicador para los valores de los encoders
        self.encoder_publisher = self.create_publisher(Float32MultiArray, 'encoder_values', 10)

    def saturate_speed(self, speed):
        """Satura la velocidad para que esté dentro del rango permitido."""
        if speed > self.max_speed:
            return self.max_speed
        elif speed < self.min_speed:
            return self.min_speed
        return speed

    def listener_callback(self, msg):
        """
        Callback para recibir comandos de velocidad y enviarlos al Arduino y ESP32.
        Si no se envía ningún valor, se establecen todos los motores a 0.
        """
        # Si no se envía ningún valor de velocidad, establecer todos los motores a 0
        if len(msg.data) < 4:
            v1 = v2 = v3 = v4 = 0.0
        else:
            # Saturar los valores de velocidad
            v1 = self.saturate_speed(msg.data[0])
            v2 = self.saturate_speed(msg.data[1])
            v3 = self.saturate_speed(msg.data[2])
            v4 = self.saturate_speed(msg.data[3])

        # Enviar comandos al Arduino (motores v1 y v2)
        if self.arduino_ser is not None:
            try:
                command_arduino = f'm {v1} {v2}\n'
                self.arduino_ser.write(command_arduino.encode())
                self.get_logger().info(f'Sent to Arduino: {command_arduino.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send command to Arduino: {e}')

        # Enviar comandos al ESP32 (motores v3 y v4)
        if self.esp32_ser is not None:
            try:
                command_esp32 = f'm {v3} {v4}\n'
                self.esp32_ser.write(command_esp32.encode())
                self.get_logger().info(f'Sent to ESP32: {command_esp32.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send command to ESP32: {e}')

    def read_encoders(self):
        """
        Leer los valores de los encoders de los controladores y publicarlos en un tópico.
        """
        encoder_values = []

        # Leer encoders del Arduino
        if self.arduino_ser is not None:
            try:
                self.arduino_ser.write(b'e\n')
                arduino_response = self.arduino_ser.readline().decode().strip()
                if arduino_response:
                    try:
                        arduino_values = [float(val.replace(',', '.')) * self.arduino_correction_factor for val in arduino_response.split(',')]
                        encoder_values.extend(arduino_values)
                        self.get_logger().info(f'Raw Arduino response: {arduino_response}')
                    except ValueError as e:
                        self.get_logger().error(f'Failed to convert Arduino response to float: {e}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to read encoder values from Arduino: {e}')

        # Leer encoders del ESP32
        if self.esp32_ser is not None:
            try:
                self.esp32_ser.write(b'e\n')
                esp32_response = self.esp32_ser.readline().decode().strip()
                if esp32_response:
                    try:
                        esp32_values = [float(val.replace(',', '.')) * self.esp32_correction_factor for val in esp32_response.split(',')]
                        encoder_values.extend(esp32_values)
                        self.get_logger().info(f'Raw ESP32 response: {esp32_response}')
                    except ValueError as e:
                        self.get_logger().error(f'Failed to convert ESP32 response to float: {e}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to read encoder values from ESP32: {e}')

        # Publicar los valores de los encoders
        if encoder_values:
            msg = Float32MultiArray()
            msg.data = encoder_values
            self.encoder_publisher.publish(msg)

    def stop_callback(self, request, response):
        """
        Callback para el servicio que detiene el nodo.
        """
        if request.data:
            self.get_logger().info('Stop service called, shutting down...')
            response.success = True
            response.message = 'Motor controller stopped successfully'
            if rclpy.ok():
                rclpy.shutdown()  # Detener el nodo y finalizar el programa
        else:
            response.success = False
            response.message = 'Stop service not activated'
        return response

    def close_serial_ports(self):
        """
        Cerrar los puertos seriales si están abiertos.
        """
        if self.arduino_ser is not None:
            self.arduino_ser.close()
            self.get_logger().info('Closed Arduino serial port')
        if self.esp32_ser is not None:
            self.esp32_ser.close()
            self.get_logger().info('Closed ESP32 serial port')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass

    motor_controller.close_serial_ports()
    motor_controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
