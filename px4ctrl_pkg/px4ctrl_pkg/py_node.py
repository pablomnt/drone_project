import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import ManualControlSetpoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

class CmdVelToManualControl(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_manual_control')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Último mensaje de velocidad recibido y última velocidad válida
        self.last_cmd_vel = Twist()
        self.last_valid_cmd_vel = Twist()  
        self.last_received_time = self.get_clock().now()

        # Suscribirse al topic /cmd_vel_smoothed
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            self.cmd_vel_callback,
            10
        )

        # Publicador al topic ManualControlSetpoint
        self.manual_control_pub = self.create_publisher(
            ManualControlSetpoint,
            '/fmu/in/manual_control_input',
            px4_qos
        )

        # Temporizador para publicar mensajes continuamente a 166 Hz (cada 0.006 segundos)
        self.timer = self.create_timer(0.006, self.publish_manual_control)

        # Temporizador para verificar si han dejado de publicarse mensajes
        self.timeout_threshold = 1.2  # Segundos sin recibir mensajes antes de detener
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("Nodo cmd_vel_to_manual_control iniciado.")

    def cmd_vel_callback(self, msg):
        """ Callback que actualiza el último mensaje recibido de /cmd_vel """
        self.last_cmd_vel = msg
        self.last_received_time = self.get_clock().now()

        # Solo actualizar la última velocidad válida si hay algún valor distinto de 0.0
        if msg.linear.x > 0.1 or msg.angular.z > 0.1:
            self.last_valid_cmd_vel = msg  # Guardamos la última instrucción útil

    def publish_manual_control(self):
        """ Publica el último mensaje recibido de /cmd_vel constantemente """

        # Si ambos valores son 0.0, usar la última velocidad válida
        if self.last_cmd_vel.linear.x == 0.0 and self.last_cmd_vel.angular.z == 0.0:
            cmd_vel = self.last_valid_cmd_vel
        else:
            cmd_vel = self.last_cmd_vel  # Usar el mensaje actual

        manual_control_msg = ManualControlSetpoint()

        # Mapear velocidades a PX4
        manual_control_msg.throttle = min(0.5 * float(cmd_vel.linear.x), 999999.0)
        manual_control_msg.roll = -1.0 * max(min(cmd_vel.angular.z / 40.0, 0.75), -0.75)

        # Configuración necesaria para PX4
        manual_control_msg.data_source = 3  # MAVLink instance 1 para UXV como joystick
        manual_control_msg.valid = True  # Necesario para PX4

        # Timestamp en microsegundos
        manual_control_msg.timestamp = self.get_clock().now().nanoseconds // 1000  

        # Publicar el mensaje
        self.manual_control_pub.publish(manual_control_msg)

    def check_timeout(self):
        """ Detiene el robot si no se han recibido mensajes en un tiempo determinado """
        now = self.get_clock().now()
        if (now - self.last_received_time).nanoseconds / 1e9 > self.timeout_threshold:
          #  self.get_logger().info("Tiempo de inactividad superado. Deteniendo robot.")
            self.last_cmd_vel = Twist()  # Forzar parada
            self.last_valid_cmd_vel = Twist()  # Limpiar última velocidad válida

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToManualControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
