""" Pablo Paramio
    This file in its current configuration allows the control of the drone in ofboard mode in the following way:
    Vehicle rates control

    TO DO:
    RESET INTEGRAL TERMS WHEN SWITCHING TO OFFBOARD MODE (Checking offboard mode flag). Topic is /fmu/out/vehicle_control_mode
"""



from numpy import NaN
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint, VehicleCommand, VehicleStatus, OffboardControlMode, TrajectorySetpoint, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math

class CmdVelToManualControl(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_manual_control')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joy_input = Joy()
        self.joy_input.axes = [0.0] * 8  # default 8 axes
        self.last_buttons = [0.0] * 13  # default 13 buttons
        self.vehicle_status = VehicleStatus()

        # Offboard control mode msg
        self.offboard_msg = OffboardControlMode()
        self.offboard_msg.position = False
        self.offboard_msg.velocity = False
        self.offboard_msg.acceleration = False
        self.offboard_msg.attitude = True
        self.offboard_msg.body_rate = False

        # VehicleRatesSetpoint msg
        self.attitude_sp_msg = VehicleAttitudeSetpoint()
        self.attitude_sp_msg.yaw_sp_move_rate = 0.0
        self.attitude_sp_msg.q_d = [0.0, 0.0, 0.0, 1.0] # Identity quaternion
        self.attitude_sp_msg.thrust_body = [0.0, 0.0, 0.0] 
        self.desired_roll = 0.0
        self.desired_pitch = 0.0

        self.px4_yaw = 0.0
        self.px4_roll = 0.0
        self.px4_pitch = 0.0

        # Suscribirse al topic /joy
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            px4_qos
        )
        # Suscribirse al topic de /fmu/out/vehicle_status
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            px4_qos
        )

        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            px4_qos
        )

        self.yaw = 0.0
        self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.odometry_callback, 
            px4_qos
        )
        
        # Publicador al topic VehicleCommand, utilizado para armar o activar modo offboard desde el script (no se hace en este caso)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            px4_qos)
        
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            px4_qos)
        
        self.attitude_sp_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            px4_qos)

        
        # Temporizador para publicar mensajes continuamente a 166 Hz (cada 0.006 segundos)
        self.timer = self.create_timer(0.006, self.publish_attitude_setpoint)
        self.get_logger().info("Nodo attitude_control_node iniciado.")

    def publish_vehicle_command(self, command, **params) -> None:
            """Publish a vehicle command."""
            msg = VehicleCommand()
            msg.command = command
            msg.param1 = params.get("param1", 0.0)
            msg.param2 = params.get("param2", 0.0)
            msg.param3 = params.get("param3", 0.0)
            msg.param4 = params.get("param4", 0.0)
            msg.param5 = params.get("param5", 0.0)
            msg.param6 = params.get("param6", 0.0)
            msg.param7 = params.get("param7", 0.0)
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def joy_callback(self, msg):
        """ Callback que actualiza el último mensaje recibido de /joy """
        self.joy_input = msg

        if self.joy_input.buttons[9] == 1 and self.last_buttons[9] == 0:  # Botón 10 para armar
            if(self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                self.arm()
            elif(self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.disarm()
        
        if self.joy_input.buttons[1] == 1 and self.last_buttons[1] == 0:  # Circulo para offboard mode
            self.engage_offboard_mode()

        self.last_buttons = self.joy_input.buttons

    def vehicle_status_callback(self, msg):
        """ Callback que actualiza el estado del vehículo """
        self.vehicle_status = msg

    def odometry_callback(self, msg):
        # PX4 quaternions are in (w, x, y, z)
        w, x, y, z = msg.q

        # Extract yaw (world heading)
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def vehicle_attitude_callback(self, msg):
        self.q_px4 = msg.q
        self.px4_roll, self.px4_pitch, self.px4_yaw = euler_from_quaternion([self.q_px4[1], self.q_px4[2], self.q_px4[3], self.q_px4[0]])
        self.get_logger().info(f"Yaw: {self.px4_yaw}")
    def publish_attitude_setpoint(self):
        # Always publish offboard control mode (optional low-rate)
        self.offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(self.offboard_msg)
        

        # Map joystick to attitude setpoint (NED)
        if self.joy_input is not None:
            roll_scale = 0.5  # rad/s per joystick unit
            pitch_scale = 0.5
            yaw_scale = 2.0
            thrust_scale = 1.0  # 0 to 1 range

            self.desired_roll_angle = - self.joy_input.axes[3] * roll_scale    # roll
            self.desired_pitch_angle = - self.joy_input.axes[4] * pitch_scale   # pitch
            self.desired_yaw_angle = - self.joy_input.axes[0] * yaw_scale + self.px4_yaw    # yaw

            


            q = quaternion_from_euler(self.desired_roll_angle, self.desired_pitch_angle, self.desired_yaw_angle)
            q_px4 = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
            self.attitude_sp_msg.q_d = q_px4
            # throttle (-1 is max throttle, 0 is no throttle)
            thrust = - self.joy_input.axes[1] * thrust_scale
            # clip thrust to [-1.0, 0.0]
            thrust = max(-1.0, min(0.0, thrust))
            self.attitude_sp_msg.thrust_body[2] = thrust

        # Publish attitude setpoint
        self.attitude_sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_sp_pub.publish(self.attitude_sp_msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToManualControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
