""" Pablo Paramio
    This file in its current configuration allows the controlof the drone in ofboard mode in the following way:
    Horizontal control done with acceleration setpoints (pitch and roll joystick axes)
    Vertical control done with velocity setpoints (throttle joystick axis)
    For this to work, a line in the offboardCheck.cpp file of PX4 must be modified to allow velocity control without an estimate (line 57)
    This was made for px4 v1.16.0
"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from px4_msgs.msg import ManualControlSetpoint, VehicleCommand, VehicleStatus, VehicleRatesSetpoint, OffboardControlMode, TrajectorySetpoint, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
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
        self.offboard_msg.velocity = True
        self.offboard_msg.acceleration = True
        self.offboard_msg.attitude = False
        self.offboard_msg.body_rate = False

        # Trajectory setpoint
        self.traj_msg = TrajectorySetpoint()
        self.traj_msg.velocity = [math.nan, math.nan, math.nan]  # start as uncontrolled
        self.traj_msg.acceleration = [math.nan, math.nan, math.nan]
        self.traj_msg.jerk = [math.nan, math.nan, math.nan]
        self.traj_msg.position = [math.nan, math.nan, math.nan]
        self.traj_msg.yaw = math.nan
        self.traj_msg.yawspeed = 0.0
        self.setpoint_publish = False


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


        self.yaw = 0.0
        self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.odometry_callback, 
            px4_qos
        )
        
        
        # Publicador al topic ManualControlSetpoint
        self.manual_control_pub = self.create_publisher(
            ManualControlSetpoint,
            '/fmu/in/manual_control_input', # input para sim (v1.16.0), setpoint para dron (1.14.4)
            px4_qos
        )
        
        # Publicador al topic VehicleRatesSetpoint
        self.vehicle_rates_pub = self.create_publisher(
            VehicleRatesSetpoint,
            '/fmu/in/vehicle_rates_setpoint',
            px4_qos
        )
        
        # Publicador al topic VehicleCommand, utilizado para armar
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            px4_qos)
        
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            px4_qos)
        
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            px4_qos)

        
        # Temporizador para publicar mensajes continuamente a 166 Hz (cada 0.006 segundos)
        # self.timer = self.create_timer(0.006, self.publish_manual_control)
        self.timer = self.create_timer(0.006, self.publish_acceleration_setpoint)
        # self.timer = self.create_timer(0.05, self.publish_setpoints)
        self.get_logger().info("Nodo cmd_vel_to_manual_control iniciado.")

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

    def takeoff(self):
        """Send a takeoff command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=10.0)
        self.get_logger().info('Takeoff command sent')

    def engage_stabilized_mode(self):
        """Switch to stabilized mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=7.0)
        self.get_logger().info("Switching to stabilized mode")

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

        # if self.joy_input.buttons[8] == 1 and self.last_buttons[8] == 0:  # Boton share para takeoff
            # self.takeoff()
        
        # if self.joy_input.buttons[3] == 1 and self.last_buttons[3] == 0:  # Cuadrado para stabilized mode
            # self.engage_stabilized_mode()

        
        # if self.joy_input.buttons[1] == 1 and self.last_buttons[1] == 0:  # Circulo para offboard mode
            # self.engage_offboard_mode()

        # if self.joy_input.buttons[0] == 1 and self.last_buttons[0] == 0:  # X para publicar / no publicar comandos
            # self.setpoint_publish = not self.setpoint_publish
        
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


    def publish_acceleration_setpoint(self):
        # Always publish offboard control mode (optional low-rate)
        self.offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(self.offboard_msg)

        # Map joystick to acceleration array
        if self.joy_input is not None:
            ax_scale = 2.0   # m/s² per joysti                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      ck unit
            ay_scale = 2.0
            az_scale = 2.0
            vz_scale = 1.0
            yaw_scale = 3.0  # rad/s
            self.traj_msg.velocity = [math.nan, math.nan, -self.joy_input.axes[1] * vz_scale]  # uncontrolled
            
            # XY joystick accelerations in body-heading frame
            ax_body = self.joy_input.axes[4] * ax_scale      # forward/back
            ay_body = -self.joy_input.axes[3] * ay_scale     # right/left

            # Rotate into world frame (yaw only)
            cy = math.cos(self.yaw)
            sy = math.sin(self.yaw)

            ax_world =  ax_body * cy - ay_body * sy
            ay_world =  ax_body * sy + ay_body * cy

                        
            # Pitch -> North / X
            self.traj_msg.acceleration[0] = ax_world
            # Roll -> East / Y
            self.traj_msg.acceleration[1] = ay_world
            # Throttle -> Down / Z (NED frame: positive down)
            # self.traj_msg.acceleration[2] = -self.joy_input.axes[1] * az_scale
            # Yaw axis -> yawspeed
            self.traj_msg.yawspeed = -self.joy_input.axes[0] * yaw_scale
            self.traj_msg.yaw = math.nan

        # Publish acceleration setpoint
        # if self.setpoint_publish:
        self.traj_pub.publish(self.traj_msg)

    def publish_velocity_setpoint(self):
        # Always publish offboard control mode (optional low-rate)
        self.offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(self.offboard_msg)

        # Map joystick to velocity array
        if self.joy_input is not None:
            vx_scale = 2.0   # m/s per joystick unit
            vy_scale = 2.0
            vz_scale = 1.0
            yaw_scale = 1.0  # rad/s

            # Pitch -> North / X
            self.traj_msg.velocity[0] = self.joy_input.axes[4] * vx_scale
            # Roll -> East / Y
            self.traj_msg.velocity[1] = -self.joy_input.axes[3] * vy_scale
            # Throttle -> Down / Z (NED frame: positive down)
            self.traj_msg.velocity[2] = -self.joy_input.axes[1] * vz_scale
            # Yaw axis -> yawspeed
            self.traj_msg.yawspeed = -self.joy_input.axes[0] * yaw_scale

        # Publish velocity setpoint
        self.traj_pub.publish(self.traj_msg)
    
    """
    def publish_rates_setpoint(self):

        # Publica el último mensaje recibido de /cmd_vel constantemente
        vehicle_rates_msg = VehicleRatesSetpoint()

        vehicle_rates_msg.pitch = self.joy_input.axes[4] * 3.14 # Pitch
        vehicle_rates_msg.roll = - self.joy_input.axes[3] * 3.14 # Roll
        vehicle_rates_msg.yaw = -self.joy_input.axes[0] * 3.14 # Yaw
        vehicle_rates_msg.thrust_body[0] = 0.0  # No se usa
        vehicle_rates_msg.thrust_body[1] = 0.0  # No se usa
        vehicle_rates_msg.thrust_body[2] = self.joy_input.axes[1]  # Throttle

        # Timestamp en microsegundos
        vehicle_rates_msg.timestamp = self.get_clock().now().nanoseconds // 1000  

        # Publicar el mensaje
        self.vehicle_rates_pub.publish(vehicle_rates_msg)
    """
    
    def publish_manual_control(self):

        # Publica el último mensaje recibido de /cmd_vel constantemente
        manual_control_msg = ManualControlSetpoint()

        manual_control_msg.pitch = self.joy_input.axes[4] # Pitch
        manual_control_msg.roll = - self.joy_input.axes[3]  # Roll
        manual_control_msg.yaw = -self.joy_input.axes[0]   # Yaw
        manual_control_msg.throttle = self.joy_input.axes[1] # Throttle
        
        if manual_control_msg.throttle < 0.0:
            manual_control_msg.throttle = 0.0
        manual_control_msg.throttle = manual_control_msg.throttle * 2 -1  # Map [0,1] to [-1,1]
        
        # Configuración necesaria para PX4
        manual_control_msg.data_source = 2  # MAVLink instance 1 para UXV como joystick
        manual_control_msg.valid = True  # Necesario para PX4
        manual_control_msg.sticks_moving = True 

        # Timestamp en microsegundos
        manual_control_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        manual_control_msg.timestamp_sample = manual_control_msg.timestamp

        # Publicar el mensaje
        self.manual_control_pub.publish(manual_control_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToManualControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
