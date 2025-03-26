#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from std_msgs.msg import Float64

class LawnMowerSurvey(Node):
    def __init__(self):
        super().__init__('lawn_mower_survey')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.start_time = time.time()

        # Lawn mower pattern parameters
        self.INITIAL_HEIGHT = 20.0  # meters
        self.SURVEY_LENGTH = 300.0  # meters
        self.SURVEY_WIDTH = 200.0    # meters
        self.STRIP_SPACING = 50.0   # meters
        self.MIN_HEIGHT = 5.0       # meters
        self.SPEED = 5.0            # m/s
        self.current_waypoint = 0

        # Generate waypoints
        self.waypoints = self.generate_lawn_mower_waypoints()

        # State machine
        self.state = "TAKEOFF"  # States: TAKEOFF, SURVEY, LAND

        self.create_timer(0.1, self.timer_callback)  # 10Hz control loop

    def generate_lawn_mower_waypoints(self):
        waypoints = []
        half_length = self.SURVEY_LENGTH / 2.0
        half_width = self.SURVEY_WIDTH / 2.0

        # Create a list of x positions from -half_length to half_length using STRIP_SPACING
        x_positions = []
        x = -half_length
        while x <= half_length:
            x_positions.append(x)
            x += self.STRIP_SPACING

        # Ensure that the center (x = 0) is included
        if 0.0 not in x_positions:
            x_positions.append(0.0)
            x_positions.sort()

        # Generate the vertical (y) passes for each x position
        for i, x in enumerate(x_positions):
            if i % 2 == 0:
                # Even lane: go from -half_width (bottom) to half_width (top)
                waypoints.append((x, -half_width, -self.INITIAL_HEIGHT, 0.0))
                waypoints.append((x, half_width, -self.INITIAL_HEIGHT, 0.0))
            else:
                # Odd lane: reverse direction (top to bottom)
                waypoints.append((x, half_width, -self.INITIAL_HEIGHT, 0.0))
                waypoints.append((x, -half_width, -self.INITIAL_HEIGHT, 0.0))
        
        return waypoints
    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_at_target_position(self, target):
        current_pos = self.vehicle_odometry.position
        distance = math.sqrt((current_pos[0] - target[0]) ** 2 + (current_pos[1] - target[1]) ** 2 + (current_pos[2] - target[2]) ** 2)
        return distance < 1.0

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.start_time = time.time()

        self.publish_offboard_control_mode()

        if self.state == "TAKEOFF":
            self.publish_trajectory_setpoint(x=0.0, y=0.0, z=-self.INITIAL_HEIGHT, yaw=0.0)
            if self.is_at_target_position((0.0, 0.0, -self.INITIAL_HEIGHT)):
                self.state = "SURVEY"
                self.get_logger().info("Starting lawn mower survey pattern")

        elif self.state == "SURVEY":
            target = self.waypoints[self.current_waypoint]
            self.publish_trajectory_setpoint(*target)

            if self.is_at_target_position(target):
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.waypoints):
                    self.state = "LAND"
                    self.get_logger().info("Survey complete, preparing to land")

        elif self.state == "LAND":
            self.publish_trajectory_setpoint(x=0.0, y=0.0, z=0.0, yaw=0.0)
            self.get_logger().info("Landing...")

        self.offboard_setpoint_counter += 1


def main():
    print('Starting lawn mower survey...')
    rclpy.init()
    node = LawnMowerSurvey()
    print('Lawn mower survey node created. Starting control loop...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Stopping lawn mower survey...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

