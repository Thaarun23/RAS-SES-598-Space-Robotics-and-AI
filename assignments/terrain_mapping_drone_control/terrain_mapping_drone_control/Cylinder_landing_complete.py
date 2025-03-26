#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point

import cv2
import numpy as np
from cv_bridge import CvBridge
from transforms3d.euler import mat2euler

# Parameters for landing approach
DESIRED_OFFSET = 10.0    # Offset (meters) to position the drone away from the cylinder center
MIN_FLY_HEIGHT = 5.0     # The drone must be at least 5m above ground when approaching

class IntegratedSurveyLanding(Node):
    def __init__(self):
        super().__init__('integrated_survey_landing')

        # Initialize odometry and status
        self.vehicle_odometry = None
        self.vehicle_status = None

        # --- Publishers ---
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Debug image publishers
        self.mono_debug_pub = self.create_publisher(Image, '/integrated/mono_debug', 10)
        self.depth_debug_pub = self.create_publisher(Image, '/integrated/depth_debug', 10)

        # --- Subscribers ---
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                                              self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                                            self.vehicle_status_callback, qos_profile)
        self.mono_image_sub = self.create_subscription(Image, '/drone/down_mono', self.mono_image_callback, 10)
        # CameraInfo subscriber with VOLATILE durability (to match the publisher)
        camera_info_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.camera_info_sub = self.create_subscription(CameraInfo, '/drone/down_mono/camera_info',
                                                         self.camera_info_callback, camera_info_qos)
        # Static CameraInfo publisher in case no external publisher is present
        self.static_camera_info_pub = self.create_publisher(CameraInfo, '/drone/down_mono/camera_info', camera_info_qos)
        self.create_timer(1.0, self.publish_static_camera_info)

        self.depth_image_sub = self.create_subscription(Image, '/drone/front_depth', self.depth_image_callback, 10)

        # --- CV Bridge & ARUCO Setup ---
        self.bridge = CvBridge()
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None  # Using older API calls
            self.get_logger().info('Using older OpenCV ArUco API')
        except AttributeError:
            self.get_logger().warn('Could not configure ArUco - check OpenCV installation')
        self.marker_size = 0.8  # meters

        # --- Camera Calibration ---
        self.camera_matrix = np.array([[554.254691191187, 0.0, 320.5],
                                       [0.0, 554.254691191187, 240.5],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(5)
        self.calibration_received = False

        # --- Survey Parameters ---
        self.INITIAL_HEIGHT = 20.0      # meters
        self.SURVEY_LENGTH = 300.0      # meters (x-direction)
        self.SURVEY_WIDTH = 200.0       # meters (y-direction)
        self.STRIP_SPACING = 50.0       # meters
        self.MIN_HEIGHT = 5.0           # meters (for landing)
        self.waypoints = self.generate_lawn_mower_waypoints()
        self.current_waypoint = 0

        # --- State Machine ---
        # States: TAKEOFF, SURVEY, APPROACH, LAND
        self.state = "TAKEOFF"
        self.offboard_setpoint_counter = 0
        self.start_time = time.time()

        # --- Candidate Storage ---
        # We'll record candidate circles with ARUCO marker data.
        # Each candidate is a dict with:
        #   "odometry": (x, y) at detection time,
        #   "aruco_tvec": [x, y, z] from marker (if available),
        #   "circle_radius": radius (from circle detection),
        #   "height": cylinder height (from depth ellipse, e.g., ellipse height)
        self.detected_candidates = []
        # Temporary candidate from current cycle:
        self.latest_candidate = None

        # Timer for main control loop at 10Hz
        self.create_timer(0.1, self.control_loop)

    def publish_static_camera_info(self):
        cam_info = CameraInfo()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = "down_mono_camera"
        cam_info.k = self.camera_matrix.flatten().tolist()
        cam_info.d = self.dist_coeffs.flatten().tolist()
        self.static_camera_info_pub.publish(cam_info)
        self.get_logger().debug("Published static CameraInfo.")

    def generate_lawn_mower_waypoints(self):
        waypoints = []
        half_length = self.SURVEY_LENGTH / 2.0
        half_width = self.SURVEY_WIDTH / 2.0
        x_positions = []
        x = -half_length
        while x <= half_length:
            x_positions.append(x)
            x += self.STRIP_SPACING
        if 0.0 not in x_positions:
            x_positions.append(0.0)
            x_positions.sort()
        for i, x in enumerate(x_positions):
            if i % 2 == 0:
                waypoints.append((x, -half_width, -self.INITIAL_HEIGHT, 0.0))
                waypoints.append((x, half_width, -self.INITIAL_HEIGHT, 0.0))
            else:
                waypoints.append((x, half_width, -self.INITIAL_HEIGHT, 0.0))
                waypoints.append((x, -half_width, -self.INITIAL_HEIGHT, 0.0))
        return waypoints

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info('Camera calibration received successfully.')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Dist coeffs: {self.dist_coeffs}')
        except Exception as e:
            self.get_logger().error(f'Camera info error: {str(e)}')

    def mono_image_callback(self, msg):
        try:
            mono_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            display_img = cv2.cvtColor(mono_image, cv2.COLOR_GRAY2BGR)

            # First, perform ARUCO marker detection
            corners, ids, _ = cv2.aruco.detectMarkers(mono_image, self.aruco_dict, parameters=self.aruco_params)
            candidate = {}
            if ids is not None and self.vehicle_odometry is not None and hasattr(self.vehicle_odometry, 'position'):
                # Use the first detected marker as reference.
                i = 0
                marker_id = ids[i][0]  # note: ids is usually a column vector
                marker_points = np.array([
                    [-self.marker_size/2, self.marker_size/2, 0],
                    [ self.marker_size/2, self.marker_size/2, 0],
                    [ self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)
                objPoints = marker_points.reshape((4, 3))
                imgPoints = corners[i].reshape((4, 2))
                success, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
                if success:
                    candidate["aruco_tvec"] = tvec.flatten().tolist()
                    odom = self.vehicle_odometry.position
                    candidate["odometry"] = (odom[0], odom[1])
                    # Draw marker on display image.
                    pts = corners[i].reshape(-1, 2).astype(int)
                    cv2.polylines(display_img, [pts], True, (0,255,0), 2)
                    cv2.putText(display_img, f"ID:{marker_id}", tuple(pts[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            # Next, perform circle detection.
            circles = cv2.HoughCircles(mono_image, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                       param1=100, param2=30, minRadius=10, maxRadius=100)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                c = circles[0][0]  # take the first detected circle
                if candidate.get("odometry", None) is None and self.vehicle_odometry is not None and hasattr(self.vehicle_odometry, 'position'):
                    odom = self.vehicle_odometry.position
                    candidate["odometry"] = (odom[0], odom[1])
                candidate["circle_radius"] = c[2]
                self.get_logger().info(f"Circle detected at ({c[0]}, {c[1]}) with radius {c[2]:.2f}")
                cv2.circle(display_img, (c[0], c[1]), c[2], (255,0,0), 2)
                cv2.circle(display_img, (c[0], c[1]), 2, (255,0,0), -1)
                cv2.putText(display_img, "Circle", (c[0]-10, c[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
            # Save the candidate (if any) for this cycle.
            if candidate != {}:
                self.latest_candidate = candidate

            mono_debug_msg = self.bridge.cv2_to_imgmsg(display_img, encoding='bgr8')
            self.mono_debug_pub.publish(mono_debug_msg)
        except Exception as e:
            self.get_logger().error(f'Mono image processing error: {str(e)}')

    def depth_image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            debug_depth = depth_image.copy()
            debug_depth[np.isnan(debug_depth)] = 0
            depth_normalized = np.clip(debug_depth, 0.0, 10.0)
            depth_normalized = ((depth_normalized) * 255 / 10.0).astype(np.uint8)
            depth_normalized = cv2.equalizeHist(depth_normalized)
            blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            debug_img = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_cylinder = None
            best_confidence = 0.0
            cylinder_dims = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 1000:
                    continue
                if len(contour) >= 5:
                    ellipse = cv2.fitEllipse(contour)
                    (cx, cy), (width, height), angle = ellipse
                    aspect_ratio = min(width, height) / max(width, height)
                    angle_conf = 1.0 - abs(angle - 90) / 90.0
                    size_conf = min(width, height) / 100.0
                    confidence = aspect_ratio * angle_conf * size_conf
                    if aspect_ratio > 0.3 and abs(angle - 90) < 30:
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_cylinder = ellipse
                            cylinder_dims = [width, height, angle, confidence]
            if best_cylinder is not None:
                (cx, cy), _, _ = best_cylinder
                cx, cy = int(cx), int(cy)
                if 0 <= cx < depth_image.shape[1] and 0 <= cy < depth_image.shape[0]:
                    center_depth = depth_image[cy, cx]
                    cv2.ellipse(debug_img, best_cylinder, (0,255,0), 2)
                    cv2.circle(debug_img, (cx, cy), 3, (0,255,0), -1)
                    cv2.putText(debug_img, f"D:{center_depth:.2f}m", (cx+10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    # If we have a candidate from the mono callback, update its height measurement.
                    if self.latest_candidate is not None and self.latest_candidate.get("height") is None:
                        # Here we assume the ellipse's height is a proxy for the cylinder height.
                        self.latest_candidate["height"] = cylinder_dims[1]  # second element
                        self.get_logger().info(f"Updated candidate with height: {cylinder_dims[1]:.2f} m")
            depth_debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.depth_debug_pub.publish(depth_debug_msg)
        except Exception as e:
            self.get_logger().error(f'Depth image processing error: {str(e)}')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent.")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode engaged.")

    def is_at_target_position(self, target, threshold=1.0):
        try:
            if self.vehicle_odometry is None or not hasattr(self.vehicle_odometry, 'position'):
                return False
            current_pos = self.vehicle_odometry.position
            distance = math.sqrt((current_pos[0] - target[0])**2 +
                                 (current_pos[1] - target[1])**2 +
                                 (current_pos[2] - target[2])**2)
            self.get_logger().info(
                f"Checking target {target}: Current=({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}), Dist={distance:.2f}"
            )
            return distance < threshold
        except Exception as e:
            self.get_logger().warn(f"is_at_target_position exception: {e}")
            return False

    def control_loop(self):
        # Log state and drone position
        if self.vehicle_odometry is not None and hasattr(self.vehicle_odometry, 'position'):
            px, py, pz = self.vehicle_odometry.position
            self.get_logger().info(f"State={self.state}, Drone pos=({px:.2f}, {py:.2f}, {pz:.2f})")
        else:
            self.get_logger().info(f"State={self.state}, Waiting for odometry...")

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.start_time = time.time()

        self.publish_offboard_control_mode()

        if self.state == "TAKEOFF":
            if self.vehicle_odometry is not None and hasattr(self.vehicle_odometry, 'position'):
                self.publish_trajectory_setpoint(x=0.0, y=0.0, z=-self.INITIAL_HEIGHT, yaw=0.0)
                if self.is_at_target_position((0.0, 0.0, -self.INITIAL_HEIGHT), threshold=2.0):
                    self.state = "SURVEY"
                    self.get_logger().info("Takeoff complete; beginning lawn mower survey.")
            else:
                self.get_logger().info("Waiting for valid odometry...")

        elif self.state == "SURVEY":
            # If we got a candidate from the latest mono image, store it.
            if self.latest_candidate is not None:
                self.detected_candidates.append(self.latest_candidate)
                self.get_logger().info(f"Stored candidate: {self.latest_candidate}")
                self.latest_candidate = None

            # Follow lawn mower waypoints.
            if self.current_waypoint < len(self.waypoints):
                target = self.waypoints[self.current_waypoint]
                self.publish_trajectory_setpoint(*target)
                if self.is_at_target_position(target):
                    self.current_waypoint += 1
            else:
                self.get_logger().info("Survey complete.")
                # After survey, select the candidate with maximum height.
                if len(self.detected_candidates) > 0:
                    tallest = max(self.detected_candidates, key=lambda c: c["height"] if c["height"] is not None else 0)
                    self.get_logger().info(f"Selected candidate: {tallest}")
                    # Use ARUCO tvec if available; otherwise, use candidate odometry.
                    if "aruco_tvec" in tallest and tallest["aruco_tvec"] is not None:
                        rel_x, rel_y = tallest["aruco_tvec"][:2]
                    else:
                        rel_x, rel_y = 0.0, 0.0
                    # Compute landing target:
                    # Here we simply add DESIRED_OFFSET to the candidate's X coordinate.
                    candidate_x, candidate_y = tallest["odometry"]
                    landing_x = candidate_x + rel_x + DESIRED_OFFSET
                    landing_y = candidate_y + rel_y
                    landing_z = -MIN_FLY_HEIGHT  # fly at least 5 m high
                    self.landing_target = (landing_x, landing_y, landing_z)
                    if tallest["height"] is not None:
                        self.get_logger().info(f"Tallest cylinder height: {tallest['height']:.2f} m")
                    else:
                        self.get_logger().info("No height measurement available for candidate.")
                    self.state = "LAND"
                else:
                    self.get_logger().warn("No candidates recorded during survey.")

        elif self.state == "LAND":
            if self.landing_target is not None:
                self.publish_trajectory_setpoint(*self.landing_target, yaw=0.0)
                if self.is_at_target_position((self.landing_target[0], self.landing_target[1], 0.0), threshold=2.0):
                    self.get_logger().info("Landing complete.")
            else:
                self.get_logger().warn("Landing target undefined; cannot land.")

        self.offboard_setpoint_counter += 1

def main():
    rclpy.init()
    node = IntegratedSurveyLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down integrated survey landing node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

