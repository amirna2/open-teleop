#!/usr/bin/env python3

# Standard library imports
import argparse
import io
import math
import time
import random
import struct
import sys
import threading
import logging
from logging import Formatter
from typing import Tuple

# Third-party imports
import numpy as np
from PIL import Image as PILImage

# ROS2 imports
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSPresetProfiles,
                       QoSProfile, ReliabilityPolicy)

# ROS2 message imports
from geometry_msgs.msg import (PointStamped, PoseStamped, Pose, Quaternion,
                               PoseWithCovarianceStamped, Twist, Point)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import (BatteryState, CompressedImage, Image, JointState,
                             Joy, LaserScan, NavSatFix, PointCloud2, PointField)
from std_msgs.msg import (Bool, Char, Float32, Float64, Header, Int16, Int32,
                          Int64, Int8, String, UInt16, UInt32, UInt64, UInt8)
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import Log
from builtin_interfaces.msg import Duration

scale_factor = 1.0


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert euler angles to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w

    return q


class DynamicObstacle:
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = 0.3
        self.mass = 1.0

    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def handle_collision(self, other):
        # Normal vector between circles
        nx = other.x - self.x
        ny = other.y - self.y
        dist = math.sqrt(nx*nx + ny*ny)

        if dist == 0:
            return

        # Normalize
        nx /= dist
        ny /= dist

        # Relative velocity
        vx = self.vx - other.vx
        vy = self.vy - other.vy

        # Relative velocity along normal
        vn = vx*nx + vy*ny

        # Don't process if objects moving apart
        if vn > 0:
            return

        # Elastic collision impulse
        j = -(1 + 0.8)*vn  # 0.8 is restitution coefficient
        j /= 1/self.mass + 1/other.mass

        # Apply impulse
        self.vx -= (j/self.mass) * nx
        self.vy -= (j/self.mass) * ny
        other.vx += (j/other.mass) * nx
        other.vy += (j/other.mass) * ny

def update(self):
    current_time = time.time()
    dt = current_time - self.last_update
    self.last_update = current_time

    # Check for collisions
    for i, obs1 in enumerate(self.dynamic_obstacles):
        for obs2 in self.dynamic_obstacles[i+1:]:
            min_dist = obs1.radius + obs2.radius
            if obs1.distance_to(obs2) < min_dist:
                obs1.handle_collision(obs2)

    # Update positions with collision response
    for obs in self.dynamic_obstacles:
        obs.x += obs.vx * dt
        obs.y += obs.vy * dt

        # Bounce off boundaries
        if abs(obs.x) > 3.5:
            obs.vx *= -1
        if abs(obs.y) > 3.5:
            obs.vy *= -1

class CoordinatedLocalizationGenerator:
    def __init__(self, node):
        self.node = node
        self.start_time = time.time()
        self.path_radius = 2.0
        self.linear_velocity = 0.5
        self.angular_velocity = self.linear_velocity / self.path_radius
        self.current_angle = 0.0
        self.last_update = time.time()
        self.last_theta = 0.0
        self.dynamic_obstacles = []

        self.current_x = 0.0  # Start position
        self.current_y = 0.0
        self.current_waypoint = 0
        # Define waypoints in a pattern that avoids known obstacles
        self.waypoints = [
            (0.0, 0.0),    # e.g Loading dock
            (2.0, 2.0),    # Storage
            (-2.0, 2.0),   # Packing
            (-1.0, -1.0)   # Charging
        ]

        # Distance threshold to consider waypoint reached
        self.waypoint_threshold = 0.2

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time

        # Spawn new obstacles
        if len(self.dynamic_obstacles) < 5 and random.random() < 0.1:
            angle = random.uniform(0, 2*math.pi)
            r = random.uniform(1.5, 3.0)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            speed = random.uniform(0.2, 0.8)
            angle = random.uniform(0, 2*math.pi)
            vx = speed * math.cos(angle)
            vy = speed * math.sin(angle)
            self.dynamic_obstacles.append(DynamicObstacle(x, y, vx, vy))

        # Update obstacle positions
        for obs in self.dynamic_obstacles:
            obs.x += obs.vx * dt
            obs.y += obs.vy * dt

            # Bounce off boundaries
            if abs(obs.x) > 3.5:
                obs.vx *= -1
            if abs(obs.y) > 3.5:
                obs.vy *= -1

    def _get_robot_position(self):
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time

        self.update()
        target_x, target_y = self.waypoints[self.current_waypoint]

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.sqrt(dx*dx + dy*dy)

        # Strong attraction to waypoint
        fx = dx/(dist + 1e-6)
        fy = dy/(dist + 1e-6)

        # Moderate obstacle avoidance
        for obs in self.dynamic_obstacles:
            to_robot_x = self.current_x - obs.x
            to_robot_y = self.current_y - obs.y
            obs_dist = math.sqrt(to_robot_x**2 + to_robot_y**2)

            if obs_dist < 1.8:
                magnitude = 1.0 * (1.8 - obs_dist)
                fx += magnitude * to_robot_x/obs_dist
                fy += magnitude * to_robot_y/obs_dist

        # Apply movement
        speed = self.linear_velocity * 1.2  # Slightly faster
        self.current_x += fx * speed * dt
        self.current_y += fy * speed * dt
        theta = math.atan2(fy, fx)

        if dist < self.waypoint_threshold:
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)

        return self.current_x, self.current_y, theta, dt

    def generate_odometry(self):
        msg = Odometry()
        msg.header = self.node._generate_header_with_frame(frame_id="map")
        msg.child_frame_id = "base_link"

        # Get current position with obstacle avoidance
        x, y, theta, dt = self._get_robot_position()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Calculate actual velocity from position change
        msg.twist.twist.linear.x = self.linear_velocity
        msg.twist.twist.angular.z = self.angular_velocity

        # Increase uncertainty due to obstacle avoidance
        pos_cov = 0.2 + 0.1 * (time.time() - self.start_time)
        rot_cov = 0.15 + 0.05 * (time.time() - self.start_time)

        msg.pose.covariance = [pos_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, pos_cov, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, pos_cov, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, rot_cov, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, rot_cov, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, rot_cov]

        return msg

    def generate_pointcloud2(self):
        msg = PointCloud2()
        msg.header = self.node._generate_header_with_frame(frame_id="map")

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.height = 1
        msg.is_bigendian = False
        msg.point_step = 16

        points = []
        num_rays = 270
        max_range = 3.0
        angle_spread = math.pi * 1.5
        vertical_angles = 6  # Number of vertical layers

        x, y, theta, _ = self._get_robot_position()

        # Ray-based point generation for each vertical layer
        for v in range(vertical_angles):
            vertical_angle = math.radians(-15 + (30 * v / (vertical_angles-1)))

            for i in range(num_rays):
                ray_angle = theta - (angle_spread/2) + (angle_spread * i / num_rays)
                angle_from_center = abs(ray_angle - theta)
                max_ray_length = max_range * (1.0 - 0.2 * (angle_from_center / (angle_spread/2)))

                # Check for obstacle intersections
                ray_length = max_ray_length
                ray_blocked = False

                ray_dx = math.cos(ray_angle)
                ray_dy = math.sin(ray_angle)

                # Check each obstacle for intersection
                for obs in self.dynamic_obstacles:
                    # Vector from ray start to obstacle center
                    to_obs_x = obs.x - x
                    to_obs_y = obs.y - y

                    # Project onto ray direction
                    proj = to_obs_x * ray_dx + to_obs_y * ray_dy

                    if 0 <= proj <= max_ray_length:
                        # Find closest point on ray to obstacle center
                        close_x = x + proj * ray_dx
                        close_y = y + proj * ray_dy

                        # Distance from closest point to obstacle center
                        dist = math.sqrt((close_x - obs.x)**2 + (close_y - obs.y)**2)

                        if dist < obs.radius:
                            ray_length = proj - math.sqrt(obs.radius**2 - dist**2)
                            ray_blocked = True
                            break

                point_x = x + ray_length * ray_dx
                point_y = y + ray_length * ray_dy
                point_z = ray_length * math.sin(vertical_angle)

                intensity = 1.0 if ray_blocked else 0.5
                points.append(struct.pack('ffff', point_x, point_y, point_z, intensity))

        # Add points specifically for obstacles - now with vertical distribution
        for obs in self.dynamic_obstacles:
            num_points = 20
            for v in range(vertical_angles):
                # Calculate height for this layer
                z_height = obs.radius * math.sin(math.radians(-15 + (30 * v / (vertical_angles-1))))

                for i in range(num_points):
                    angle = 2 * math.pi * i / num_points
                    px = obs.x + obs.radius * math.cos(angle)
                    py = obs.y + obs.radius * math.sin(angle)
                    points.append(struct.pack('ffff', px, py, z_height, 1.0))

        msg.width = len(points)
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join(points)

        return msg

    def generate_occupancygrid(self):
        msg = OccupancyGrid()
        msg.header = self.node._generate_header_with_frame(frame_id="map")

        msg.info.resolution = 0.05
        msg.info.width = 100
        msg.info.height = 100

        origin_pose = Pose()
        origin_pose.position.x = -2.5
        origin_pose.position.y = -2.5
        origin_pose.position.z = 0.0
        origin_pose.orientation.w = 1.0
        msg.info.origin = origin_pose

        grid = [-1] * (msg.info.width * msg.info.height)
        robot_x, robot_y, theta, _ = self._get_robot_position()

        grid_robot_x = int((robot_x - msg.info.origin.position.x) / msg.info.resolution)
        grid_robot_y = int((robot_y - msg.info.origin.position.y) / msg.info.resolution)

        visibility_radius = 40
        num_rays = 720

        for ray_angle in range(num_rays):
            angle_rad = math.radians(ray_angle * 360.0 / num_rays)
            ray_blocked = False

            ray_dx = math.cos(angle_rad)
            ray_dy = math.sin(angle_rad)

            # Smaller step size for better resolution
            for dist in range(visibility_radius):
                if ray_blocked:
                    break
                ray_x = int(grid_robot_x + dist * ray_dx)
                ray_y = int(grid_robot_y + dist * ray_dy)

                if (0 <= ray_x < msg.info.width and 0 <= ray_y < msg.info.height):
                    idx = ray_y * msg.info.width + ray_x

                    # Check for obstacle intersection
                    world_x = ray_x * msg.info.resolution + msg.info.origin.position.x
                    world_y = ray_y * msg.info.resolution + msg.info.origin.position.y

                    for obs in self.dynamic_obstacles:
                        if math.sqrt((world_x - obs.x)**2 + (world_y - obs.y)**2) < obs.radius:
                            grid[idx] = 100
                            ray_blocked = True
                            break

                    if not ray_blocked:
                        grid[idx] = 0

        msg.data = grid
        return msg

    def generate_markers(self):
        msg = MarkerArray()
        x, y, theta, _ = self._get_robot_position()

        # Path Marker
        path_marker = Marker()
        path_marker.header = Header(frame_id="map")
        path_marker.ns = "path"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        # Create path from waypoints
        points = []
        for wx, wy in self.waypoints:
            point = Point()
            point.x = wx
            point.y = wy
            point.z = 0.0
            points.append(point)
        # Close the loop
        points.append(points[0])

        path_marker.points = points
        path_marker.scale.x = 0.05
        path_marker.color.g = 1.0
        path_marker.color.a = 0.5

        # Robot Position Marker
        robot_marker = Marker()
        robot_marker.header = Header(frame_id="map")
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.TRIANGLE_LIST
        robot_marker.action = Marker.ADD
        # Create triangle vertices
        size = 0.3
        p1 = Point(x=size, y=0.0, z=0.0)  # Front
        p2 = Point(x=-size/1.2, y=size/1.8, z=0.0)  # Back right
        p3 = Point(x=-size/1.2, y=-size/1.8, z=0.0)  # Back left

        robot_marker.points = [p1, p2, p3]
        robot_marker.scale.x = 1.0
        robot_marker.scale.y = 1.0
        robot_marker.scale.z = 1.0
        robot_marker.color.r = 1.0
        robot_marker.color.a = 1.0

        robot_marker.pose.position.x = x
        robot_marker.pose.position.y = y
        robot_marker.pose.position.z = 0.1

        q = quaternion_from_euler(0, 0, theta)
        robot_marker.pose.orientation.x = q[0]
        robot_marker.pose.orientation.y = q[1]
        robot_marker.pose.orientation.z = q[2]
        robot_marker.pose.orientation.w = q[3]


        # Dynamic Obstacles Markers
        obstacle_markers = []
        for i, obs in enumerate(self.dynamic_obstacles):
            marker = Marker()
            marker.header = Header(frame_id="map")
            marker.ns = "obstacles"
            marker.id = i + 10  # Offset to avoid ID conflicts
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = obs.x
            marker.pose.position.y = obs.y
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0

            marker.scale.x = obs.radius * 2
            marker.scale.y = obs.radius * 2
            marker.scale.z = 0.4

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            obstacle_markers.append(marker)

        msg.markers = [path_marker, robot_marker] + obstacle_markers

        # Set lifetime for all markers
        for marker in msg.markers:
            marker.lifetime = Duration(sec=1, nanosec=0)

        return msg

class TopicPublisher:
    def __init__(self, node, topic_name, msg_type, data_generator, log_function=None):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = node.create_publisher(msg_type, topic_name, qos)
        self.data_generator = data_generator
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.log_function = log_function or self.default_log

        self.node = node
        self.logger = node.get_logger()

    def publish(self):
        data = self.data_generator()
        self.publisher.publish(data)
        self.log_function(self.topic_name, data)
        #self.log_publish_info()

    def log_publish_info(self):
        subscribers_count = self.publisher.get_subscription_count()
        self.logger.info(
            f"Published message on topic '/{self.topic_name}' "
            f"(type: {self.msg_type.__name__}) "
            f"with {subscribers_count} subscriber(s)"
        )

    def default_log(self, topic_name, data):
        subscribers_count = self.publisher.get_subscription_count()
        self.logger.info(
            f"Published message on topic '/{self.topic_name}' "
            f"(type: {self.msg_type.__name__}) "
            f"with {subscribers_count} subscriber(s)"
        )

class RosDataGenerator(Node):
    def __init__(self, selected_topics, publish_rate):
        super().__init__('ros_msgs_test')

        self.publish_rate = publish_rate
        self.center_lat = 34.0522
        self.center_lon = -118.2437
        self.radius_km = 5
        self.angle_deg = 0
        self.topic_publishers = []
        self.selected_topics = selected_topics

        # Bouncing ball parameters
        self.ball_x = 20
        self.ball_y =  20
        self.ball_radius = 20
        self.ball_dx = 10
        self.ball_dy = 10
        self.image_width =  320
        self.image_height = 240


        self.localization_generator = CoordinatedLocalizationGenerator(self)

        self.running = False
        self.publish_thread = None

        self.logger = self.get_logger()



    def start(self):
        self.logger.info(f"Starting publisher script with rate {self.publish_rate} Hz")
        self._setup_publishers()

        self.running = True
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.start()

        def shutdown_hook():
            self.logger.info("Shutdown hook called. Stopping publish thread...")
            self.running = False
            if self.publish_thread:
                self.publish_thread.join(timeout=1.0)
            self.logger.info("Publisher script has been shut down.")
            sys.exit(0)

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.logger.info("KeyboardInterrupt received, initiating shutdown...")
        finally:
            shutdown_hook()
            self.destroy_node()
            rclpy.shutdown()

    def _publish_loop(self):
        global scale_factor
        rate = self.create_rate(self.publish_rate)

        while self.running and rclpy.ok():
            try:
                self._publish_messages()
                if scale_factor >= 0.2:
                  scale_factor = scale_factor - 0.1
                else:
                  scale_factor = scale_factor + 0.1
                rate.sleep()
            except rclpy.exceptions.ROSInterruptException:
                self.logger.info("ROS interrupt received in publish loop")
                break


    def _setup_publishers(self):
        self.logger.info("Setting up publishers")
        all_publishers = {
            "string": TopicPublisher(self, "stringpub", String, lambda: String(data=f"Random string: {random.choice(['alpha', 'beta', 'gamma', 'delta'])}")),
            "char": TopicPublisher(self, "charpub", Char, lambda: Char(data=random.randint(0, 2**8 - 1))),
            "uint8": TopicPublisher(self, "uint8pub", UInt8, lambda: UInt8(data=random.randint(0, 2**8 - 1))),
            "uint16": TopicPublisher(self, "uint16pub", UInt16, lambda: UInt16(data=random.randint(0, 2**16 - 1))),
            "uint32": TopicPublisher(self, "uint32pub", UInt32, lambda: UInt32(data=random.randint(0, 2**32 - 1))),
            "uint64": TopicPublisher(self, "uint64pub", UInt64, lambda: UInt64(data=random.randint(0, 2**32 - 1))),
            "int8": TopicPublisher(self, "int8pub", Int8, lambda: Int8(data=random.randint(-2**7, 2**7 - 1))),
            "int16": TopicPublisher(self, "int16pub", Int16, lambda: Int16(data=random.randint(-2**15, 2**15 - 1))),
            "int32": TopicPublisher(self, "int32pub", Int32, lambda: Int32(data=random.randint(-2**31, 2**31 - 1))),
            "int64": TopicPublisher(self, "int64pub", Int64, lambda: Int64(data=random.randint(-2**31, 2**31 - 1))),
            "float32": TopicPublisher(self, "float32pub", Float32, lambda: Float32(data=random.uniform(-1000.0, 1000.0))),
            "float64": TopicPublisher(self, "float64pub", Float64, lambda: Float64(data=random.uniform(-1000.0, 1000.0))),
            "bool": TopicPublisher(self, "boolpub", Bool, lambda: Bool(data=random.choice([True, False]))),
            "navsatfix": TopicPublisher(self, "navsatfixpub", NavSatFix, self._generate_navsatfix),
            "batterystate": TopicPublisher(self, "batterystatepub", BatteryState, self._generate_batterystate),
            "image": TopicPublisher(self, "imagepub", Image, self._generate_image),
            "compressedimage": TopicPublisher(self, "compressedimagepub", CompressedImage, self._generate_compressedimage),
            "pointcloud2": TopicPublisher(self, "pointcloud2pub", PointCloud2, self.localization_generator.generate_pointcloud2),
            "laserscan": TopicPublisher(self, "laserscanpub", LaserScan, self._generate_laserscan),
            "pointstamped": TopicPublisher(self, "pointstampedpub", PointStamped, self._generate_pointstamped),
            "posestamped": TopicPublisher(self, "posestampedpub", PoseStamped, self._generate_posestamped),
            "posewithcovariancestamped": TopicPublisher(self, "posewithcovariancestampedpub", PoseWithCovarianceStamped, self._generate_posewithcovariancestamped),
            "twist": TopicPublisher(self, "twistpub", Twist, self._generate_twist),
            "occupancygrid": TopicPublisher(self, "occupancygridpub", OccupancyGrid, self.localization_generator.generate_occupancygrid, self._log_occupancygrid),
            "odometry": TopicPublisher(self, "odometrypub", Odometry, self.localization_generator.generate_odometry),
            "path": TopicPublisher(self, "pathpub", Path, self._generate_path),
            "log": TopicPublisher(self, "logpub", Log, self._generate_log),
            "jointstate": TopicPublisher(self, "jointstatepub", JointState, self._generate_jointstate),
            "joy": TopicPublisher(self, "joypub", Joy, self._generate_joy),
            "markerarray": TopicPublisher(self, "markerarraypub", MarkerArray, self.localization_generator.generate_markers),
        }

        if self.selected_topics:
            self.topic_publishers = [all_publishers[topic] for topic in self.selected_topics if topic in all_publishers]
        else:
            self.topic_publishers = list(all_publishers.values())


    def _publish_messages(self):
        for publisher in self.topic_publishers:
            publisher.publish()

    #
    # Data type generators
    #

    def _generate_pointstamped(self):
        msg = PointStamped()
        msg.header = self._generate_header_with_frame(frame_id="map")
        msg.point.x = random.uniform(-10, 10)
        msg.point.y = random.uniform(-10, 10)
        msg.point.z = random.uniform(-10, 10)
        self.logger.info(f"PointStamped msg: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")
        return msg

    def _generate_posestamped(self):
        msg = PoseStamped()
        msg.header = self._generate_header_with_frame(frame_id="map")
        msg.pose.position.x = random.uniform(-10, 10)
        msg.pose.position.y = random.uniform(-10, 10)
        msg.pose.position.z = random.uniform(-10, 10)
        msg.pose.orientation.x = random.uniform(-1, 1)
        msg.pose.orientation.y = random.uniform(-1, 1)
        msg.pose.orientation.z = random.uniform(-1, 1)
        msg.pose.orientation.w = random.uniform(-1, 1)
        return msg

    def _generate_posewithcovariancestamped(self):
        msg = PoseWithCovarianceStamped()
        msg.header = self._generate_header_with_frame(frame_id="map")
        msg.pose.pose = self._generate_posestamped().pose
        msg.pose.covariance = [random.uniform(0, 1) for _ in range(36)]
        return msg

    def _generate_twist(self):
        msg = Twist()
        msg.linear.x = random.uniform(-5, 5)
        msg.linear.y = random.uniform(-5, 5)
        msg.linear.z = random.uniform(-5, 5)
        msg.angular.x = random.uniform(-1, 1)
        msg.angular.y = random.uniform(-1, 1)
        msg.angular.z = random.uniform(-1, 1)
        return msg

    def _generate_occupancygrid(self):
        msg = OccupancyGrid()
        msg.header = self._generate_header_with_frame(frame_id="map")

        # Set map properties
        msg.info.resolution = 0.05  # 5cm per cell
        msg.info.width = 200        # 10 meters wide
        msg.info.height = 100       # 5 meters tall

        # Set the origin to be at the center of the map
        # This places (0,0) at the center rather than the corner
        origin_pose = self._generate_posestamped().pose
        origin_pose.position.x = -2.5  # -width * resolution / 2
        origin_pose.position.y = -2.5  # -height * resolution / 2
        origin_pose.position.z = 0.0

        # Set orientation for the grid (usually identity quaternion)
        origin_pose.orientation.x = 0.0
        origin_pose.orientation.y = 0.0
        origin_pose.orientation.z = 0.0
        origin_pose.orientation.w = 1.0

        msg.info.origin = origin_pose

        # Initialize empty grid
        grid = [0] * (msg.info.width * msg.info.height)

        # Helper function to set grid value
        def set_cell(x, y, value):
            if 0 <= x < msg.info.width and 0 <= y < msg.info.height:
                grid[y * msg.info.width + x] = value

        # Helper function to draw a line (for walls)
        def draw_line(x1, y1, x2, y2):
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            x, y = x1, y1
            sx = 1 if x2 > x1 else -1
            sy = 1 if y2 > y1 else -1

            if dx > dy:
                err = dx / 2.0
                while x != x2:
                    set_cell(x, y, 100)
                    err -= dy
                    if err < 0:
                        y += sy
                        err += dx
                    x += sx
            else:
                err = dy / 2.0
                while y != y2:
                    set_cell(x, y, 100)
                    err -= dx
                    if err < 0:
                        x += sx
                        err += dy
                    y += sy
            set_cell(x2, y2, 100)

        # Helper function to create a room
        def create_room(x, y, width, height, door_positions=None):
            # Walls
            for i in range(width):
                set_cell(x + i, y, 100)  # Top wall
                set_cell(x + i, y + height - 1, 100)  # Bottom wall
            for i in range(height):
                set_cell(x, y + i, 100)  # Left wall
                set_cell(x + width - 1, y + i, 100)  # Right wall

            # Add doors if specified
            if door_positions:
                for door_x, door_y in door_positions:
                    set_cell(door_x, door_y, 0)  # Door is free space

            # Add some furniture (obstacles) inside the room
            for _ in range(int((width * height) * 0.1)):  # 10% coverage
                fx = random.randint(x + 1, x + width - 2)
                fy = random.randint(y + 1, y + height - 2)
                set_cell(fx, fy, 100)

        # Create main layout
        # Main corridor
        corridor_width = 6
        corridor_y = msg.info.height // 2
        for x in range(msg.info.width):
            for y in range(corridor_y - corridor_width//2, corridor_y + corridor_width//2):
                set_cell(x, y, 0)

        # Vertical corridor
        corridor_x = msg.info.width // 2
        for y in range(msg.info.height):
            for x in range(corridor_x - corridor_width//2, corridor_x + corridor_width//2):
                set_cell(x, y, 0)

        # Create rooms
        rooms = [
            # (x, y, width, height, [(door_x, door_y)])
            (10, 10, 30, 20, [(25, 30)]),  # Large room top left
            (60, 10, 25, 25, [(60, 20)]),  # Room top right
            (10, 70, 25, 20, [(20, 70)]),  # Room bottom left
            (60, 70, 30, 20, [(75, 70)]),  # Room bottom right
        ]

        for room in rooms:
            create_room(*room)

        # Add some random noise and unknown areas
        for i in range(len(grid)):
            if grid[i] == 0:  # Only modify free space
                r = random.random()
                if r < 0.05:  # 5% chance of unknown space
                    grid[i] = -1
                elif r < 0.08:  # 3% chance of noise (false positive obstacle)
                    grid[i] = 100

        # Add some "scanning artifacts" (clusters of unknown cells)
        for _ in range(3):  # Add 3 artifact clusters
            cx = random.randint(0, msg.info.width - 1)
            cy = random.randint(0, msg.info.height - 1)
            radius = random.randint(3, 7)
            for x in range(cx - radius, cx + radius):
                for y in range(cy - radius, cy + radius):
                    if ((x - cx)**2 + (y - cy)**2) <= radius**2:  # Circular artifact
                        set_cell(x, y, -1)

        msg.data = grid
        return msg


    def _generate_odometry(self):
        msg = Odometry()
        msg.header = self._generate_header_with_frame(frame_id="map")
        msg.child_frame_id = "base_link"
        msg.pose = self._generate_posewithcovariancestamped().pose
        msg.twist.twist = self._generate_twist()
        msg.twist.covariance = [random.uniform(0, 1) for _ in range(36)]
        return msg

    def _generate_path(self):
        msg = Path()
        msg.header = self._generate_header_with_frame(frame_id="map")
        msg.poses = [self._generate_posestamped() for _ in range(10)]
        return msg

    def _generate_log(self):
        msg = Log()
        msg.level = int.from_bytes(Log.INFO, byteorder='big')
        msg.name = "test_node"
        msg.msg = f"Test log message {random.randint(1, 100)}"
        msg.file = "test_file.py"
        msg.function = "test_function"
        msg.line = random.randint(1, 100)
        return msg

    def _generate_jointstate(self):
        msg = JointState()
        msg.header = self._generate_header_with_frame(frame_id="base_link")
        num_joints = random.randint(1, 5)
        msg.name = [f"joint_{i}" for i in range(num_joints)]
        msg.position = [random.uniform(-3.14, 3.14) for _ in range(num_joints)]
        msg.velocity = [random.uniform(-1, 1) for _ in range(num_joints)]
        msg.effort = [random.uniform(-100, 100) for _ in range(num_joints)]
        return msg

    def _generate_joy(self):
        msg = Joy()
        msg.header = self._generate_header_with_frame(frame_id="base_link")
        msg.axes = [random.uniform(-1, 1) for _ in range(6)]
        msg.buttons = [random.choice([0, 1]) for _ in range(12)]
        return msg

    def _generate_markerarray(self):
        msg = MarkerArray()
        num_markers = random.randint(2, 5)
        for i in range(num_markers):
            marker = Marker()
            marker.header = self._generate_header_with_frame()
            marker.ns = "test_namespace"
            marker.id = i
            marker.type = random.choice([Marker.CUBE, Marker.SPHERE, Marker.ARROW, Marker.CYLINDER])
            marker.action = Marker.ADD
            marker.pose = self._generate_posestamped().pose
            marker.scale.x = random.uniform(0.1, 1)
            marker.scale.y = random.uniform(0.1, 1)
            marker.scale.z = random.uniform(0.1, 1)
            marker.color.r = random.random()
            marker.color.g = random.random()
            marker.color.b = random.random()
            marker.color.a = 1.0
            marker.lifetime = Duration(sec=5, nanosec=0)
            marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=1.0, y=1.0, z=1.0)]
            msg.markers.append(marker)
        return msg

    def _generate_header_with_frame(self, frame_id=None):
        """Generate header with specified or random frame_id"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        if frame_id:
            header.frame_id = frame_id
        else:
            # List of common ROS frame IDs
            common_frames = [
                "map",
                "odom",
                "base_link",
                "base_footprint",
                "camera_link",
                "laser_link",
                "arm_base_link"
            ]
            header.frame_id = random.choice(common_frames)

        return header

    def _generate_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        return header


    def _generate_navsatfix(self):
        self.angle_deg = (self.angle_deg + 1) % 360
        angle_rad = math.radians(self.angle_deg)
        delta_lat_km = self.radius_km * math.cos(angle_rad)
        delta_lon_km = self.radius_km * math.sin(angle_rad)
        delta_lat_deg = delta_lat_km / 110.574
        delta_lon_deg = delta_lon_km / (111.320 * math.cos(math.radians(self.center_lat)))
        msg = NavSatFix()
        msg.latitude = self.center_lat + delta_lat_deg
        msg.longitude = self.center_lon + delta_lon_deg
        msg.altitude = random.uniform(-1000.0, 10000.0)
        return msg

    def _generate_batterystate(self):
        msg = BatteryState()
        msg.voltage = random.uniform(0.0, 12.0)
        msg.current = random.uniform(-10.0, 10.0)
        msg.charge = random.uniform(0.0, 100.0)
        msg.capacity = random.uniform(0.0, 100.0)
        msg.percentage = random.uniform(0.0, 1.0)
        msg.power_supply_status = random.choice([0, 1, 2, 3, 4])
        return msg

    def _generate_image(self):
        msg = Image()
        msg.height = self.image_height
        msg.width = self.image_width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = self.image_width * 3
        img_data = self._generate_bouncing_ball()
        msg.data = img_data.flatten().tolist()
        return msg

    def _generate_bouncing_ball(self):
        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        self.ball_x += self.ball_dx
        self.ball_y += self.ball_dy
        if self.ball_x - self.ball_radius <= 0 or self.ball_x + self.ball_radius >= self.image_width:
            self.ball_dx = -self.ball_dx
        if self.ball_y - self.ball_radius <= 0 or self.ball_y + self.ball_radius >= self.image_height:
            self.ball_dy = -self.ball_dy
        self.ball_x = max(self.ball_radius, min(self.image_width - self.ball_radius, self.ball_x))
        self.ball_y = max(self.ball_radius, min(self.image_height - self.ball_radius, self.ball_y))
        y, x = np.ogrid[:self.image_height, :self.image_width]
        mask = ((x - self.ball_x)**2 + (y - self.ball_y)**2 <= self.ball_radius**2)
        # Draw random color ball
        image[mask] = [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]
        return image

    def _generate_compressedimage(self):
        msg = CompressedImage()
        msg.format = "jpeg"

        # Create a random image using Pillow
        width, height = 1920,1080
        image = PILImage.new('RGB', (width, height), (255, 255, 255))
        pixels = image.load()

        for i in range(width):
            for j in range(height):
                pixels[i, j] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        # Save the image to a bytes buffer
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG")
        msg.data = list(buffer.getvalue())

        return msg

    def _generate_pointcloud2(self):
        msg = PointCloud2()
        # Set frame id and timestamp
        msg.header = self._generate_header_with_frame(frame_id="map")
        msg.height = 1  # Unordered point cloud, height is 1

        # Define the fields for x, y, z, and intensity
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = len(msg.fields) * 4  # Each field is a 32-bit float
        data = []

        # Define base cube vertices
        base_vertices = [
            (-1, -1, -1),  # V0
            (-1, -1,  1),  # V1
            (-1,  1, -1),  # V2
            (-1,  1,  1),  # V3
            ( 1, -1, -1),  # V4
            ( 1, -1,  1),  # V5
            ( 1,  1, -1),  # V6
            ( 1,  1,  1),  # V7
        ]

        # Scale the vertices
        cube_vertices = [(x * scale_factor, y * scale_factor, z * scale_factor) for x, y, z in base_vertices]

        # Define edges as pairs of vertex indices
        edges = [
            (0, 1), (0, 2), (0, 4),
            (1, 3), (1, 5),
            (2, 3), (2, 6),
            (3, 7),
            (4, 5), (4, 6),
            (5, 7),
            (6, 7),
        ]

        num_points_per_edge = 50000  # Adjust this number for more or fewer points

        # Generate points along each edge
        for edge in edges:
            start = cube_vertices[edge[0]]
            end = cube_vertices[edge[1]]
            for i in range(num_points_per_edge):
                t = i / (num_points_per_edge - 1)  # Parameter t ranges from 0 to 1
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                z = start[2] + t * (end[2] - start[2])
                intensity = 1.0  # You can vary this if needed
                data.append(struct.pack('ffff', x, y, z, intensity))

        # Update width and row_step based on the number of points
        msg.width = len(data)
        msg.row_step = msg.point_step * msg.width

        msg.data = b''.join(data)

        return msg

    def _generate_pointcloud2_v2(self):
        msg = PointCloud2()
        # Set frame id and timestamp
        msg.header = Header()
        msg.header.frame_id = "base_link"  # Replace with your actual frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1  # Unordered point cloud, height is 1

        # Define the fields for x, y, z, and intensity
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = len(msg.fields) * 4  # Each field is a 32-bit float
        data = []

        # Define cube vertices
        cube_vertices = [
            (-1, -1, -1),  # V0
            (-1, -1,  1),  # V1
            (-1,  1, -1),  # V2
            (-1,  1,  1),  # V3
            ( 1, -1, -1),  # V4
            ( 1, -1,  1),  # V5
            ( 1,  1, -1),  # V6
            ( 1,  1,  1),  # V7
        ]

        # Define edges as pairs of vertex indices
        edges = [
            (0, 1), (0, 2), (0, 4),
            (1, 3), (1, 5),
            (2, 3), (2, 6),
            (3, 7),
            (4, 5), (4, 6),
            (5, 7),
            (6, 7),
        ]

        num_points_per_edge = 20  # Adjust this number for more or fewer points

        # Generate points along each edge
        for edge in edges:
            start = cube_vertices[edge[0]]
            end = cube_vertices[edge[1]]
            for i in range(num_points_per_edge):
                t = i / (num_points_per_edge - 1)  # Parameter t ranges from 0 to 1
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                z = start[2] + t * (end[2] - start[2])
                intensity = 1.0  # You can vary this if needed
                data.append(struct.pack('ffff', x, y, z, intensity))

        # Update width and row_step based on the number of points
        msg.width = len(data)
        msg.row_step = msg.point_step * msg.width

        msg.data = b''.join(data)

        return msg

    def _generate_pointcloud2_v1(self):
        msg = PointCloud2()
        # Set frame id and timestamp
        msg.header = Header()
        msg.header.frame_id = "base_link"  # Replace with your actual frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = 10

        # Define the fields for x, y, z, and intensity
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = len(msg.fields) * 4
        msg.row_step = msg.point_step * msg.width

        # Generate data for a cube
        cube_points = [
            (-1, -1, -1), (-1, -1, 1), (-1, 1, -1), (-1, 1, 1),
            (1, -1, -1), (1, -1, 1), (1, 1, -1), (1, 1, 1),
            (0, 0, 0), (0, 0, 0)  # Two extra points at the center to make 10 points
        ]

        data = []
        for point in cube_points:
            x, y, z = point
            intensity = 1.0  # constant intensity
            data.append(struct.pack('ffff', x, y, z, intensity))

        msg.data = b''.join(data)

        return msg

    def _generate_laserscan(self):
        msg = LaserScan()
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / 360
        msg.time_increment = 1.0 / 40 / 360
        msg.scan_time = 1.0 / 40
        msg.range_min = 0.1
        msg.range_max = 100.0
        msg.ranges = [random.uniform(0.1, 100.0) for _ in range(360)]
        msg.intensities = [random.uniform(0.0, 100.0) for _ in range(360)]
        return msg

    def _log_image(self, topic_name, data):
        self.get_logger().info(f"Published to {topic_name}: Image size: {data.width}x{data.height}, Encoding: {data.encoding}, Ball position: ({self.ball_x:.2f}, {self.ball_y:.2f})")

    def _log_navsatfix(self, topic_name, data):
        self.get_logger().info(f"Published to {topic_name}: Lat: {data.latitude:.6f}, Lon: {data.longitude:.6f}, Alt: {data.altitude:.2f}")

    def _log_batterystate(self, topic_name, data):
        self.get_logger().info(f"Published to {topic_name}: Voltage: {data.voltage:.2f}V, Current: {data.current:.2f}A, Charge: {data.charge:.2f}%, Capacity: {data.capacity:.2f}%, Percentage: {data.percentage:.2f}")

    def _log_compressedimage(self, topic_name, data):
        self.get_logger().info(f"Published to {topic_name}: Compressed image format: {data.format}, Data size: {len(data.data)} bytes")

    def _log_pointcloud2(self, topic_name, data):
        self.get_logger().info(f"Published to {topic_name}: PointCloud2 size: {data.width}x{data.height}, Data size: {len(data.data)} bytes")

    def _log_laserscan(self, topic_name, data):
        self.get_logger().info(f"Published to {topic_name}: LaserScan with {len(data.ranges)} points, Range: [{min(data.ranges):.2f}, {max(data.ranges):.2f}]")

    def _log_occupancygrid(self, topic_name, data):
        free = data.data.count(0)
        occupied = data.data.count(100)
        unknown = data.data.count(-1)
        total = len(data.data)
        coverage = ((free + occupied) / total) * 100 if total > 0 else 0

        self.get_logger().info(
            f"Published to {topic_name}: Grid {data.info.width}x{data.info.height} "
            f"({data.info.resolution}m/cell). "
            f"Cells: {total} (Free: {free}, Occupied: {occupied}, Unknown: {unknown}). "
            f"Map coverage: {coverage:.1f}%"
        )

def parse_arguments():
    parser = argparse.ArgumentParser(
        description="ROS2 Topic Publisher",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        help="List of topics to publish (default: all). See available topics below.",
        metavar="TOPIC"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=0.2,
        help="Publishing rate in Hz (default: 0.2)",
        metavar="RATE"
    )
    parser.epilog = """
    available topics:
      string          Random string messages
      char            Random character messages
      uint8           8-bit unsigned integer messages
      uint16          16-bit unsigned integer messages
      uint32          32-bit unsigned integer messages
      uint64          64-bit unsigned integer messages
      int8            8-bit signed integer messages
      int16           16-bit signed integer messages
      int32           32-bit signed integer messages
      int64           64-bit signed integer messages
      float32         32-bit float messages
      float64         64-bit float messages
      bool            Boolean messages
      navsatfix       Navigation satellite fix messages
      batterystate    Battery state messages
      image           Image messages
      compressedimage Compressed image messages
      pointcloud2     Point cloud messages
      laserscan       Laser scan messages
      pointstamped    PointStamped messages
      posestamped     PoseStamped messages
      posewithcovariancestamped PoseWithCovarianceStamped messages
      twist           Twist messages
      occupancygrid   OccupancyGrid messages
      odometry        Odometry messages
      path            Path messages
      log             Log messages
      jointstate      JointState messages
      joy             Joy messages
      markerarray     MarkerArray messages
    """
    return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)
    # Set the global ROS2 logging level
    rclpy.logging.set_logger_level('ros2-pub', rclpy.logging.LoggingSeverity.INFO)

    parsed_args = parse_arguments()
    ros_pub_node = RosDataGenerator(parsed_args.topics, parsed_args.rate)
    ros_pub_node.start()

if __name__ == "__main__":
    main()

