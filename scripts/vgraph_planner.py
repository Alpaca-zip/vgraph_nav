#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2023-2024  Alapaca-zip
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
import sys

import cv2
import mip
import numpy
import rospy
import tf
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PIL import Image, ImageDraw
from sensor_msgs.msg import LaserScan


class VgraphPlannerNode:
    def __init__(self):
        self.map_file_path = rospy.get_param("~map_file", "map.yaml")
        self.test_folder_path = rospy.get_param("~test_folder", "test")
        self.epsilon_factor = rospy.get_param("~epsilon_factor", 0.05)
        self.robot_radius = rospy.get_param("~robot_radius", 0.1)
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.vel_linear = rospy.get_param("~vel_linear", 0.1)
        self.vel_theta = rospy.get_param("~vel_theta", 0.1)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.1)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.1)
        self.verbose = rospy.get_param("~verbose", False)
        self.start_point = None
        self.end_point = None

        self.costmap_pub = rospy.Publisher("/cost_map", OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.original_image, self.resolution, self.origin = self.load_map_file(
            self.map_file_path
        )
        black_pixels = self.find_black_pixels(self.original_image)
        self.enlarged_image, _ = self.enlarge_black_pixels(
            self.original_image, black_pixels, self.robot_radius
        )
        self.publish_initial_cost_map(self.enlarged_image)

    def __del__(self):
        pass

    def load_map_file(self, yaml_file_path):
        with open(yaml_file_path, "r") as file:
            map_yaml = yaml.safe_load(file)

        directory = os.path.dirname(yaml_file_path)
        image_file_name = map_yaml.get("image", None)

        if image_file_name is not None:
            image_path = os.path.join(directory, image_file_name)
        else:
            image_path = None

        image = Image.open(image_path)
        resolution = map_yaml.get("resolution", None)
        origin = map_yaml.get("origin", None)

        return image, resolution, origin

    def find_black_pixels(self, image):
        black_pixels = []

        for y in range(image.height):
            for x in range(image.width):
                if image.getpixel((x, y)) == 0:
                    black_pixels.append((x, y))

        return black_pixels

    def enlarge_black_pixels(self, image, black_pixels, width):
        new_image = image.copy()
        pixel_width = math.ceil(width / self.resolution)
        pixel_width = pixel_width + 1
        new_black_pixels = set(black_pixels)

        for x, y in black_pixels:
            for dx in range(-pixel_width, pixel_width + 1):
                for dy in range(-pixel_width, pixel_width + 1):
                    selected_x, selected_y = x + dx, y + dy
                    if (
                        0 <= selected_x < new_image.width
                        and 0 <= selected_y < new_image.height
                    ):
                        if (selected_x, selected_y) not in new_black_pixels:
                            new_image.putpixel((selected_x, selected_y), 0)
                            new_black_pixels.add((selected_x, selected_y))

        return new_image, new_black_pixels

    def publish_initial_cost_map(self, image):
        while True:
            if image is not None and self.costmap_pub.get_num_connections() > 0:
                self.publish_cost_map(image)
                break
            rospy.sleep(5.0)

    def publish_cost_map(self, image, edges=None):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)

        if edges is not None:
            for start, end in edges:
                draw.line([start, end], fill=(128, 128, 128), width=1)

        cost_map_msg = OccupancyGrid()
        cost_map_msg.header.frame_id = "map"
        cost_map_msg.header.stamp = rospy.Time.now()
        cost_map_msg.info.resolution = self.resolution
        cost_map_msg.info.width = rgb_image.width
        cost_map_msg.info.height = rgb_image.height
        cost_map_msg.info.origin.position.x = self.origin[0]
        cost_map_msg.info.origin.position.y = self.origin[1]
        cost_map_msg.info.origin.orientation.w = 1.0
        cost_map_msg.data = []

        for y in reversed(range(rgb_image.height)):
            for x in range(rgb_image.width):
                original_pixel = image.getpixel((x, y))
                rgb_pixel = rgb_image.getpixel((x, y))
                if original_pixel == 0 and rgb_pixel == (128, 128, 128):
                    cost_map_msg.data.append(50)
                elif original_pixel == 0:
                    cost_map_msg.data.append(100)
                else:
                    cost_map_msg.data.append(0)

        self.costmap_pub.publish(cost_map_msg)

    def initial_pose_callback(self, msg):
        self.start_point = self.pose_to_pixel(
            (msg.pose.pose.position.x, msg.pose.pose.position.y)
        )
        rospy.loginfo("Set initial pose.")

    def goal_callback(self, msg):
        if self.start_point is None:
            current_position = self.get_current_position()
            if current_position is not None:
                self.start_point = self.pose_to_pixel(current_position)
                rospy.loginfo("Automatically set initial pose.")
            else:
                rospy.logwarn(
                    "Cannot set initial pose automatically. Please set initial pose manually."
                )
                return
        elif self.enlarged_image is None:
            rospy.logwarn("Map is not loaded. Please wait for the map to be loaded.")
            return

        self.end_point = self.pose_to_pixel((msg.pose.position.x, msg.pose.position.y))
        rospy.loginfo("Set goal.")

        path = self.make_plan(self.start_point, self.end_point)

        self.navigate_along_path(path, msg)

        self.start_point = None
        self.end_point = None

    def odom_callback(self, msg):
        self.current_odom = msg

    def scan_callback(self, msg):
        self.scan_data = []
        self.range_min = msg.range_min

        for angle in range(0, 31):
            self.scan_data.append(msg.ranges[angle])

        for angle in range(330, 360):
            self.scan_data.append(msg.ranges[angle])

    def pose_to_pixel(self, pose):
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pixel_x = int(round((pose[0] - origin_x) / self.resolution))
        pixel_y = self.original_image.height - int(
            round((pose[1] - origin_y) / self.resolution)
        )

        return (pixel_x, pixel_y)

    def pixel_to_pose(self, pixel):
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pose_x = origin_x + (pixel[0] + 0.5) * self.resolution
        pose_y = (
            origin_y + (self.original_image.height - pixel[1] - 0.5) * self.resolution
        )

        return (pose_x, pose_y)

    def make_plan(self, start, goal):
        rospy.loginfo("[Make Plan] Start planning.")
        self.animation_running = True
        timer = rospy.Timer(rospy.Duration(0.1), self.animate_loading)
        corners = []
        corners.append(start)
        corners = self.find_corners(self.enlarged_image, corners)
        corners.append(goal)

        valid_edges = self.get_valid_edges(self.enlarged_image, corners)

        shortest_path_edges = self.calculate_shortest_path(corners, valid_edges)

        if shortest_path_edges is not None:
            shortest_path_edges = self.aligne_path(shortest_path_edges, start)

            self.publish_path(shortest_path_edges)

            self.publish_cost_map(self.enlarged_image, shortest_path_edges)

            path_graph_image = self.draw_path_with_markers(
                self.enlarged_image, corners, valid_edges
            )
            optimized_path_image_enlarged = self.draw_path_with_markers(
                self.enlarged_image, corners, shortest_path_edges
            )
            optimized_path_image_original = self.draw_path_with_markers(
                self.original_image, corners, shortest_path_edges
            )

            self.original_image.save(self.test_folder_path + "/original.png")
            self.enlarged_image.save(self.test_folder_path + "/enlarged.png")
            path_graph_image.save(self.test_folder_path + "/path_graph.png")
            optimized_path_image_enlarged.save(
                self.test_folder_path + "/optimized_path_enlarged.png"
            )
            optimized_path_image_original.save(
                self.test_folder_path + "/optimized_path_original.png"
            )

            self.animation_running = False
            timer.shutdown()
            sys.stdout.write("\r" + " " * 30 + "\r")
            rospy.loginfo("[Make Plan] Finished planning successfully.")

        return shortest_path_edges

    def animate_loading(self, event):
        if self.animation_running:
            chars = "/—\\|"
            char = chars[int(event.current_real.to_sec() * 10) % len(chars)]
            sys.stdout.write("\r" + "Planning... " + char)
            sys.stdout.flush()

    def find_corners(self, image, corners):
        numpy_image = numpy.array(image)
        _, binary = cv2.threshold(numpy_image, 1, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            epsilon = self.epsilon_factor * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            for point in approx:
                corner = tuple(point[0])
                corners.append(corner)

        return corners

    def get_valid_edges(self, image, corners):
        valid_edges = []

        for i in range(len(corners)):
            for j in range(i + 1, len(corners)):
                if not self.check_line_crossing(image, corners[i], corners[j]):
                    valid_edges.append((corners[i], corners[j]))
                    valid_edges.append((corners[j], corners[i]))

        return valid_edges

    def check_line_crossing(self, image, start, end):
        start_pixels = self.get_surrounding_pixels(start)
        end_pixels = self.get_surrounding_pixels(end)
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        err = dx - dy
        not_all_black = False
        not_all_white = False
        is_horizontal_or_vertical = x0 == x1 or y0 == y1

        while True:
            if (x, y) not in start_pixels and (x, y) not in end_pixels:
                pixel = image.getpixel((x, y))

                if pixel != 0:
                    not_all_black = True
                if pixel != 254:
                    not_all_white = True

                if is_horizontal_or_vertical:
                    if x0 == x1:
                        left_pixel = image.getpixel((x - 1, y)) if x > 0 else 0
                        right_pixel = (
                            image.getpixel((x + 1, y)) if x < image.width - 1 else 0
                        )

                        if left_pixel != 254 and right_pixel != 254:
                            not_all_black = True
                    else:
                        top_pixel = image.getpixel((x, y - 1)) if y > 0 else 0
                        bottom_pixel = (
                            image.getpixel((x, y + 1)) if y < image.height - 1 else 0
                        )

                        if top_pixel != 254 and bottom_pixel != 254:
                            not_all_black = True

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        if not is_horizontal_or_vertical:
            not_all_black = True

        return not_all_black and not_all_white

    def get_surrounding_pixels(self, point):
        x, y = point
        pixels = []

        for i in range(-1, 2):
            for j in range(-1, 2):
                pixels.append((x + i, y + j))

        return pixels

    def calculate_shortest_path(self, corners, valid_edges):
        model = mip.Model()
        model.verbose = int(self.verbose)
        x = {(i, j): model.add_var(var_type=mip.BINARY) for i, j in valid_edges}
        start_point = corners[0]
        end_point = corners[-1]

        model += mip.xsum(x[i, j] for i, j in valid_edges if i == start_point) == 1
        model += mip.xsum(x[i, j] for i, j in valid_edges if j == start_point) == 0
        model += mip.xsum(x[i, j] for i, j in valid_edges if i == end_point) == 0
        model += mip.xsum(x[i, j] for i, j in valid_edges if j == end_point) == 1

        for k in corners[1:-1]:
            model += mip.xsum(x[i, j] for i, j in valid_edges if j == k) == mip.xsum(
                x[i, j] for i, j in valid_edges if i == k
            )

        model.objective = mip.minimize(
            mip.xsum(x[i, j] * self.euclidean_distance(i, j) for i, j in valid_edges)
        )

        model.optimize()

        if model.status == mip.OptimizationStatus.OPTIMAL:
            shortest_path_edges = [(i, j) for i, j in valid_edges if x[i, j].x >= 0.99]

            if not shortest_path_edges:
                rospy.logerr("Optimization succeeded, but no valid path was found.")
                return None

            return shortest_path_edges
        else:
            rospy.logerr("Optimization failed. No path found.")
            return None

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def aligne_path(self, edges, start):
        aligned_edges = []
        edge_map = {e[0]: e[1] for e in edges}
        current_point = start

        while current_point in edge_map:
            next_point = edge_map[current_point]
            aligned_edges.append((current_point, next_point))
            current_point = next_point

        return aligned_edges

    def publish_path(self, aligned_edges):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        start_edge = aligned_edges[0]
        start_pose = self.pixel_to_pose((start_edge[0][0], start_edge[0][1]))

        start_pose_stamped = PoseStamped()
        start_pose_stamped.header = path_msg.header
        start_pose_stamped.pose.position.x = start_pose[0]
        start_pose_stamped.pose.position.y = start_pose[1]
        start_pose_stamped.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose_stamped)

        for edge in aligned_edges:
            end_pose = self.pixel_to_pose((edge[1][0], edge[1][1]))

            end_pose_stamped = PoseStamped()
            end_pose_stamped.header = path_msg.header
            end_pose_stamped.pose.position.x = end_pose[0]
            end_pose_stamped.pose.position.y = end_pose[1]
            end_pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(end_pose_stamped)

        self.path_pub.publish(path_msg)

    def draw_path_with_markers(self, image, corners, aligned_edges):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)

        for start, end in aligned_edges:
            draw.line([start, end], fill=(255, 0, 0), width=1)

        start_point = corners[0]
        end_point = corners[-1]
        radius = 5
        draw.ellipse(
            (
                start_point[0] - radius,
                start_point[1] - radius,
                start_point[0] + radius,
                start_point[1] + radius,
            ),
            fill=(0, 255, 0),
        )
        draw.ellipse(
            (
                end_point[0] - radius,
                end_point[1] - radius,
                end_point[0] + radius,
                end_point[1] + radius,
            ),
            fill=(0, 0, 255),
        )

        return rgb_image

    def navigate_along_path(self, edges, final_pose):
        rospy.loginfo("[Navigate] Start navigation.")
        if edges is None or self.current_odom is None or self.scan_data is None:
            return

        for edge in edges:
            rospy.loginfo(f"[Navigate] Moving to waypoint: {edge[1]}.")
            if not self.move_to_goal(edge):
                rospy.logwarn("Aborting navigation.")
                return
            rospy.loginfo("[Navigate] Reached waypoint.")

        if final_pose is not None:
            if not self.rotate_to_final_pose(final_pose):
                rospy.logwarn("Aborting navigation.")
                return
            rospy.loginfo("[Navigate] Reached final pose.")

    def move_to_goal(self, edge):
        current_position = self.get_current_position()
        if current_position is None:
            return False
        end_position = self.pixel_to_pose((edge[1][0], edge[1][1]))
        goal_distance = math.sqrt(
            (end_position[0] - current_position[0]) ** 2
            + (end_position[1] - current_position[1]) ** 2
        )

        while goal_distance > self.goal_tolerance:
            if not self.rotate_to_goal(edge):
                return False

            current_position = self.get_current_position()
            if current_position is None:
                return False
            goal_distance = math.sqrt(
                (end_position[0] - current_position[0]) ** 2
                + (end_position[1] - current_position[1]) ** 2
            )

            if self.check_obstacle():
                rospy.logwarn_throttle(3.0, "Obstacle detected. Stopping the robot.")
                self.send_stop()
                continue

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.vel_linear
            self.cmd_vel_pub.publish(cmd_vel_msg)

            rospy.sleep(0.1)

        self.send_stop()
        return True

    def rotate_to_goal(self, edge):
        rotate_flag = False
        current_position = self.get_current_position()
        if current_position is None:
            return False
        end_position = self.pixel_to_pose((edge[1][0], edge[1][1]))
        goal_angle = math.atan2(
            end_position[1] - current_position[1], end_position[0] - current_position[0]
        )
        current_angle = self.get_current_angle()
        if current_angle is None:
            return False
        diff_angle = self.normalize_angle(goal_angle - current_angle)
        current_speed = self.vel_theta

        while abs(diff_angle) > self.angle_tolerance:
            rotate_flag = True
            current_angle = self.get_current_angle()
            if current_angle is None:
                return False
            diff_angle = self.normalize_angle(goal_angle - current_angle)
            rotation_time = abs(diff_angle) / current_speed
            rotation_direction = 1 if diff_angle > 0 else -1

            if self.check_obstacle():
                rospy.logwarn_throttle(3.0, "Obstacle detected. Stopping the robot.")
                self.send_stop()
                continue

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = rotation_direction * current_speed
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(rotation_time)

            current_speed *= 0.8

        if rotate_flag:
            self.send_stop()

        return True

    def rotate_to_final_pose(self, final_pose):
        final_angle = tf.transformations.euler_from_quaternion(
            [
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w,
            ]
        )[2]
        current_angle = self.get_current_angle()
        if current_angle is None:
            return False
        diff_angle = self.normalize_angle(final_angle - current_angle)
        current_speed = self.vel_theta

        while abs(diff_angle) > self.angle_tolerance:
            current_angle = self.get_current_angle()
            if current_angle is None:
                return False
            diff_angle = self.normalize_angle(final_angle - current_angle)
            rotation_time = abs(diff_angle) / current_speed
            rotation_direction = 1 if diff_angle > 0 else -1

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = rotation_direction * current_speed
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(rotation_time)

            current_speed *= 0.8

        self.send_stop()
        return True

    def send_stop(self):
        start_time = rospy.get_time()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0

        while True:
            self.cmd_vel_pub.publish(cmd_vel_msg)

            current_velocity = math.sqrt(
                self.current_odom.twist.twist.linear.x**2
                + self.current_odom.twist.twist.linear.y**2
                + self.current_odom.twist.twist.angular.z**2
            )

            if current_velocity < 0.01:
                break
            elif rospy.get_time() - start_time > 10:
                rospy.logwarn_throttle(
                    3.0,
                    "It took more than 10 seconds to stop. Please check if the odometry is working properly.",
                )

            rospy.sleep(0.1)

    def check_obstacle(self):
        if self.robot_radius < self.range_min:
            rospy.logwarn(
                "robot_radius is smaller than range_min. Please check the parameters."
            )
            return False

        for distance in self.scan_data:
            if self.range_min < distance < self.robot_radius:
                return True

        return False

    def get_current_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.current_odom.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except:
            rospy.logwarn("Failed to get current position.")
            return None

    def get_current_angle(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.current_odom.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            quaternion = transform.transform.rotation
            euler = tf.transformations.euler_from_quaternion(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )

            return euler[2]
        except:
            rospy.logwarn("Failed to get current angle.")
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


def main():
    rospy.init_node("vgraph_planner_node")
    VgraphPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
