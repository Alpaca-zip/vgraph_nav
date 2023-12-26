#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2023  Alapaca-zip
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

import mip
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image, ImageDraw


class VgraphPlannerNode:
    def __init__(self):
        self.map_file_path = rospy.get_param("~map_file", "map.yaml")
        self.test_folder_path = rospy.get_param("~test_folder", "test")
        self.down_scale_factor = rospy.get_param("~down_scale_factor", 0.1)
        self.clearance = rospy.get_param("~clearance", 0.1)
        self.use_origin = rospy.get_param("~use_origin", True)
        self.start_point = None
        self.end_point = None

        self.original_image, self.resolution, self.origin = self.load_map_file(
            self.map_file_path
        )

        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

    def __del__(self):
        pass

    def initial_pose_callback(self, msg):
        self.start_point = self.pose_to_pixel(
            (msg.pose.pose.position.x, msg.pose.pose.position.y)
        )
        rospy.loginfo("Set initial pose.")

    def goal_callback(self, msg):
        if self.start_point is None:
            rospy.logwarn("Initial pose is not set. Please set initial pose first.")
            return

        self.end_point = self.pose_to_pixel((msg.pose.position.x, msg.pose.position.y))
        rospy.loginfo("Set goal.")

        self.make_plan(self.original_image, self.start_point, self.end_point)
        self.start_point = None
        self.end_point = None

    def pose_to_pixel(self, pose):
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pixel_x = int(round((pose[0] - origin_x) / self.resolution))
        pixel_y = self.original_image.height - int(
            round((pose[1] - origin_y) / self.resolution)
        )

        return (pixel_x, pixel_y)

    def make_plan(self, image, start, goal):
        print("\033[92m[Make Plan] Process started.\033[0m")

        (
            enlarged_image,
            down_scaled_image,
            up_scaled_image,
        ) = self.change_image_resolution(image, self.down_scale_factor)

        print("\033[92m[Make Plan] Image resolution changed.\033[0m")

        corners = []
        corners.append(start)
        self.find_black_pixel_corners(up_scaled_image, corners)
        corners.append(goal)

        valid_edges = self.get_valid_edges(up_scaled_image, corners)

        shortest_path_edges = self.calculate_shortest_path(corners, valid_edges)

        if shortest_path_edges is not None:
            path_graph_image = self.draw_lines_between_corners(
                up_scaled_image, corners, valid_edges
            )
            optimized_path_image_upscaled = self.draw_lines_between_corners(
                up_scaled_image, corners, shortest_path_edges
            )
            optimized_path_image_original = self.draw_lines_between_corners(
                image, corners, shortest_path_edges
            )

            image.save(self.test_folder_path + "/original.png")
            enlarged_image.save(self.test_folder_path + "/enlarged.png")
            down_scaled_image.save(self.test_folder_path + "/down_scaled.png")
            up_scaled_image.save(self.test_folder_path + "/up_scaled.png")
            path_graph_image.save(self.test_folder_path + "/path_graph.png")
            optimized_path_image_upscaled.save(
                self.test_folder_path + "/optimized_path_upscaled.png"
            )
            optimized_path_image_original.save(
                self.test_folder_path + "/optimized_path_original.png"
            )
            print(
                f"\033[92m[Make Plan] Images saved to test folder: {self.test_folder_path}.\033[0m"
            )
            print("\033[92m[Make Plan] Process completed successfully.\033[0m")

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

    def change_image_resolution(self, image, factor):
        width = int(image.width * factor)
        height = int(image.height * factor)
        black_pixels = self.find_black_pixels(image)

        enlarged_image, black_pixels = self.enlarge_black_pixels(
            image, black_pixels, self.clearance
        )

        down_scaled_image = enlarged_image.resize(
            (width, height), Image.Resampling.NEAREST
        )

        down_scaled_image = self.apply_black_pixels(
            down_scaled_image, black_pixels, factor
        )

        up_scaled_image = down_scaled_image.resize(image.size, Image.Resampling.NEAREST)

        return enlarged_image, down_scaled_image, up_scaled_image

    def find_black_pixels(self, image):
        black_pixels = []

        for y in range(image.height):
            for x in range(image.width):
                if image.getpixel((x, y)) == 0:
                    black_pixels.append((x, y))

        return black_pixels

    def enlarge_black_pixels(self, image, black_pixels, width):
        new_image = image.copy()
        pixel_width = round(width / self.resolution)
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

    def apply_black_pixels(self, image, black_pixels, scale):
        for x, y in black_pixels:
            scaled_x = int(x * scale)
            scaled_y = int(y * scale)
            if 0 <= scaled_x < image.width and 0 <= scaled_y < image.height:
                image.putpixel((scaled_x, scaled_y), 0)

        return image

    def find_black_pixel_corners(self, image, corners):
        for y in range(1, image.height - 1):
            for x in range(1, image.width - 1):
                if image.getpixel((x, y)) == 0:
                    black_neighbors = sum(
                        image.getpixel((j, i)) == 0
                        for i in range(y - 1, y + 2)
                        for j in range(x - 1, x + 2)
                        if (i, j) != (y, x)
                    )
                    if black_neighbors == 3:
                        corners.append((x, y))

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

            print("\033[92m[Make Plan] Shortest path calculated.\033[0m")

            return shortest_path_edges
        else:
            rospy.logerr("Optimization failed. No path found.")
            return None

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def draw_lines_between_corners(self, image, corners, edges):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)
        start_point = corners[0]
        end_point = corners[-1]
        radius = 5

        for start, end in edges:
            draw.line([start, end], fill=(255, 0, 0), width=1)

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


def main():
    rospy.init_node("vgraph_planner_node")
    VgraphPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
