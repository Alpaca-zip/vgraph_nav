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

import rospy
from PIL import Image, ImageDraw


class VgraphPlannerNode:
    def __init__(self):
        self.pgm_file_path = rospy.get_param("~pgm_file", "map.pgm")
        self.save_graph_file_path = rospy.get_param("~save_graph_file", "vgraph.png")
        self.resolution = rospy.get_param("~resolution", 0.1)
        self.start_point = rospy.get_param("~start_point", (200, 30))
        self.end_point = rospy.get_param("~end_point", (150, 370))
        up_scaled_image = self.change_image_resolution(
            self.pgm_file_path, self.resolution
        )
        corners = self.find_black_pixel_corners(up_scaled_image)
        corners.append(self.start_point)
        corners.append(self.end_point)
        line_marked_image = self.draw_lines_between_corners(up_scaled_image, corners)
        line_marked_image.save(self.save_graph_file_path)

    def __del__(self):
        pass

    def change_image_resolution(self, image, resolution):
        original_image = Image.open(image)
        width = int(original_image.width * resolution)
        height = int(original_image.height * resolution)
        black_pixels = self.find_black_pixels(original_image)
        down_scaled_image = original_image.resize((width, height), Image.NEAREST)
        down_scaled_image = self.apply_black_pixels(
            down_scaled_image, black_pixels, resolution
        )
        up_scaled_image = down_scaled_image.resize(original_image.size, Image.NEAREST)
        return up_scaled_image

    def find_black_pixels(self, image):
        black_pixels = []
        for y in range(image.height):
            for x in range(image.width):
                if image.getpixel((x, y)) == 0:
                    black_pixels.append((x, y))
        return black_pixels

    def apply_black_pixels(self, image, black_pixels, scale):
        for x, y in black_pixels:
            scaled_x = int(x * scale)
            scaled_y = int(y * scale)
            if 0 <= scaled_x < image.width and 0 <= scaled_y < image.height:
                image.putpixel((scaled_x, scaled_y), 0)
        return image

    def find_black_pixel_corners(self, image):
        corners = []
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
        return corners

    def draw_lines_between_corners(self, image, corners):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)
        for i in range(len(corners)):
            for j in range(i + 1, len(corners)):
                if not self.check_line_crossing(image, corners[i], corners[j]):
                    draw.line([corners[i], corners[j]], fill=(255, 0, 0), width=1)
        return rgb_image

    def check_line_crossing(self, image, start, end):
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
            if (x, y) != start and (x, y) != end:
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


def main():
    rospy.init_node("vgraph_planner_node")
    VgraphPlannerNode()


if __name__ == "__main__":
    main()
