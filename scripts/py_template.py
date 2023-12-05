#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2023  template
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


class TemplateNode:
    def __init__(self):
        # constructor
        pass

    def __del__(self):
        # destructor
        pass


def main():
    rospy.init_node("py_template")
    TemplateNode()
    rospy.loginfo("From Python: Hello, world!!")


if __name__ == "__main__":
    main()
