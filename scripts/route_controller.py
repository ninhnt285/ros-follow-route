#!/usr/bin/env python

import rospy
from turtlebot import Turtlebot
from geometry_msgs.msg import Point
from math import pi, degrees, cos, sin


class RouteController():
    def __init__(self, robot, mapData=[]):
        self.robot = robot
        self.mapData = mapData

    def start(self, loop=1):
        rospy.sleep(2.0)

        for _ in range(loop):
            index = 0
            while index < len(self.mapData):
                next_point = self.mapData[index]['position']
                velocity = self.mapData[index]['velocity']
                robot.move_to_point(next_point, velocity)
                index += 1

        self.robot.rotate_to_angle(0)
        print("Done")

    def square(self, loop=1):
        self.mapData = [
            {'position': Point(1.0, 0.0, 0.0), 'velocity': 0.1},
            {'position': Point(1.0, 1.0, 0.0), 'velocity': 0.1},
            {'position': Point(0, 1.0, 0.0), 'velocity': 0.2},
            {'position': Point(0, 0, 0.0), 'velocity': 0.3}
        ]
        self.start(loop)

    def circle(self, center_point, radius, loop=1, steps=32):
        self.mapData = []
        for i in range(0, steps + 1):
            angle = -pi/2.0 + pi*2/float(steps) * i
            x = center_point.x + radius * cos(angle)
            y = center_point.y + radius * sin(angle)
            self.mapData.append(
                {'position': Point(x, y, 0.0), 'velocity': 0.2})

        self.start(loop)

    def eight_path(self, center_point, radius, loop=1, steps=32):
        self.mapData = []
        start_angle = -pi / 2.0
        angle_step = pi*2.0 / float(steps)

        for _ in range(2):
            for i in range(0, steps):
                angle = start_angle + angle_step * i
                x = center_point.x + radius * cos(angle)
                y = center_point.y + radius * sin(angle)
                self.mapData.append(
                    {'position': Point(x, y, 0.0), 'velocity': 0.2})

            center_point.y = -center_point.y
            start_angle = -start_angle
            angle_step = -angle_step

        self.mapData.append({'position': Point(
            center_point.x, center_point.y - radius, 0.0), 'velocity': 0.2})

        self.start(loop)


if __name__ == "__main__":
    # Initial
    robot = Turtlebot()
    route_controller = RouteController(robot)

    # Draw Circle
    # route_controller.circle(Point(0.0, 1.0, 0.0), 1.0)

    # 8 Path
    route_controller.eight_path(Point(0.0, 1.0, 0.0), 1.0)

    # Custom Path
    # mapData = [
    #     {'position': Point(1.0, 0.0, 0.0), 'velocity': 0.1},
    #     {'position': Point(1.0, 1.0, 0.0), 'velocity': 0.1},
    #     {'position': Point(0, 1.0, 0.0), 'velocity': 0.2},
    #     {'position': Point(0, 0, 0.0), 'velocity': 0.3}
    # ]
    # route_controller.mapData = mapData
    # route_controller.start(10)

    # Move to (0, 0)
    # route_controller.robot.move_to_point(Point(0.0, 0.0, 0.0), 0.1)
    # route_controller.robot.rotate_to_angle(0.0)
