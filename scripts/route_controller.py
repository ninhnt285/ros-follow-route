#!/usr/bin/env python

import rospy
from turtlebot import Turtlebot
from geometry_msgs.msg import Point
from math import pi


class RouteController():
    def __init__(self, mapData, robot):
        self.mapData = mapData
        self.robot = robot

        self.index = 0

    def start(self):
        rospy.sleep(1.0)
        while self.index < len(mapData):
            next_point = mapData[self.index]['position']
            velocity = mapData[self.index]['velocity']
            robot.move_to_point(next_point, velocity)
            self.index += 1
        print("Done")


if __name__ == "__main__":
    mapData = [
        {'position': Point(2.0, 2.0, 0.0), 'velocity': 0.3},
        {'position': Point(-2.0, 2.0, 0.0), 'velocity': 0.4},
        {'position': Point(-2.0, -2.0, 0.0), 'velocity': 0.2},
        {'position': Point(0, 0, 0.0), 'velocity': 0.3}
    ]
    robot = Turtlebot()
    route_controller = RouteController(mapData, robot)
    route_controller.start()

    # robot.stop_robot()

    # rospy.sleep(2.0)
    # Rotate Test
    # robot.rotate(radians(170))

    # Rotate to Angle Test
    # robot.rotate_to_angle(radians(45))
    # robot.rotate_to_angle(radians(170))
    # robot.rotate_to_angle(-radians(170))
    # robot.rotate_to_angle(0.0)

    # Go to Point
    # target_point = Point(2, 2, 0)
    # rospy.sleep(2.0)

    # robot.move_straight()
    # rospy.sleep(0.2)
    # robot.stop_robot()

    # (position, rotation) = robot.get_odom()
    # print(position.x, position.y)
    # print(degrees(rotation))

    # target_angle = robot.find_target_angle(target_point)
    # print(degrees(target_angle))

    # diff_angle = robot.find_diff_angle(target_angle)
    # print(degrees(diff_angle))

    # robot.move_to_point(target_point, 2.0)

    # (point, rotation) = robot.get_odom()
    # print(point)
    # print(rotation)
