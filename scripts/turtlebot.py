#!/usr/bin/env python

from math import pi, radians, acos, sqrt, degrees
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


DISTANCE_EPSILON = 1e-4
DEFAULT_SPEED = 0.5

class Turtlebot():

    def __init__(self):
        rospy.init_node('turtlebot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.reset_publisher = rospy.Publisher('/reset', Empty, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.cmd = Twist()
        self.orientation = Quaternion()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.position = Point()

        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        self.odom_frame = '/odom'
        self.angular_tolerance = radians(1)
        rospy.on_shutdown(self.shutdown_hook)

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                self.rate.sleep()
                break
            else:
                self.rate.sleep()

    def shutdown_hook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def reset_robot(self):
        while not self.ctrl_c:
            connections = self.reset_publisher.get_num_connections()
            if connections > 0:
                self.reset_publisher.publish()
                self.rate.sleep()
                break
            else:
                self.rate.sleep()

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):
        # Initilize velocities
        self.cmd.linear.x = 0.01
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def calc_distance(self, current_point, target_point):
        d = sqrt(
            (current_point.x - target_point.x)**2 + (current_point.y - target_point.y)**2)
        return d

    def move_straight_velocity(self, target_point, max_speed, speed_step=0.2):
        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        start_point = self.position
        distance = 0.0
        max_distance = self.calc_distance(start_point, target_point)
        speed = min(max_distance, speed_step)

        self.cmd.linear.x = speed

        print("- Move distance: ", max_distance)
        while distance < max_distance  and not self.ctrl_c:
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            distance = self.calc_distance(start_point, self.position)
            # Calculate new speed
            if max_distance - distance < speed:
                speed = 0.1
                self.cmd.linear.x = speed

        self.stop_robot()

    def find_target_angle(self, target_point):
        diff_x = target_point.x - self.position.x
        diff_y = target_point.y - self.position.y
        if abs(diff_x) < DISTANCE_EPSILON:
            if diff_y < 0:
                target_angle = -pi / 2.0
            else:
                target_angle = pi / 2.0
        else:
            target_angle = acos(diff_x / sqrt(diff_x**2 + diff_y**2))
            if diff_y < 0:
                target_angle = -target_angle
        return target_angle

    def move_to_point(self, point, velocity):
        self.rate.sleep()
        print("Move to point", point.x, point.y)
        # Rotate
        target_angle = self.find_target_angle(point)
        self.rotate_to_angle(target_angle)

        distance = self.calc_distance(self.position, point)
        print("- Start moving: velocity = ", velocity)
        while distance > 0.04 and not self.ctrl_c:
            # Find angular step
            target_angle = self.find_target_angle(point)
            diff_angle = self.find_diff_angle(target_angle)
            step_angle = min(0.1, abs(diff_angle))
            if diff_angle < 0:
                step_angle = -step_angle
            self.cmd.angular.z = step_angle
            # Find velocity step
            distance = self.calc_distance(self.position, point)
            step_vel = min(max(distance/2.0, 0.02), velocity)
            self.cmd.linear.x = step_vel
            # Publish vel
            # print(step_angle, step_vel, distance)
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()
        
        # Stop robot
        self.stop_robot()

    def get_odom(self):
        return (self.position, self.yaw)

    def find_diff_angle(self, target_angle):
        norm_target_angle = target_angle

        (_, rotation) = self.get_odom()
        while norm_target_angle < rotation:
            norm_target_angle += (2.0*pi)
        diff_angle = norm_target_angle - rotation

        if diff_angle > pi:
            diff_angle = diff_angle - pi*2.0

        return diff_angle

    def rotate_to_angle(self, target_angle):
        diff_angle = self.find_diff_angle(target_angle)

        (_, rotation) = self.get_odom()
        print("- Start Rotating: ", degrees(rotation),
              degrees(target_angle), degrees(diff_angle))
        self.rotate(diff_angle)
        # print("- End Rotate: ", self.yaw, target_angle)

    def rotate(self, rotate_angle):
        # Get the current position
        (_, rotation) = self.get_odom()
        # Track how far we have turned
        turn_angle = 0
        last_angle = rotation
        goal_angle = rotate_angle

        # Set the movement command to a rotation
        diff_angle = abs(goal_angle)
        if goal_angle > 0:
            self.cmd.angular.z = min(0.3, max(diff_angle, radians(5)))
        else:
            self.cmd.angular.z = -min(0.3, max(diff_angle, radians(5)))

        # Begin the rotation
        # print(abs(turn_angle + self.angular_tolerance), abs(goal_angle))
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not self.ctrl_c:
            # Publish the Twist message and sleep 1 cycle
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Get the current rotation
            (_, rotation) = self.get_odom()
            diff_angle = abs(rotate_angle - turn_angle)
            # Set the movement command to a rotation
            if rotate_angle > 0:
                self.cmd.angular.z = min(0.3, max(diff_angle, radians(2)))
            else:
                self.cmd.angular.z = -min(0.3, max(diff_angle, radians(2)))

            # Compute the amount of rotation since the last lopp
            delta_angle = abs(rotation - last_angle)
            if delta_angle > pi:
                delta_angle = abs(pi*2.0 - delta_angle)
            turn_angle += delta_angle
            last_angle = rotation
            # print(abs(turn_angle + self.angular_tolerance), abs(goal_angle))

        self.stop_robot()

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == '__main__':
    #rospy.init_node('robot_control_node', anonymous=True)
    turtlebot_object = Turtlebot()
    try:
        turtlebot_object.move_straight()

    except rospy.ROSInterruptException:
        pass
