#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
# revise from station_helmet1.py

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from math import atan2, sqrt
from nav_msgs.msg import Odometry
import math


def euler_from_quaternion(x1, y1, z1, w1):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x2 = math.sqrt(2)/2
        y2 = 0
        z2 = 0
        w2 = math.sqrt(2)/2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1) 
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z


class SparkRos2(Node):
    def __init__(self):
        super().__init__('pepper_node')
        # self.subscription = self.create_subscription(
        #     PoseStamped,
        #     '/vrpn_client_node/station/pose',
        #     self.vrpnSubscriberCallback,
        #     10)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/mapping/odometry',
            self.odometry_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_client_node/helmet/pose',
            self.goalSubscriberCallback,
            10)

        self.robo_vel = self.create_publisher(Twist, "/airsbot_cmd_vel", 10)

        self.rate = self.create_rate(10)
        self.goal_pose = Pose()
        self.goal_pose.x = 6.0
        self.goal_pose.y = 1.0
        self.self_pose = Pose()
        self.unity_iter = 0
        self.real_iter = 0

        self.move2goal()

    def odometry_callback(self, msg):
        # self.self_pose.x, self.self_pose.y = y_to_z(msg)
        self.self_pose.x = msg.pose.pose.position.x
        self.self_pose.y = msg.pose.pose.position.y
        print("new:", self.self_pose)
        _, _, yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.self_pose.theta = yaw
        self.real_iter += 1

    def goalSubscriberCallback(self, msg):
        # self.goal_pose.x, self.goal_pose.y = y_to_z(msg)
        self.goal_pose.x = 6.0
        self.goal_pose.y = 1.0
        self.unity_iter += 1

    def euclidean_distance(self, goal_pose, current_pos):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow(goal_pose.x - current_pos.x, 2) +
                    pow(goal_pose.y - current_pos.y, 2))

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.self_pose.y, goal_pose.x - self.self_pose.x)
    
    def linear_vel(self, goal_pose, constant=0.4):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return max(min(constant * self.euclidean_distance(goal_pose), 1), -1)

    def angular_vel(self, goal_pose, constant=0.4):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return max(min(constant * (self.steering_angle(goal_pose) - self.self_pose.theta), 1), -1)
    
    def limit(self, x, constant=0.05):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return round(max(min(constant * x, 0.08), -0.08), 3)


    def station_linear_vel(self, self_pose, goal_pose, constant=0.2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        #return max(min(constant * self.euclidean_distance(self_pose, goal_pose), 1), -1)
        return max(min(constant * self.euclidean_distance(self_pose, goal_pose), 0.4), -0.4)

    def station_angular_vel(self, self_pose, goal_pose, constant=0.4):
        def dotproduct(v1, v2):
            return sum((a*b) for a, b in zip(v1, v2))
        def length(v):
            return math.sqrt(dotproduct(v, v))
        def angle(v1, v2):
            return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))
 
        def crossproduct(v1,v2):
            x1, y1 =v1
            x2, y2 =v2
            return x1*y2 -x2*y1
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        #print(self.steering_angle(goal_pose), self.self_pose.theta)
        v1 = [math.cos(self_pose.theta), math.sin(self_pose.theta)]
        v2 = [goal_pose.x - self_pose.x, goal_pose.y - self_pose.y]
        #max(min(constant * (self.steering_angle(self.goal_pose) - self.self_pose.theta), 1), -1)
        return max(min(constant * math.copysign( angle(v1,v2),crossproduct(v1,v2)), 1), -1)  

    def move2goal(self):
        print("Entering move to goal.")
        robo_vel_msg = Twist()
        distance_tolerance = 1
        current_x_vel = 0
        current_y_vel = 0
        while rclpy.ok():
            print("distance: ", self.euclidean_distance(self.goal_pose, self.self_pose))
            
            # input()
            if self.euclidean_distance(self.goal_pose, self.self_pose) >= distance_tolerance:
                print("speed: ", self.station_linear_vel(self.self_pose, self.goal_pose))
                # self.rate.sleep()
                robo_vel_msg.linear.x = self.station_linear_vel(self.self_pose, self.goal_pose)
                print("self: ", self.self_pose)
                robo_vel_msg.linear.y = 0.0
                robo_vel_msg.linear.z = 0.0
                robo_vel_msg.angular.x = 0.0
                robo_vel_msg.angular.y = 0.0
                robo_vel_msg.angular.z = self.station_angular_vel(self.self_pose, self.goal_pose)
                # self.robo_vel.publish(robo_vel_msg)
            else:
                self.rate.sleep()
                robo_vel_msg.linear.x = 0
                robo_vel_msg.linear.y = 0
                robo_vel_msg.linear.z = 0
                robo_vel_msg.angular.x = 0
                robo_vel_msg.angular.y = 0
                robo_vel_msg.angular.z = 0

                self.robo_vel.publish(robo_vel_msg)

        sys.exit()


def main(args=None):
    rclpy.init(args=args)
    robo = SparkRos2()
    # rclpy.spin(robo)
    robo.move2goal()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

