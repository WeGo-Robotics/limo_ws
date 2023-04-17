#!/usr/bin/env python3 

import rospy  # Importing rospy module for ROS node communication
from geometry_msgs.msg import Twist

# Importing Twist message type from geometry_msgs package
from sensor_msgs.msg import LaserScan

# Importing LaserScan message type from sensor_msgs package
from math import *  # Importing math module for mathematical operations


class Limo_obstacle_avoidence:
    def __init__(self):
        rospy.init_node(
            "laser_scan_node"
        )  # Initializing the ROS node with name "laser_scan_node"
        rospy.Subscriber(
            "/scan", LaserScan, self.laser_callback
        )  # Subscribing to "/scan" topic with LaserScan message type and calling laser_callback function
        self.pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=3
        )  # Creating a Publisher object to publish Twist message on "/cmd_vel" topic with queue_size of 3
        self.rate = rospy.Rate(
            30
        )  # Creating a Rate object with 30 Hz frequency to control the publishing rate

        self.cmd_vel_msg = Twist()  # Creating an object of Twist message

        self.msg = None  # Initializing msg variable with None value
        self.lidar_flag = False  # Initializing lidar_flag variable with False value
        self.dist_data = 0  # Initializing dist_data variable with 0 value
        self.direction = None  # Initializing direction variable with None value
        self.is_scan = False  # Initializing is_scan variable with False value

        self.obstacle_ranges = []  # Initializing obstacle_ranges list with empty value
        self.center_list_left = (
            []
        )  # Initializing center_list_left list with empty value
        self.center_list_right = (
            []
        )  # Initializing center_list_right list with empty value

        self.scan_dgree = 60  # Initializing scan_dgree variable with 60 value
        self.min_dist = 0.2  # Initializing min_dist variable with 0.2 value

        self.speed = 0  # Initializing speed variable with 0 value
        self.angle = 0  # Initializing angle variable with 0 value
        self.default_speed = 0.15  # Initializing default_speed variable with 0.15 value
        self.default_angle = 0.0  # Initializing default_angle variable with 0.0 value
        self.turning_speed = 0.08  # Initializing turning_speed variable with 0.08 value
        self.backward_speed = (
            -0.08
        )  # Initializing backward_speed variable with -0.08 value

        self.OBSTACLE_PERCEPTION_BOUNDARY = (
            20  # Initializing OBSTACLE_PERCEPTION_BOUNDARY variable with 20 value
        )

        self.ranges_length = None  # Initializing ranges_length variable with None value

    def laser_callback(self, msg):
        self.msg = (
            msg  # Storing the message received from "/scan" topic in self.msg variable
        )

        if (
            len(self.obstacle_ranges) > self.OBSTACLE_PERCEPTION_BOUNDARY
        ):  # Checking if the length of obstacle_ranges list is greater than OBSTACLE_PERCEPTION_BOUNDARY value
            self.obstacle_exit = True  # Setting obstacle_exit variable to True
        else:
            self.obstacle_exit = False  # Setting obstacle_exit variable to False
        self.is_scan = True  # Setting is_scan variable to True
        self.obstacle_ranges = []  # Emptying the obstacle_ranges list

    def LiDAR_scan(self):
        # Create an empty list to store detected obstacles
        obstacle = []
        # If the LiDAR has not been initialized, initialize it
        if self.lidar_flag == False:
            # Convert the angle increment values to degrees for the given number of ranges
            self.degrees = [
                (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
                for i, data in enumerate(self.msg.ranges)
            ]
            # Record the length of the ranges
            self.ranges_length = len(self.msg.ranges)
            # Set the flag to True to indicate that the LiDAR is initialized
            self.lidar_flag = True

        # Loop over the ranges in the message and check for obstacles
        for i, data in enumerate(self.msg.ranges):
            # If a distance measurement is less than 0.3m and the angle is within the scan degree range,
            # append the index of that measurement to the obstacle list and record the distance
            if 0 < data < 0.3 and -self.scan_dgree < self.degrees[i] < self.scan_dgree:
                obstacle.append(i)
                self.dist_data = data

        # If any obstacles were detected, calculate their start and end indices and extract the range data
        if obstacle:
            first = obstacle[0]
            first_dst = first
            last = obstacle[-1]
            last_dst = self.ranges_length - last
            self.obstacle_ranges = self.msg.ranges[first : last + 1]
        # If no obstacles were detected, set the indices and distance values to 0
        else:
            first, first_dst, last, last_dst = 0, 0, 0, 0

        # Return the start and end indices of the detected obstacles and the indices of the first and last ranges
        # that did not detect an obstacle
        return first, first_dst, last, last_dst

    def move_direction(self, last, first):
        # Check the direction of the obstacle and adjust the speed and angle accordingly
        if self.direction == "right":
            # If the obstacle is on the right, turn left towards the center of the gap
            for i in range(first):
                self.center_list_left.append(i)
            Lcenter = self.center_list_left[floor(first / 2)]
            center_angle_left = -self.msg.angle_increment * Lcenter
            self.angle = center_angle_left
            self.speed = self.default_speed

        elif self.direction == "left":
            # If the obstacle is on the left, turn right towards the center of the gap
            for i in range(len(self.msg.ranges)):
                self.center_list_right.append(last + i)
            Rcenter = self.center_list_right[
                floor(last + (self.ranges_length - last) / 2)
            ]
            center_angle_right = self.msg.angle_increment * Rcenter
            self.angle = center_angle_right / 2.5
            self.speed = self.default_speed

        elif self.direction == "back":
            # If there is no gap ahead, move backward
            self.angle = self.default_angle
            self.speed = self.backward_speed

        else:
            # If the gap is ahead and no obstacle in the way, move forward
            self.angle = self.default_angle
            self.speed = self.default_speed

    def compare_space(self, first_dst, last_dst):
        # Check if there are too many obstacles detected
        if self.obstacle_exit == True:
            # If there are obstacles and the first obstacle is farther than the last obstacle and the distance of the obstacle is greater than the minimum distance, turn right
            if first_dst > last_dst and self.dist_data > self.min_dist:
                self.direction = "right"
            # If there are obstacles and the last obstacle is farther than the first obstacle and the distance of the obstacle is greater than the minimum distance, turn left
            elif first_dst <= last_dst and self.dist_data > self.min_dist:
                self.direction = "left"
            # If there are obstacles and the distance of the obstacle is less than or equal to the minimum distance, move back
            else:
                self.direction = "back"
        # If there are no obstacles detected, move forward
        else:
            self.direction = "front"

    def main(self):
        # Check if LiDAR scan is available
        if self.is_scan == True:
            # Get distance to the first and last obstacle detected by LiDAR scan
            first, first_dst, last, last_dst = self.LiDAR_scan()

            # Compare the distance to the first and last obstacles and update robot's angle accordingly
            self.compare_space(first_dst, last_dst)

            # Determine the direction of robot based on the last and first obstacles detected by LiDAR scan
            self.move_direction(last, first)

            # Set robot's linear and angular velocities based on the updated angle
            self.cmd_vel_msg.linear.x = self.speed
            self.cmd_vel_msg.angular.z = self.angle

            # Publish the velocity command to move the robot
            self.pub.publish(self.cmd_vel_msg)

            # Wait for a short period of time to achieve the desired rate
            self.rate.sleep()


# Check if this is the main module that is being run
if __name__ == "__main__":
    # Create an instance of the Limo_obstacle_avoidence class
    limo_obstacle_avoidence = Limo_obstacle_avoidence()
    try:
        # Start a loop that will continue until ROS is shutdown
        while not rospy.is_shutdown():
            # Call the main method of the Limo_obstacle_avoidence class
            limo_obstacle_avoidence.main()
    # Catch the ROSInterruptException to ensure a clean exit
    except rospy.ROSInterruptException:
        pass