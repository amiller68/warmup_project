#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower():
    def __init__(self):
        rospy.init_node("wall_follower")

        # Specify a shutdown method
        rospy.on_shutdown(self.shutdown)

        # Rospy Params
        self.queue_size = 20  # The size of our message queues
        self.rateLimit = rospy.Rate(10)  # How often we publish messages (2 Hz), utilize with self.rateLimit.sleep()

        # Topic Objects
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=self.queue_size)
        rospy.Subscriber("/scan", LaserScan, self.setScan)

        # The base speed our robot should turn in order to follow a wall
        self.base_angular_vel = 2.5
        # A base speed for our robot to move at to follow a wall
        self.base_linear_vel = .075

        # The closest the robot should get to the wall
        self.wall_dist = .25
        # The point at which the robot should start turning away from a wall in front of it
        self.wall_warn = .5

        # Initialize variables to hold our Scan state

        # All the points around the robot
        self.scanState = None
        # The farthest our robot can scan
        self.max_scan_dist = None
        # A distance reading to warm us if we're going to run into a wall
        self.warn_distance = None

    # Update our state of data ranges from the Subscriber
    def setScan(self, msg):
        if not self.scanState:
            self.max_scan_dist = msg.range_max
        # Extract the observed ranges and save them into state
        self.scanState = msg.ranges

        # If we get a valid reading right in front of us
        if self.scanState[0]:
            # Set it
            self.warn_distance = self.scanState[0]
        else:
            # Otherwise, it means the point is too far away
            self.warn_distance = self.max_scan_dist

    # Find the object closest to the robot
    def closestPoint(self):
        # Extract the ranges our lidar sensor recorded
        ranges = self.scanState
        # Initialize a variable to hold our target
        closest_point = None
        # Iterate through the ranges stored in our scan state, keeping track of range degrees
        for i in range(360):  # Scan through all 360 degrees of data
            # Skip any data points that are 0, as they are too far away
            if ranges[i] == 0:
                continue
            if closest_point:
                # Extract the current closest point:
                r, a = closest_point
                if ranges[i] < r:
                    closest_point = ranges[i], i
            # Set our first closest point
            else:
                closest_point = ranges[i], i
        return closest_point

    # Follow a wall on the robots left based on the closest point to the robot
    # This method determines a linear and angular pid to scale the base speeds
    # And then published the scaled velocity to the cmd_vel topic
    def followWall(self):
        # Determine the closet point
        cp = self.closestPoint()
        linear_pid = 1
        angular_pid = 1

        # In case there's not, go and find one
        if cp:
            # Extract its range and angle
            r, a = cp

            # Determine our angular pid control scalar

            # If the point is on the robots left
            if 0 <= a <= 180:
                # Make the robot turn towards the wall
                angular_pid = (a - 90) / 90
            # If the robot is on the right
            else:
                # Make it turn make towards its left
                angular_pid = -(a - 270) / 90

            # This term is used to accelerate the turns and decelerate the linear movement of the bot when needed
            # If the robot gets two far away we want to be able to recover
            distance_penalty = (r - self.wall_dist) / (self.max_scan_dist - self.wall_dist)

            # Increase the amplitude of our linear velocity accordingly
            if angular_pid > 0:
                angular_pid = angular_pid + distance_penalty
            else:
                angular_pid = angular_pid - distance_penalty

            # Determine Linear PID, and affect the angular pid based on special cases

            # If the robot is approaching a wall
            if self.warn_distance <= self.wall_warn:
                # Slow down as we approach
                linear_pid = pow(((self.warn_distance - self.wall_dist) / (self.wall_warn - self.wall_dist)), 2)
                # Turn faster as we approach
                angular_pid = -(1 - linear_pid)

            # Otherwise, if we're getting too far away from the wall
            elif self.wall_dist < r:
                # Move Slower the faster we're turning,
                # and subtract a penalty based on how far away we are from the wall
                linear_pid = (1 - abs(angular_pid)) - distance_penalty
                # Turn into the wall faster
                angular_pid = angular_pid + distance_penalty

            # If we're within range, merely move slower the faster we're turning
            else:
                linear_pid = 1 - abs(angular_pid)

        # Initialize a movement, calculate its velocity using our PIDs and publish it
        movement = Twist()
        movement.linear.x = linear_pid * self.base_linear_vel
        movement.angular.z = angular_pid * self.base_angular_vel
        self.move.publish(movement)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move.publish(Twist())
        self.rateLimit.sleep()

    def run(self):
        print("Waiting for scan...")
        while self.scanState is None and not rospy.is_shutdown():
            pass
        print("Ready to follow!")
        while not rospy.is_shutdown():
            self.followWall()
            self.rateLimit.sleep()


if __name__ == '__main__':
    try:
        node = WallFollower()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")