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
        rospy.Subscriber("/scan", LaserScan, self.updateScanState)

        # A class variable to hold our scan state
        self.scanState = None

        # The fastest our robot should turn in order to follow a wall
        self.max_angular_vel = 1.0
        # The fastest our robot should move along a wall
        self.max_linear_vel = .75
        # The closest the robot should get to the wall
        self.min_dist = 0.25
        # Farthest the robot should be from a wall
        self.max_dist = 0.5
        self.max_scan = 4.0


    # Update our state of data ranges from the Subscriber
    def updateScanState(self, msg):
        # Extract the observed ranges and save them into state
        self.scanState = msg.ranges

    def closestPoint(self):
        ranges = self.scanState
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

    def findWall(self):
        if self.min_dist <= self.scanState[90] <= self.max_dist:
            return
        movement = Twist()
        cp = self.closestPoint()
        movement.linear.x = self.max_linear_vel
        # If there's nothing nearby, go find something
        print("Moving towards wall")
        while (not cp or cp[0] > self.max_dist) and not rospy.is_shutdown():
            if cp:
                r = cp[0]
                movement.linear.x = (r - self.max_dist) / (self.max_scan - self.max_dist) * self.max_linear_vel + .05
            self.move.publish(movement)
            cp = self.closestPoint()
            self.rateLimit.sleep()

        movement = Twist()
        horizontal_dist = self.scanState[90]
        print("Orienting towards wall")
        while horizontal_dist == 0 and not rospy.is_shutdown():
            movement.angular.z = - self.max_angular_vel
            horizontal_dist = self.scanState[90]
            self.move.publish(movement)
            self.rateLimit.sleep()

        print("Found a distance")
        while not (horizontal_dist < self.max_dist) and not rospy.is_shutdown():
            movement.angular.z = - (horizontal_dist - self.max_dist) / (self.max_scan - self.max_dist) * self.max_angular_vel - .1
            print("Turning at: ", movement.angular.z)
            horizontal_dist = self.scanState[90]
            self.move.publish(movement)
            self.rateLimit.sleep()
        movement = Twist()
        self.move.publish(movement)

    # follow a wall given our current scan state
    # Publishes a velocity that orients the bot towards the wall
    # Keeps walls on the robots left
    def followWall(self):
        # Extract the ranges our lidar sensor recorded
        ranges = self.scanState

        # Use the measurements 45 deg and 135 deg left of center in order to orient the robot
        range_45_deg = ranges[45]
        range_135_deg = ranges[135]

        # Orient the robot to follow a wall at any distance

        movement = Twist()
        # If we're heading towards a wall
        if abs(range_45_deg - range_135_deg) < .1:
            movement.linear.x = self.max_linear_vel
        elif range_45_deg < range_135_deg:
            print("On a collision course")
            # Take the difference of our measurements
            diff = range_135_deg - range_45_deg
            movement.angular.z = - self.max_angular_vel / 10
        elif range_45_deg > range_135_deg:
            print("Moving away from a wall")
            diff = range_135_deg - range_45_deg
            movement.angular.z = self.max_angular_vel / 10
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

        # print("Looking for a wall")
        # self.findWall()

        while not rospy.is_shutdown():
            self.followWall()
            self.rateLimit.sleep()

if __name__ == '__main__':
    try:
        node = WallFollower()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")