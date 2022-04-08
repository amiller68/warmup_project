#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower():
    def __init__(self):
        rospy.init_node("person_follower")

        # Specify a shutdown method
        rospy.on_shutdown(self.shutdown)

        # Rospy Params
        self.queue_size = 20  # The size of our message queues
        self.rateLimit = rospy.Rate(10)  # How often we publish messages (2 Hz), utilize with self.rateLimit.sleep()

        # Topic Objects
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=self.queue_size)
        rospy.Subscriber("/scan", LaserScan, self.setScan)

        # A class variable to hold our scan state
        self.scanState = None

        # The fastest our robot should turn in order to face a target
        self.max_angular_vel = 1.0
        # The fastest our robot should move towards a target
        self.max_linear_vel = 0.25
        # The closest the robot should get to the target object
        self.target_dist = 0.2
        # The farthest the robot should be able to sense as a valid target
        self.max_dist = 3.0


    # Update our state of data ranges from the Subscriber
    def setScan(self, msg):
        # Extract the observed ranges and save them into state
        self.scanState = msg.ranges

    # follow a person given our current scan state
    # Publishes a velocity that orients the bot towards the thing closest to it
    # Tells the bot to do nothing if there's nothing close by
    def followPerson(self):
        # Extract the ranges our lidar sensor recorded
        ranges = self.scanState
        # Initialize a variable to hold our target
        closest_point = None

        # Iterate through the ranges stored in our scan state, keeping track of range degrees
        for i in range(360):  # Scan through all 360 degrees of data
            # Skip any data points that are 0, as they are too far away
            if ranges[i] == 0 or ranges[i] >= self.max_dist:
                continue
            if closest_point:
                # Extract the current closest point:
                r, a = closest_point
                if ranges[i] < r:
                    closest_point = ranges[i], i
            # Set our first closest point
            else:
                closest_point = ranges[i], i

        # Make the robot move based on the closest point
        movement = Twist()
        if not closest_point:
            print("There's nothing nearby!")
        else:
            r, a = closest_point

            # Only try and move forward if the object is in front of the robot
            if 0 <= a < 90:
                movement.linear.x = (r - self.target_dist) * ((90 - a)/90) * self.max_linear_vel
            elif 270 <= a < 360:
                _a = 360 - a
                movement.linear.x = (r - self.target_dist) * ((90 - _a)/90) * self.max_linear_vel

            if 0 < a < 180:
                movement.angular.z = a / 180 * self.max_angular_vel
            else: # 180 <= a < 359:
                a = 360 - a
                movement.angular.z = - a / 180 * self.max_angular_vel
            # print(r, "meter @ ", a, "degrees left of center")
            print("Moving towards object: ", movement.linear.x, "@", movement.angular.z)
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
            self.followPerson()
            self.rateLimit.sleep()

if __name__ == '__main__':
    try:
        node = PersonFollower()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")