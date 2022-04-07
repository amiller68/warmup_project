#!/usr/bin/env python3

# Python libraries
import math
from time import sleep, time
import sys

# ROS
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Returns the distance between two sets of X-Y coordinates
def getDistance(startPos, endPos):
    # print("Start Pos: ", startPos, " | End pos: ", endPos)
    (start_x, start_y) = startPos # Gotta love tuple unpacking
    (end_x, end_y) = endPos
    ret = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    # print("Robot has moved", ret, "m")
    return ret


# Assumes we're always turning with positive angular velocity
def getTurn(startRad, endRad):
    if startRad <= endRad:
        ret = endRad - startRad
    else:
        ret = (math.pi * 2 - startRad) + endRad
    # print("Turned: ", ret)
    return ret

class DriveInSquare(object):
    def __init__(self):
        # Our Initialized Node:
        rospy.init_node('drive_in_square')
        
        #Rospy Params
        self.queue_size = 20  # The size of our message queues
        self.rateLimit = rospy.Rate(10)  # How often we publish messages (2 Hz), utilize with self.rateLimit.sleep()

        # Topic Objects
        self.move = rospy.Publisher('/cmd_vel', Twist , queue_size=self.queue_size) # A publisher to tell Turtlebot what to do
        self.getPose = rospy.Subscriber('/odom', Odometry, callback=self.setPose, queue_size=self.queue_size) # A subscriber to keep track of where we are

        # Movement parameters
        self.loops = 1 # How many times we want the robot to move in a square
        self.forwardDistance = 1 # The length the robot should move in any one direction (m)
        self.turnRadians = math.pi / 2 # How many radians we should turn at one time (rad)
        self.forwardSpeed = 0.2  # m/s
        self.turnSpeed = 0.2  # rad/s
        

        # Movement Commands
        _forwardCommand = Twist()
        _forwardCommand.linear.x = self.forwardSpeed
        _forwardCommand.angular.z = 0.0

        self.forwardCommand = _forwardCommand # initialize a Forward commend

        _turnCommand = Twist()
        _turnCommand.linear.x = 0.0
        _turnCommand.angular.z = self.turnSpeed

        self.turnCommand = _turnCommand # initialize a turn command

        _stopCommand = Twist()
        _stopCommand.linear.x = 0.0
        _stopCommand.angular.z = 0.0

        self.stopCommand = _stopCommand # initialize a stop command

        # Odometry State holder
        self.poseX = None # An x coordinate
        self.poseY = None # A y coordinate
        self.poseRad = None # A radian rotation coordinate

        print("Using Forward Speed: ", self.forwardSpeed)
        print("Using Turn Speed: ", self.turnSpeed)
        

    # A callback for setting Odometry state
    def setPose(self, odomMsg):
        # Extract the robot's position and record it
        self.poseX = odomMsg.pose.pose.position.x
        self.poseY = odomMsg.pose.pose.position.y

        # Calculate the robot's rotations and record it
        orientation = odomMsg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.poseRad = yaw
        # print("New Coords: ", self.poseX, "," ,self.poseY, "@", self.poseRad)
    
    # A stop command to cancel out prior instructions 
    def stop(self, duration):
        t_end = time() + duration
        # Publish for a specified duration of time
        while time() < t_end and not rospy.is_shutdown():
            self.move.publish(self.stopCommand)
            self.rateLimit.sleep()

    # Tell the bot to move forwardDistance meters, and then stop.
    def moveForward(self):
        # Extract an initial position from state
        startPos = (self.poseX, self.poseY)

        # Go forward until we move 'distance' m
        while getDistance(startPos, (self.poseX, self.poseY)) < self.forwardDistance and not rospy.is_shutdown():
            self.move.publish(self.forwardCommand)
            self.rateLimit.sleep()
        self.stop(1)

    # Tell the bot to turn turnRadians rads, and then stop.
    def turn(self):
        # Extract an initial orientation from state
        startRad = self.poseRad

        # Turn until we turn 'radians' radians
        while getTurn(startRad, self.poseRad) < self.turnRadians and not rospy.is_shutdown():
            self.move.publish(self.turnCommand)
            self.rateLimit.sleep()
        self.stop(1)


    # Tell the robot to move in the shape of a square once
    def singleLoop(self):
        # Move along all four edges of the square
        for i in range(4):
            print("Turn ", i)
            # Go Forward by self.squareEdgeLength m
            print("Moving Forward")
            self.moveForward()
            # Then turn by self.turnRadians rads
            print("Turning")
            self.turn()

    # Run method: Describes the behavior of our robot
    def run(self):
        # Wait to have actionable Odometry
        # Use the last set variable to be safe
        print("Waiting on Odemetry")
        while self.poseRad is None and not rospy.is_shutdown():
            print("...")
            sleep(1)
        
        # Move in a square `self.loops` times
        for i in range(self.loops):
            self.singleLoop()
        sys.exit(1)
        
if __name__ == "__main__":
    node = DriveInSquare() 
    node.run()



