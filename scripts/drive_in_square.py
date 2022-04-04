#!/usr/bin/env python3
import rospy
from time import sleep, time
from geometry_msgs.msg import Twist

class DriveInSquare(object):
    def __init__(self):
        # Our Initialized Node:
        rospy.init_node('drive_in_square')

        # A publisher to tell Turtlebot what to do
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # The length the robot should move in any one direction, m
        self.squareEdgeLength = 1
        # How many radians we should turn at one time
        self.turnRadians = 1.5708
        # How fast we want to drive and turn the bot
        self.forwardSpeed = 0.25 #m/s
        self.turnSpeed = -0.25 #radians/s, ~28.5 degrees per second


        _forwardCommand = Twist()
        _forwardCommand.linear.x = self.forwardSpeed
        _forwardCommand.angular.z = 0.0

        # initialize a Forward commend
        self.forwardCommand = _forwardCommand

        _turnCommand = Twist()
        _turnCommand.linear.x = 0.0
        _turnCommand.angular.z = self.turnSpeed

        # initialize a turn command
        self.turnCommand = _turnCommand

        _stopCommand = Twist()
        _stopCommand.linear.x = 0.0
        _stopCommand.angular.z = 0.0

        # initialize a stop command
        self.stopCommand = _stopCommand
    
    # A stop command to cancel out prior instructions 
    def stop(self):
        t_end = time() + 1.5
        while time() < t_end:
            self.move.publish(self.stopCommand)

    # Tell the bot to move squareEdgeLength meters, and then stop
    def forward(self):
        forward_time = abs(self.squareEdgeLength / self.forwardSpeed)
        t_end = time() + forward_time
        while time() < t_end:
            self.move.publish(self.forwardCommand)
        # sleep(abs(self.squareEdgeLength / self.forwardSpeed))
        self.stop()

    # Tell the bot to turn 90 degress, and then stop
    def turn(self):
        turn_time = abs(self.turnRadians / self.turnSpeed)
        t_end = time() + turn_time
        while time() < t_end:
            self.move.publish(self.turnCommand)
        # sleep(abs(self.turnRadians / self.turnSpeed))
        self.stop()


    # Init Run method
    def run(self):
        # Move in a square, once
        
        # Go Forward
        print("Moving Forward")
        self.forward()
        # Then turn
        print("Turning")
        self.turn()
        # ... do it again

if __name__ == "__main__":
    node = DriveInSquare() 
    node.run()

