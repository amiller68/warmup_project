# warmup_project

## Drive in a square behaviour:

### High Level Description
The run method of my implementation carries out a single iteration of a loop that runs 4 times and instructs the robot to move forward 1m and turn left pi / 2 radians. At the end of this loop the robot has moved in a square and ends up back where it started. After the loop end, the robot is commanded to stop. My implmentation utilizes odometrey in order to coordinate the movements of the robot. 

### Code Explanation
I used one helper function
- getDistance(a, b):
	- This function returns the distance between two (x, y) coords a and b. 

The rest of my implementation is contained within the class DriveInSquare. It initializes the behavious' node, defines a publisher for the /cmd_vel topic, defines a subscriber for the /odom topic, sets the rate of the topic publishers, sets the speed of the robot, sets the side lenghts (1 m) and angles of the shape (pi / 2 rad) the robot should move around, initializes forward, turn, and stop commands for the robot to publish to /cmd_vel using the specified speeds, and initializes variables to hold the odometry state of the robot.
My DriveInSquare object implements the following functions:
- setPose(msg):
	- this function is used as Callback to the Odometry subscriber
	- It sets the odometry state of the object, by taking and recording the x, y, and orientation coordinates of the robot
- stop(duration):
	- this function publishes a stop command for 'duration' s
	- it does this in order to assure that the robot transitions to a stopped state
- moveForward():
	- this method first records a start position using the current odometry (x,y) position state
	- it then proceeds to publih the forward command to the /cmd_vel topic until the robot has moved the specified length of the square, in  m
	- It checks this by continually comparing `getDistance(star position, current position)` to `distance`
	- once its done moving the robot the method calls stop(1)
- turn():
	- this method first records a start rotation using the current odometry rotation state
	- it then proceeds to publih the turn command to the /cmd_vel topic until the robot has rotated the specified number of radians, in rad
	- It checks this by continually comparing `abs(start rotation, current rotation) ` to `radians`
	- once its done moving the robot the method calls stop(1)
- singleLoop():
	- This function uses  moveForward and turn to move the robot in a square
	- it implements this behaviour by calling moveForward(), turn() four times

- run():
	- This function sleeps until the odometry state of the object is initialized
	- it then runs singleLoop() once and then exits
### GIF
(gif)[./handin.gif]

## Challenges
The first challenge I faced when handling the robot was imprecision. My first approach was to transition my implementation from velocity duration based, to odometry based. The extra measurement radically improved my robots accuracy, however it still suffered issues. I realized I could fix this by adjusting the rate to reflect the data resolution I needed in my implementation. I settled on a rate of 20 Hz.

## Future Work
In the future I would take this basic class and implement a movement library to use in future projects. In order to do this I would operationalize the class and all its methods. This might come in handy for the final project.

## Takeaways
- When working with kinematics and asynchronous programming, its beneficial to implement checks on movements with odometry. Movind by speed and distance was often inaccurate.
- It's helpful to operationalize your methods as much as possible in the beginning, to make testing and experimentation easier later.
-
-
