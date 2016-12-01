#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    #capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    #capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw * (180.0/math.pi)
    #compute distance to target
    distance = math.sqrt(math.pow((desiredX - xPosition), 2) + math.pow((desiredY - yPosition), 2))
    adjustedX = goal.pose.position.x - xPosition
    adjustedY = goal.pose.position.y - yPosition
    print goal.pose.position.x, goal.pose.position.y
    print xPosition, yPosition
    print adjustedX, adjustedY
    #compute initial turn amount
    initialTurn = (math.atan2(adjustedY, adjustedX) * (180 / math.pi)) - theta

    print "moving from (" + str(xPosition) + ", " + str(yPosition) + ") @ " + str(theta) + " degrees"
    print "moving to (" + str(desiredX) + ", " + str(desiredY) + ") @ " + str(desiredT) + " degrees"
    print "distance: " + str(distance) + ", initial turn: " + str(initialTurn)
    print "spin!" #turn to calculated angle
    rotateDegrees(initialTurn)
    print "move!" #move in straight line specified distance to new pose
    driveSmooth(0.25, distance)
    rospy.sleep(2)
    print "spin!" #spin to final angle
    finalTurn = desiredT - theta
    print "rotate " + str(finalTurn) +  " to " + str(desiredT)
    rotateDegrees(finalTurn)
    print "done" 


#This function sequentially calls methods to perform a trajectory.
#def executeTrajectory():
#    pass  # Delete this 'pass' once implemented


    #spinWheels(20,20,5)
    #driveStraight(2, 5)

# Initialize set up of the functions, take in inputs and then uses methods within to calculate speed, angles and velocities
# and then begins to output them while updating information from the sensors thorughout the robot



#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    #pass  # Delete this 'pass' once implemented

 #Set X and y, this sets one wheel to each veleocity and then uses the time given to set how long the wheels  are supposed to run


    #wheel diameter = 76 mm = 0.076 m
    #wheel radius d/2 = 0.038 m
    WheelRadius = 0.038
    #wheel base = 25 cm = 0.25 m
    WheelBase = 0.25

    #v (m/s) = wheelradius (m) * Angular Velocity(u1 or u2) (rads/sec)

    v1 = WheelRadius * u1

    v2 = WheelRadius * u2

    avgVel = (v1 + v2)/(2.)

    # (Vr - Vl)/wheel base
    angVel = (v2 - v1)/WheelBase

    start = rospy.Time().now().secs

    
    #use HW1 to get u1 and U2 into LinVel and AngVel
    #publishTwist(linearVelocity, angularVelocity)
    #publishTwist(avgVel, angVel)

    
    while((rospy.Time().now().secs - start) < time and not rospy.is_shutdown()):
        #publishTwist(linearVelocity, angularVelocity)
        publishTwist(avgVel, angVel)
    
    #Stop
    publishTwist(0, 0)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    #pass  # Delete this 'pass' once implemented

# Set the x and y directions, move forward, while moving check distace to see if you've hit your goal. If you hit your goal stop moving
    global pose
    pose = Pose()

    initx = pose.position.x 
    inity = pose.position.y

    thereYet = False

    currDistance = 0

    while (not thereYet and not rospy.is_shutdown()):
        print "going"
        currx = pose.position.x
        curry = pose.position.y
        print currx
        print curry

        currDistance = math.sqrt((currx - initx)**2 + (curry - inity)**2)
        print currDistance
        if (currDistance >= distance):
            thereYet = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
            #rospy.sleep(0.05)
            print "Still going"




    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
#    pass  # Delete this 'pass' once implemented

# set x and y angle, spin wheels, while spinning check to see if you have the correct angle, if you do, stop moving
    global odom_list
    global pose
    pose = Pose()

    if(angle > 180 or angle < -180):
        print "angle is too large or small"
    
    angvel = Twist()
    done = False

    if(angle < 0):
        publishTwist(0,-.5)
    else:
        publishTwist(0, .5)

    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))

    roll, pitch, yaw = euler_from_quaternion(rot)
    
    state = math.degrees(yaw)
    print "state"    
    print state

    goal = state + angle 


    while(not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        
        roll, pitch, yaw = euler_from_quaternion(rot)
        theta = math.degrees(yaw)
        print "theta"
        print theta

        print "goal"
        print goal

        if(angle > 0):
            if (goal <= theta):
                publishTwist(0, 0)
                done = True
                print "I turned"
            else:
                if(angle < 0):
                    publishTwist(0,-.5)
                else:
                    publishTwist(0, .5)

        else:
            if (goal >= theta):
                publishTwist(0, 0)
                done = True
                print "I turned"
            else:
                if(angle < 0):
                    publishTwist(0,-.5)
                else:
                    publishTwist(0, .5)
    angvel.angular.z = 0.0
    pub.publish(angvel)









def rampedriveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a ramped straight line."""
    global pose

    initX = pose.position.x
    initY = pose.position.y
    thereYet = False
    rampSpeed = 0.0
    sleepFor = 0.05
    rampPercent = 0.3
    step = speed / ((rampPercent * (distance / speed)) / sleepFor)

    while (not thereYet and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initX), 2) + math.pow((currentY - initY), 2))
        if (currentDistance >= distance):
            thereYet = True
            publishTwist(0, 0)

        else:
            if ((distance - currentDistance) <= distance * rampPercent and rampSpeed >= 0):
                rampSpeed -= step
                publishTwist(rampSpeed, 0)
            elif ((distance - currentDistance) >= distance * (1.0 - rampPercent) and rampSpeed <= speed):
                rampSpeed += step
                publishTwist(rampSpeed, 0)
            else:
                publishTwist(speed, 0)
            rospy.sleep(sleepFor)











#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented

# this initalizes x and y, then it calculates the radius of the arc needed from the distance to the gaol,
# then by going at the set speed, it moves along the arc while using the angle to help determine the specific wheel speed of the robot
# (If wheel speeds are equal then it will move forward straight)



#Bumper Event Callback function
def readBumper(msg):
    """Bumper event callback"""
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
    
        #pass  # Delete this 'pass' once implemented

        # When pressed the wheels will immediatley stop moving and will not move until the button is no longer being pressed
        print "Bumper pressed!"
        executeTrajectory()



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.

# Set up a timer loop herer to repeeat the desiered code every X seconds/ticks

# Start the timer with the following line of code: 
    rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    pose = Pose()
    global xPosition
    global yPosition
    global theta

    bumper_sub = rospy.Subscriber(
        'mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1)


    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0))

    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    print position
    xPosition=position[0]
    yPosition=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)

    pose.orientation.z = yaw
    theta = math.degrees(yaw)

    #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    

    #pass # Delete this 'pass' once implemented


def publishTwist(linearVelocity, angularVelocity):
    """Send a movement (twist) message."""
    global pub

    msg = Twist()

    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)
    pass







# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node_ajkrasa')

    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    pose = Pose()
    global odom_tf
    global odom_list

    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1)
                # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    rospy.Timer(rospy.Duration(.01), timerCallback)
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))


    nav_part = rospy.Subscriber('/move_base_simple/goal', PoseStamped, navToPose, queue_size=10)


    print "Starting Lab 2"
    #publishTwist(linearVelocity, angularVelocity):
    #publishTwist(2, 0)

    #spinWheels(u1, u2, time):
    #spinWheels(10,10,5)

    #driveStraight(speed, distance):
    #driveStraight(.1, .5)

    #def rotate(angle)
    #rotate(45)

    rampedriveStraight(.1, 1)  


    #executeTrajectory()


    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(1), timerCallback)

    # Make the robot do stuff...

    print "Lab 2 complete!"

