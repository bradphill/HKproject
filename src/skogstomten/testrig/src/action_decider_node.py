#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Point

##-----------------------------DESCRIPTION---------------------------------##
#Plans action..... 
#
#Subscribes to "cmd_goal": 
#
#Subscribes to "set_pose":
#
#
#Publishes "motor_action_sw":
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('action_decider')
pubAction = rospy.Publisher('motor_action_sw', Int64, queue_size = 2)
rate = rospy.Rate(10)

xg, yg = [0, 0] #init goal in origin
xp, yp, yaw = [0, 0, 0] #init pose as 0, 0 ,0
##-----------------------------INIT---------------------------------##


##----------------CONFIGURATION PARAMETERS----------------##
#all measurements in mm and seen from top plane   
L = 500 #distance between the two waists
d_f = 235 #distance between front waist and front wheel axle
d_m = 285 #distance between front waist and middle wheel axle
d_r = 235 #distance between front waist and rear wheel axle
allowedTwist = [0, 45] #maximum twist of waists
tolerance = 0.01 #percentage based error tolerance for acceptable turn radius
iterationLimit = 1500 #maximum amount of iterations
MAX_TURN_RADIUS = 4000 #mm
MIN_TURN_RADIUS = 500 #mm
turnRate = 10 #mm/increment
##--------------------------------------------------------##
def setGoalCallback(goal):
    xg, yg = [goal.x, goal.y]

def setPoseCallback(pose)
    xp, yp, yaw = [pose.x, pose.y, pose.z]

def decideOnAction():
    action_msg = Int64()
    pubAction.publish(action_msg)
    pass      


def doStuff():
	action = decideOnAction() #positive angles mean that the testrig should turn clockwise and negative counter-clockwise
	#print("Front angle: " + str(frontAngle) + " Rear Angle: " + str(rearAngle))
	angles_msg = Point()
	angles_msg.x = frontAngle
	angles_msg.y = rearAngle
	angles_msg.z = R_des
	pubAngles.publish(angles_msg)
	

def main():

	subGoal = rospy.Subscriber('cmd_goal', point, setGoalCallback)
        subpose = rospy.Subscriber('set_pose', point, setPoseCallback)
	while not rospy.is_shutdown():
		doStuff()
		rate.sleep()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
