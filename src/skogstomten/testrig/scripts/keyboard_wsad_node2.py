#!/usr/bin/env python

## keyboard node to control the robot with WSAD
## publishes an action ID that is then taken by motor_controller_node
## printed feedback is a bit weird 
import rospy 					# to be able to use python code in ros
from std_msgs.msg import Int64 			# Datatype in published message (pub to motor_action)
from pynput import keyboard

class KeyboardWsadNode:
    def __init__(self):
        # node init 
        rospy.init_node('keyboard_wsad_node', anonymous=True)

        self.x = 0 # left = -1, right = +1 # bor nog andras?
        self.y = 0 # backward = +1, forward = -1
	self.pwm = 10;
	self.pendulumAction = 0;

        # actions
        self.action = 5 # 4 = idle, changed from 5, see action_recalc below

        # publisher obj
        self.pub1 = rospy.Publisher('motor_action', Int64, queue_size=1) # publish message
	self.pub2 = rospy.Publisher('motor_pwm',Int64, queue_size=1)
	self.pubPendel = rospy.Publisher('pendulum_action', Int64, queue_size = 1) #should be replaced with a custom message once the power supply and fuses have been upgraded (not in this project)

        # loop rate
        rate = rospy.Rate(10) # checks if a button is pressed (10Hz)

        # kb listener setup
        with keyboard.Listener(on_press=lambda key: self.on_press(key), on_release=lambda key: self.on_release(key)) as listener:
            listener.join() # listens to keyboard inputs

    def on_press(self, key):
	flag = 0
        try:
            print('')

