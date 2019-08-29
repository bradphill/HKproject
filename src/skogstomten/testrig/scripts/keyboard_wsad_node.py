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

        self.x = 0 # left = -1, right = +1 # bör nog ändras?
        self.y = 0 # backward = +1, forward = -1

        # actions
        self.action = 4 # 4 = idle, changed from 5, see action_recalc below

        # publisher obj
        self.pub = rospy.Publisher('motor_action', Int64, queue_size=1) # publish message

        # loop rate
        rate = rospy.Rate(10) # checks if a button is pressed (10Hz)

        # kb listener setup
        with keyboard.Listener(on_press=lambda key: self.on_press(key), on_release=lambda key: self.on_release(key)) as listener:
            listener.join() # listens to keyboard inputs

    def on_press(self, key):
        try:
            print('')

            if key.char == 'q' or key == keyboard.Key.esc:
                exit()
            elif key.char == 'w':
                self.y = 1
                print('y = {0}'.format(self.y))
            elif key.char == 's':
                self.y = -1
                print('y = {0}'.format(self.y))
            elif key.char == 'a':
                self.x = -1
                print('x = {0}'.format(self.x))
            elif key.char == 'd':
                self.x = 1
                print('x = {0}'.format(self.x))

            self.action_recalc()  	# function below 
            self.publish() 		# same

        except AttributeError: 		# if not wsad
            pass

    # on button release set vaiables x and y to 0
    def on_release(self,key):  
        try:
            print('')

            if key.char == 'w' or key.char == 's':
                self.y = 0
            elif key.char == 'a' or key.char == 'd':
                self.x = 0

            self.action_recalc()
            self.publish()
        except AttributeError:
            pass

    def action_recalc(self):
        # actions - imagine a keypad:
        # 6 7 8     FL F FR
        # 3 4 5  =  L  I  R
        # 0 1 2     BL B BR
        # F = forward, B = backward, L = left, R = right, I = idle
        self.action = self.x + 3*self.y + 4 # see numbers on "keypad" and directions above
        print('action: {0}'.format(self.action))

    def publish(self):
        # action_msg is of type Int64
        action_msg = Int64() 
        # pack action
        action_msg.data = self.action
        # pub
        self.pub.publish(action_msg)

# If no key is pressed -> nothing/pass/nemas 
if __name__ == '__main__':
    try:
        kn = KeyboardWsadNode() 
    except rospy.ROSInterruptException:
        pass
