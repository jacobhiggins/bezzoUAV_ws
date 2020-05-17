#!/usr/bin/env python

# Takes keyboard inputs (numbers 1-6) and publishes joystick commands for sim
# To use, run this node along with: rosrun keyboard keyboard
# Use keyboard input from keyboard package to send joy commands to sim
import rospy
from sensor_msgs.msg import Joy
from keyboard.msg import Key

# b1 = 1 key, b2 = 2 key, etc.
# Descriptors of what happens in sim also included
b1 = 0 # Trajector 1, press 1
b2 = 0 # Land, press 2
b3 = 0 # Take off, press 3
b4 = 0 # Trajectory 2, press 4
b5 = 0 # Unmapped
b6 = 0 # Attack (?), press 6

def buttonUp_cb(data):
	global b1,b2,b3,b4,b5
	# print("Up button")
	if data.code == data.KEY_1:
		b1 = 0
	elif data.code == data.KEY_2:
		b2 = 0
	elif data.code == data.KEY_3:
		b3 = 0
	elif data.code == data.KEY_4:
		b4 = 0
	elif data.code == data.KEY_5:
		b5 = 0
	return

def buttonDown_cb(data):
	global b1,b2,b3,b4,b5
	# print("Down button")
	if data.code == data.KEY_1:
		b1 = 1
	elif data.code == data.KEY_2:
		b2 = 1
	elif data.code == data.KEY_3:
		b3 = 1
	elif data.code == data.KEY_4:
		b4 = 1
	elif data.code == data.KEY_5:
		b5 = 1
	return
	

def main():
	pub = rospy.Publisher('/joy',Joy,queue_size=10)
	subUp = rospy.Subscriber('/keyboard/keyup',Key,buttonUp_cb)
	subDown = rospy.Subscriber('/keyboard/keydown',Key,buttonDown_cb)
	rospy.init_node('my_joynode')
	rate = rospy.Rate(10)
	joy_msg = Joy()
	joy_msg.buttons = [0,0,0,0,0]
	while not rospy.is_shutdown():
		joy_msg.buttons = [b1,b2,b3,b4,b5]
		# print(joy_msg.buttons)
		pub.publish(joy_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
