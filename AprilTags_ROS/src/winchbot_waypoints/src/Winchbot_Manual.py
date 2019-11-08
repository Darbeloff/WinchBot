#!/usr/bin/env python3
# Source: https://github.com/madcowswe/ODrive/blob/master/tools/odrive_demo.py
''' This code subscribes to the transform information coming from the apriltags_ROS /tf function. Setting the vector location of the apriltag (with offset from the hook) as the goal, a series of waypoints is generated to move the end effector into position. This involves lifting the winch to a certain z height to clear obstacles, then moving to the correct x,y location, and then lowering. (Details on actual hooking tbd, maybe elevate before recording goal to ensure out of the way of camera)'''
from __future__ import print_function

#import odrive
#from odrive.enums import *
import time
import math
import roslib
import rospy
import numpy as np
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge, CvBridgeError
#import tf2_ros
import geometry_msgs.msg
import odrive
from odrive.enums import *

def callback(data):
	global pos
	global end_effector
	tf_list = data.transforms
	tf_stamped = tf_list[0]
	tf = tf_stamped.transform
	tag = tf_stamped.child_frame_id
	if tag == "tag_0":
		pos = tf.translation
	elif tag == "tag_1":
		end_effector = tf.translation

	#print(pos) '''print statements for debugging'''
	#print(tf)

# Define a subscriber node that takes a the apriltag position as input
def waypoint_listener():
	rospy.init_node('waypoint_listener',anonymous=True)
	rospy.Subscriber("/tf",TFMessage,callback)
	print("Listening...")
	for i in range(5):
		rospy.rostime.wallsleep(0.5)

def odrive_move(drive,axis,delta):
	'''moves the selected odrive by using position control, changes encoder value by delta'''
	if axis == 0:
		target = drive.axis0.encoder.pos_estimate + delta
		print(target)
		drive.axis0.controller.pos_setpoint = target
		print("Moving axis0...")
		#print(drive.axis0.encoder.pos_estimate)
		#time.sleep(10)
		#print(drive.axis0.encoder.pos_estimate)
	elif axis == 1:
		target = drive.axis1.encoder.pos_estimate + delta
		print(target)
		drive.axis1.controller.pos_setpoint = target
		print("Moving axis1...")
		#print(drive.axis1.encoder.pos_estimate)
		#time.sleep(10)
		#print(drive.axis1.encoder.pos_estimate)

def winchbot_IK(winches,platform):
	q = [0,0,0]
	N0 = platform[0]
	N1 = platform[1]
	N2 = platform[2]
	winch0 = winches[0]
	winch1 = winches[1]
	winch2 = winches[2]
	q[0] = math.sqrt((N0 - winch0[0])**2 + (N1-winch0[1])**2 + (N2-winch0[2])**2)
	q[1] = math.sqrt((N0 - winch1[0])**2 + (N1-winch1[1])**2 + (N2-winch1[2])**2)
	q[2] = math.sqrt((N0 - winch2[0])**2 + (N1-winch2[1])**2 + (N2-winch2[2])**2)
	return q



if __name__ == '__main__':
	# Find a connected ODrive (this will block until you connect one)
	print("finding an odrive...")
	odrv0 = odrive.find_any(serial_number="2061377C3548")
	print("odrv0 found!")
	odrv1 = odrive.find_any(serial_number="2087377E3548")
	print("odrv1 found!")
	all_drives = [odrv0,odrv1]

	# Calibrate motor and wait for it to finish
	print("starting calibration...")
	my_drive = odrv0
	#my_drive.axis0.controller.config.vel_limit =20000 # set velocity limit
	'''my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
	while my_drive.axis0.current_state != AXIS_STATE_IDLE:
	    time.sleep(0.1)'''

	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	my_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	print('Drive 1 calibrated!')

	my_drive = odrv1
	#my_drive.axis0.controller.config.vel_limit =20000 # set velocity limit
	'''my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
	while my_drive.axis0.current_state != AXIS_STATE_IDLE:
	    time.sleep(0.1)'''

	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	my_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	print('Drive 2 calibrated!')
	#my_drive.axis1.controller.config.vel_limit =20000
	'''my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
	while my_drive.axis1.current_state != AXIS_STATE_IDLE:
	    time.sleep(0.1)'''

	my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	my_drive.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	print('Drive 3 calibrated!')

	# Begin listening for AprilTag location
	pos = [0, 0, 0]
	waypoint_listener()
	print(end_effector)
	print(pos)
	# Convert from camera coords to lead screw coords:
	cam_to_lead = np.array([0, 2.75, -4.25]) # Conversion from Camera position to Lead Screw
	goal_pos = np.array([pos.x*39.3701,pos.y*39.3701,pos.z*39.3701]) - cam_to_lead
	print('Goal:')
	print(goal_pos)
	# CALIBRATION INACCURATE!!!
	calibration = np.array([end_effector.x*39.3701,end_effector.y*39.3701,end_effector.z*39.3701]) # Calibration AprilTag pos relative to Camera
	platform_cam = calibration + np.array([0, 1.07, 0]) # Platform centre pos relative to Camera
	platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
	# Find initial cable lengths:
	zp = -1.125
	winches = [[0,-20.1875,zp],[14.2747, 14.2747, zp], [-14.2747, 14.2747, zp]]
	qi = [0,0,0]
	N0 = platform[0]
	N1 = platform[1]
	N2 = platform[2]
	winch0 = winches[0]
	winch1 = winches[1]
	winch2 = winches[2]
	qi[0] = math.sqrt((N0 - winch0[0])**2 + (N1-winch0[1])**2 + (N2-winch0[2])**2)
	qi[1] = math.sqrt((N0 - winch1[0])**2 + (N1-winch1[1])**2 + (N2-winch1[2])**2)
	qi[2] = math.sqrt((N0 - winch2[0])**2 + (N1-winch2[1])**2 + (N2-winch2[2])**2)
	#qi=[39,38,35] #For testing
	#pub = rospy.Publisher('position', Vector3,queue_size=10) # Vector3 is not defined
	rate = rospy.Rate(10) # 10Hz
	goal_reached = 0
	stage = 0
	e_resolution = 400000 # Number of encoder counts in 1 revolution
	drum_diam = 4.125 # Drum diameter in inches
	while not rospy.is_shutdown():
		#pub.publish(pos)
		print('qi=')
		print(qi)
		print('Current location is x='+str(platform[0])+' y='+str(platform[1])+' z='+str(platform[2]))
		print('Input destination as [x,y,z]:')
		destination_input = input()
		if destination_input == 'exit':
			break
		# Converting string to list 
		destination_string = destination_input.strip('][').split(',')
		destination = [float(i) for i in destination_string]
		q = winchbot_IK(winches, destination)
		print('q=')
		print(q)
		q = np.array(q)
		qi = np.array(qi)
		dq = q - qi
		de = (dq/(np.pi*drum_diam))*e_resolution
		print('de=')
		print(de)
		qi = q
		odrive_move(all_drives[0],0,de[0])
		odrive_move(all_drives[1],0,de[1])
		odrive_move(all_drives[1],1,de[2])
		print("Moving...")
		time.sleep(5)
		platform = destination
		print("Moved.")
