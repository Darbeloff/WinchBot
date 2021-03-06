#!/usr/bin/env python3
# This version of WinchBot Controller attempts feedback by updating the goal.
# Problem with the y position in pickup.
# Collies with vertical portion of hook in pickup.
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
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Quaternion Manipulation, source: https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion
def q_mult(q1, q2):
	w1, x1, y1, z1 = q1
	w2, x2, y2, z2 = q2
	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
	return w, x, y, z

def q_conjugate(q):
	w, x, y, z = q
	return (w, -x, -y, -z)

def qv_mult(q1, v1):
	q2 = (0.0,) + v1
	return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def callback(data):
	global pos_list
	global pos_rot_list
	global end_effector_list
	global effector_rot_list
	tf_list = data.transforms
	tf_stamped = tf_list[0]
	tf = tf_stamped.transform
	tag = tf_stamped.child_frame_id
	if tag == "tag_0":
		pos_list.append([tf.translation.x,tf.translation.y,tf.translation.z])
		pos_rot_list.append([tf.rotation.w,tf.rotation.x, tf.rotation.y, tf.rotation.z])
		#pos_rot = tf.transformations.euler_from_quaternion(tf.rotation)
	elif tag == "tag_1":
		end_effector_list.append([tf.translation.x, tf.translation.y, tf.translation.z])
		effector_rot_list.append([tf.rotation.w,tf.rotation.x, tf.rotation.y, tf.rotation.z])
		#effector_rot = tf.transformations.euler_from_quaternion(tf.rotation)

	#print(pos) '''print statements for debugging'''
	#print(tf)

# Define a subscriber node that takes a the apriltag position as input
def waypoint_listener():
	global pos_list
	global pos_rot_list
	global end_effector_list
	global effector_rot_list
	pos_list = []
	pos_rot_list = []
	end_effector_list = []
	effector_rot_list = []
	rospy.init_node('waypoint_listener',anonymous=True)
	rospy.Subscriber("/tf",TFMessage,callback)
	print("Listening...")
	for i in range(5):
		rospy.rostime.wallsleep(0.5)

def odrive_move(drive,axis,delta):
	'''moves the selected odrive by using position control, changes encoder value by delta'''
	if axis == 0:
		target = drive.axis0.encoder.pos_estimate + delta
		#print(target)
		drive.axis0.controller.pos_setpoint = target
		#print("Moving axis0...")
		#print(drive.axis0.encoder.pos_estimate)
		#time.sleep(10)
		#print(drive.axis0.encoder.pos_estimate)
	elif axis == 1:
		target = drive.axis1.encoder.pos_estimate + delta
		#print(target)
		drive.axis1.controller.pos_setpoint = target
		#print("Moving axis1...")
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

def pickup():
	global end_effector
	global platform
	time.sleep(5)
	waypoint_listener()
	end_effector = np.array(end_effector_list)
	end_effector = np.mean(end_effector,axis=0)
	effector_rot = np.array(effector_rot_list)
	effector_rot = np.mean(effector_rot,axis=0)
	pos = np.array(pos_list)
	pos = np.mean(pos,axis=0)
	pos_rot = np.array(pos_rot_list)
	pos_rot = np.mean(pos_rot,axis=0)
	print(pos_rot)
	#print(pos)
	#print(pos_rot)
	# Convert from camera coords to lead screw coords:
	cam_to_lead = np.array([0, -3.38, -4.25]) # Conversion from Camera position to Lead Screw
	hook_centre = np.array((qv_mult(tuple(pos_rot),(0,-2.4,0))))*-1 # displacement from AprilTag to hook
	#print(hook_centre)
	print([pos[0]*39.3701,pos[1]*39.3701,pos[2]*39.3701])
	print([pos[0]*39.3701,pos[1]*39.3701,pos[2]*39.3701] - cam_to_lead)
	print([pos[0]*39.3701,pos[1]*39.3701,pos[2]*39.3701] - cam_to_lead + hook_centre)
	print(hook_centre)
	goal_pos = np.array([pos[0]*39.3701,pos[1]*39.3701,pos[2]*39.3701]) + hook_centre - cam_to_lead
	goal_pos_temp = goal_pos
	print('Goal:')
	print(goal_pos)
	# CALIBRATION INACCURATE!!!
	calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
	platform_centre = np.array((qv_mult(tuple(effector_rot),(0,0,2))))*-1
	platform_cam = calibration + platform_centre # Platform centre pos relative to Camera
	#print('Platform Centre:')
	#print(platform_centre)
	platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
	# Find initial cable lengths:
	zp = -1.125
	winches = [[0,20.1875,zp],[-14.2747, -14.2747, zp], [14.2747, -14.2747, zp]]
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
	safe_z = 35 # safe Z coordinate for translation over the object
	x_offset = 0
	y_offset = -4.5
	y_offset2 = 0
	z_offset = -3.5
	while goal_reached==0:
		#pub.publish(pos)
		if stage == 0:
			target = [goal_pos[0]+x_offset,goal_pos[1]+y_offset,safe_z]
			q = winchbot_IK(winches, target)
			print('Target:')
			print(target)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 1:
			target = [goal_pos[0]+x_offset,goal_pos[1]+y_offset,goal_pos[2]+z_offset]
			q = winchbot_IK(winches, target)
			print('target=')
			print(target)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 2:
			end_effector = 'none'
			time.sleep(5)
			waypoint_listener()
			end_effector = np.array(end_effector_list)
			end_effector = np.mean(end_effector,axis=0)
			effector_rot = np.array(effector_rot_list)
			effector_rot = np.mean(effector_rot,axis=0)
			pos = np.array(pos_list)
			pos = np.mean(pos,axis=0)
			pos_rot = np.array(pos_rot_list)
			pos_rot = np.mean(pos_rot,axis=0)
			if end_effector == 'none':
				print('Failed to detect end effector!')
				stage = 0
				continue
			calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
			hook_centre = np.array((qv_mult(tuple(effector_rot),(0,0,4))))*-1
			print('hook centre:')
			print(hook_centre)
			calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
			hook_cam = calibration + hook_centre # Platform centre pos relative to Camera
			hook_offset = [goal_pos[0]+x_offset-hook_cam[0],goal_pos[1]+y_offset-hook_cam[1]]
			print('hook_offset:')
			print(hook_offset)
			if abs(hook_offset[0])>1:
				increment = 2
				#hook_offset = [hook_offset[0]/2,hook_offset[1]/2]
				print('hook_offset:')
				print(hook_offset)
			else:
				increment = 3
			platform_centre = np.array((qv_mult((effector_rot[0],effector_rot[1],effector_rot[2],effector_rot[3]),(0,0,2))))*-1
			platform_cam = calibration + platform_centre # Platform centre pos relative to Camera
			#print('Platform Centre:')
			#print(platform_centre)
			platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
			print('Platform Centre:')
			print(platform_centre)
			hook = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
			#target = [platform[0]+hook_offset[0],platform[1]+hook_offset[1],goal_pos[2]+z_offset]
			target = [goal_pos_temp[0]+x_offset+hook_offset[0],goal_pos_temp[1]+y_offset+hook_offset[1],goal_pos_temp[2]+z_offset]
			goal_pos_temp = [goal_pos_temp[0]+hook_offset[0],goal_pos_temp[1]+hook_offset[1],goal_pos_temp[2]]
			q = winchbot_IK(winches, target)
			print('target=')
			print(target)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = target
			stage=increment
			# Look for residual error
			'''time.sleep(5)
			waypoint_listener()
			calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
			hook_centre = np.array((qv_mult(tuple(effector_rot),(0,0,4))))
			calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
			hook_cam = calibration + hook_centre # Platform centre pos relative to Camera
			hook_offset = [goal_pos[0]+x_offset-hook_cam[0],goal_pos[1]+y_offset-hook_cam[1]]
			print("Residual error:")
			print(hook_offset)'''
		if stage == 3:
			#target = [platform[0]+hook_offset[0],platform[1]+hook_offset[1]+y_offset2,goal_pos[2]+z_offset]
			target = [goal_pos_temp[0]+x_offset+hook_offset[0],goal_pos_temp[1]+y_offset2,goal_pos_temp[2]+z_offset] # removed hook offset for y
			q = winchbot_IK(winches, target)
			print('target=')
			print(target)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 4:
			target = [goal_pos_temp[0],goal_pos_temp[1],safe_z]
			q = winchbot_IK(winches, target)
			#print('q=')
			#print(q)
			print('target=')
			print(target)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			#print('de=')
			#print(de)
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 5:
			target = [0,0,safe_z]
			q = winchbot_IK(winches, target)
			#print('q=')
			#print(q)
			print('target=')
			print(target)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			#print('de=')
			#print(de)
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
			goal_reached=1
		if goal_reached:
			print("Goal reached!")

def animate_error(i, xs, ys, ys2):

	# Read temperature (Celsius) from TMP102
	waypoint_listener()    
	end_effector = np.array(end_effector_list)
	end_effector = np.mean(end_effector,axis=0)
	effector_rot = np.array(effector_rot_list)
	effector_rot = np.mean(effector_rot,axis=0)
	#pos = np.array(pos_list)
	#pos = np.mean(pos,axis=0)
	#pos_rot = np.array(pos_rot)
	#pos_rot = np.mean(pos_rot,axis=0)

	calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
	hook_centre = np.array((qv_mult(tuple(effector_rot),(0,0,4))))
	calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
	hook_cam = calibration + hook_centre # Platform centre pos relative to Camera
	error = [platform[0]-hook_cam[0],platform[1]-hook_cam[1]]

	# Add x and y to lists
	xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
	ys.append(error[0])
	ys2.append(error[1])

	# Limit x and y lists to 20 items
	xs = xs[-20:]
	ys = ys[-20:]
	ys2 = ys2[-20:]

	# Draw x and y lists
	ax.clear()
	ax.plot(xs, ys, xs, ys2)

	# Format plot
	plt.xticks(rotation=45, ha='right')
	plt.subplots_adjust(bottom=0.30)
	plt.title('End effector position error')
	plt.ylabel('error in m')



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
	pos = None
	waypoint_listener()
	end_effector = np.array(end_effector_list)
	end_effector = np.mean(end_effector,axis=0)
	effector_rot = np.array(effector_rot_list)
	effector_rot = np.mean(effector_rot,axis=0)
	#pos = np.array(pos_list)
	#pos = np.mean(pos,axis=0)
	#pos_rot = np.array(pos_rot)
	#pos_rot = np.mean(pos_rot,axis=0)
	print(end_effector)
	print(effector_rot)
	#print(pos)
	#print(pos_rot)
	if pos is None:
		print('No target found!')
	# Convert from camera coords to lead screw coords:
	cam_to_lead = np.array([0, -3.38, -4.25]) # Conversion from Camera position to Lead Screw
	#goal_pos = np.array([pos.x*39.3701,pos.y*39.3701,pos.z*39.3701]) - cam_to_lead
	#print('Goal:')
	#print(goal_pos)
	# CALIBRATION INACCURATE!!!
	calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
	platform_centre = np.array((qv_mult(tuple(effector_rot),(0,0,2))))*-1
	platform_cam = calibration + platform_centre # Platform centre pos relative to Camera
	print('Platform Centre:')
	print(platform_centre)
	global platform
	platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
	# Find initial cable lengths:
	zp = -1.125
	winches = [[0,20.1875,zp],[-14.2747, -14.2747, zp], [14.2747, -14.2747, zp]]
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
	'''while goal_reached==0:
		#pub.publish(pos)
		if stage == 0:
			target = [goal_pos[0]+x_offset,goal_pos[1]+y_offset,safe_z]
			q = winchbot_IK(winches, target)
			print('Target:')
			print(target)
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
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 1:
			target = [goal_pos[0]+x_offset,goal_pos[1]+y_offset,goal_pos[2]+z_offset]
			q = winchbot_IK(winches, target)
			print('q=')
			print(q)
			print('target=')
			print(target)
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
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 2:
			target = [goal_pos[0]+x_offset,goal_pos[1],goal_pos[2]+z_offset]
			q = winchbot_IK(winches, target)
			print('q=')
			print(q)
			print('target=')
			print(target)
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
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 3:
			target = [goal_pos[0],goal_pos[1],safe_z]
			q = winchbot_IK(winches, target)
			print('q=')
			print(q)
			print('target=')
			print(target)
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
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
		if stage == 4:
			target = [0,0,safe_z]
			q = winchbot_IK(winches, target)
			print('q=')
			print(q)
			print('target=')
			print(target)
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
			platform = target
			print("Stage "+str(stage)+" complete.")
			stage+=1
			goal_reached=1
		if goal_reached:
			print("Goal reached!")'''

	while True:
		print('Current location is x='+str(platform[0])+' y='+str(platform[1])+' z='+str(platform[2]))
		print('Input destination as [x,y,z]:')
		destination_input = input()
		if destination_input == 'exit':
			break
		elif destination_input == 'pickup':
			# Move out of the way of camera
			'''destination = [4,0,10]
			q = winchbot_IK(winches, destination)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = destination
			print("Moved.")'''
			# Pick up object with hook
			pickup()
		else:
			# Converting string to list 
			destination_string = destination_input.strip('][').split(',')
			destination = [float(i) for i in destination_string]
			q = winchbot_IK(winches, destination)
			q = np.array(q)
			qi = np.array(qi)
			dq = q - qi
			de = (dq/(np.pi*drum_diam))*e_resolution
			qi = q
			odrive_move(all_drives[0],0,de[0])
			odrive_move(all_drives[1],0,de[1])
			odrive_move(all_drives[1],1,de[2])
			print("Moving...")
			time.sleep(5)
			platform = destination
			print("Moved.")
				# Create figure for plotting
			fig = plt.figure()
			ax = fig.add_subplot(1, 1, 1)
			xs = []
			ys = []
			ys2 = []
			# Set up plot to call animate() function periodically
			ani = animation.FuncAnimation(fig, animate_error, fargs=(xs, ys, ys2), interval=500)
			plt.show()




	'''# Find a connected ODrive (this will block until you connect one)
	print("finding an odrive...")
	my_drive = odrive.find_any()

	# Calibrate motor and wait for it to finish
	print("starting calibration...")
	my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
	while my_drive.axis0.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	# To read a value, simply read the property
	print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

	# Or to change a value, just assign to the property
	my_drive.axis0.controller.pos_setpoint = 3.14
	print("Position setpoint is " + str(my_drive.axis0.controller.pos_setpoint))

	# And this is how function calls are done:
	for i in [1,2,3,4]:
		print('voltage on GPIO{} is {} Volt'.format(i, my_drive.get_adc_voltage(i)))

	# A sine wave to test
	t0 = time.monotonic()
	while True:
		setpoint = 10000.0 * math.sin((time.monotonic() - t0)*2)
		print("goto " + str(int(setpoint)))
		my_drive.axis0.controller.pos_setpoint = setpoint
		time.sleep(0.01)
		'''
