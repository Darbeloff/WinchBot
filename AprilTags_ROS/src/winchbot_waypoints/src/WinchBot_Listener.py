#!/usr/bin/env python3
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

# Quaternion Manipulation, source: https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion
def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = math.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

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

def q_to_axisangle(q):
    w, v = q[0], q[1:]
    theta = math.acos(w) * 2.0
    return normalize(v), math.degrees(theta)

def axisangle_to_q(v, theta):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = math.cos(theta)
    x = x * math.sin(theta)
    y = y * math.sin(theta)
    z = z * math.sin(theta)
    return w, x, y, z

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
	for i in range(2):
		rospy.rostime.wallsleep(0.5)

def winchbot_IK(winches,platform):
	q = [0,0,0]
	N0 = platform[0]
	N1 = platform[1]
	N2 = platform[2]
	winch0 = winches[0]
	winch1 = winches[1]
	winch2 = winches[2]
	q[0] = math.sqrt((N0 - winch0[0])**2 + (N1-(winch0[1]+0.4531))**2 + (N2-winch0[2])**2)
	q[1] = math.sqrt((N0 - (winch1[0]-0.866))**2 + (N1-(winch1[1]-0.5))**2 + (N2-winch1[2])**2)
	q[2] = math.sqrt((N0 - (winch2[0]+0.866))**2 + (N1-(winch2[1]-0.5))**2 + (N2-winch2[2])**2)
	return q

if __name__ == '__main__':
	pos = None
	waypoint_listener()
	end_effector = np.array(end_effector_list)
	end_effector = np.mean(end_effector,axis=0)
	effector_rot = np.array(effector_rot_list)
	effector_rot = np.mean(effector_rot,axis=0)
	cam_to_lead = np.array([0, -3.38, -4.25]) # Conversion from Camera position to Lead Screw
	#goal_pos = np.array([pos.x*39.3701,pos.y*39.3701,pos.z*39.3701]) - cam_to_lead
	#print('Goal:')
	#print(goal_pos)
	# CALIBRATION INACCURATE!!!
	calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
	platform_centre = np.array((qv_mult(tuple(effector_rot),(0,0,2))))*-1
	platform_cam = calibration + platform_centre # Platform centre pos relative to Camera
	global platform
	platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
	# Find initial cable lengths:
	zp = -1.125
	winches = [[0,20.1875,zp],[-14.2747, -14.2747, zp], [14.2747, -14.2747, zp]]
	qi = [0,0,0]
	qi = winchbot_IK(winches,platform)
	e_resolution = 400000 # Number of encoder counts in 1 revolution
	drum_diam = 4.125 # Drum diameter in inches
	while True:
		print('Current location is x='+str(platform[0])+' y='+str(platform[1])+' z='+str(platform[2]))
		print('Input Command:')
		destination_input = input()
		if destination_input == 'exit':
			break
		elif destination_input == 'listen once':
				waypoint_listener()
				end_effector = np.array(end_effector_list)
				end_effector = np.mean(end_effector,axis=0)
				effector_rot = np.array(effector_rot_list)
				effector_rot = np.mean(effector_rot,axis=0)
				cam_to_lead = np.array([0, -3.38, -4.25]) # Conversion from Camera position to Lead Screw
				#goal_pos = np.array([pos.x*39.3701,pos.y*39.3701,pos.z*39.3701]) - cam_to_lead
				#print('Goal:')
				#print(goal_pos)
				# CALIBRATION INACCURATE!!!
				calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
				platform_centre = np.array((qv_mult(tuple(effector_rot),(0,0,2))))*-1
				platform_cam = calibration + platform_centre # Platform centre pos relative to Camera
				platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
				print(platform)
		elif destination_input == 'endeffectorrotation':
			while True:
				try:
					waypoint_listener()
					end_effector = np.array(end_effector_list)
					end_effector = np.mean(end_effector,axis=0)
					effector_rot = np.array(effector_rot_list)
					effector_rot = np.mean(effector_rot,axis=0)
					euler = q_to_axisangle(effector_rot)
					print(euler)
				except KeyboardInterrupt:
					break
				except IndexError:
					break
		elif destination_input == 'winchoffset':
			while True:
				try:
					waypoint_listener()
					end_effector = np.array(end_effector_list)
					end_effector = np.mean(end_effector,axis=0)
					effector_rot = np.array(effector_rot_list)
					effector_rot = np.mean(effector_rot,axis=0)
					winch0_offset = np.array((qv_mult(tuple(effector_rot),(0,1,0))))
					winch1_offset = np.array((qv_mult(tuple(effector_rot),(-0.866,-0.5,0))))
					winch2_offset = np.array((qv_mult(tuple(effector_rot),(0.866,-0.5,0))))
					print('Winch0 Offset: '+str(winch0_offset))
					print('Winch1 Offset: '+str(winch1_offset))
					print('Winch2 Offset: '+str(winch2_offset))
					time.sleep(0.5)
				except KeyboardInterrupt:
					break
				except IndexError:
					break
				except TypeError:
					break
		elif destination_input == 'listen':
			while True:
				try:
					waypoint_listener()
					end_effector = np.array(end_effector_list)
					end_effector = np.mean(end_effector,axis=0)
					effector_rot = np.array(effector_rot_list)
					effector_rot = np.mean(effector_rot,axis=0)
					cam_to_lead = np.array([0, -3.38, -4.25]) # Conversion from Camera position to Lead Screw
					#goal_pos = np.array([pos.x*39.3701,pos.y*39.3701,pos.z*39.3701]) - cam_to_lead
					#print('Goal:')
					#print(goal_pos)
					# CALIBRATION INACCURATE!!!
					calibration = np.array([end_effector[0]*39.3701,end_effector[1]*39.3701,end_effector[2]*39.3701]) # Calibration AprilTag pos relative to Camera
					platform_centre = np.array((qv_mult(tuple(effector_rot),(0,0,2))))*-1
					platform_cam = calibration + platform_centre # Platform centre pos relative to Camera
					platform = platform_cam - cam_to_lead # Platform centre pos relative to Lead Screw
					print(platform)
				except KeyboardInterrupt:
					break
				except IndexError:
					break
		else:
			print('Invalid input. Please try again.')
			time.sleep(1)