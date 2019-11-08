#!/usr/bin/env python3
# Source: https://github.com/madcowswe/ODrive/blob/master/tools/odrive_demo.py
# This version of WinchBot Controller attempts feedback by updating the end effector position in the inverse kinematics.
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
import matplotlib.pyplot as plt

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
	for i in range(10):
		rospy.rostime.wallsleep(1)

if __name__ == '__main__':
	waypoint_listener()
	#end_effector_list = [[1,0,0],[2,0,1],[3,5,3]]
	#pos_list = end_effector_list
	end_effector = np.array(end_effector_list)
	#effector_rot = np.array(effector_rot_list)
	pos = np.array(pos_list)
	#pos_rot = np.array(pos_rot_list)
	plt.figure(1)
	plt.suptitle('End effector position over time')
	plt.subplot(211)
	plt.plot(end_effector[:,0],end_effector[:,1],'ro')
	plt.xlabel('x/m')
	plt.ylabel('y/m')
	plt.subplot(212)
	plt.plot(end_effector[:,0],end_effector[:,2],'ro')
	plt.ylabel('z/m')
	plt.xlabel('x/m')

	plt.figure(2)
	plt.suptitle('Target position over time')
	plt.subplot(211)
	plt.plot(pos[:,0],pos[:,1],'ro')
	plt.xlabel('x/m')
	plt.ylabel('y/m')
	plt.subplot(212)
	plt.plot(pos[:,0],pos[:,2],'ro')
	plt.xlabel('x/m')
	plt.ylabel('z/m')
	plt.show()
