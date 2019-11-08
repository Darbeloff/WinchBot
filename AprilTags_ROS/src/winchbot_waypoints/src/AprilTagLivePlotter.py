import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
    #print("Listening...")
    for i in range(1):
        rospy.rostime.wallsleep(0.5)

def animate(i, xs, ys):

    # Read temperature (Celsius) from TMP102
    waypoint_listener()
    #end_effector_list = [[1,0,0],[2,0,1],[3,5,3]]
    #pos_list = end_effector_list
    end_effector = np.array(end_effector_list)
    end_effector = np.mean(end_effector,axis=0)
    #effector_rot = np.array(effector_rot_list)
    pos = np.array(pos_list)
    pos = np.mean(pos,axis=0)
    #pos_rot = np.array(pos_rot_list)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(end_effector[0])

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('End effector position over time')
    plt.ylabel('x/m')


if __name__ == '__main__':     
    # Create figure for plotting
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    xs = []
    ys = []

    # Set up plot to call animate() function periodically
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=500)
    plt.show()
