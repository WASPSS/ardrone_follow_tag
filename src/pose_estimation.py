#!/usr/bin/env python
import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf


P = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
width_and_height = [0.0,0.0]
acc = [0,0,0]
acc_bias = [-0.0905859717205,   0.0330284067355,   0.0430109761953]
thetas = [0.0,0.0]
def callback(data):
    global acc, pub
    #print "number of tags",data.tags_count
    acc[0] = data.ax+acc_bias[0]
    acc[1] = data.ay+acc_bias[1]
    acc[2] = data.az+acc_bias[2]

    thetas[0] = np.arctan(acc[0]/acc[2])
    thetas[1] = np.arctan(acc[1]/acc[2])

    if data.tags_count > 0:
        for i in range(data.tags_count):
            if data.tags_type > 100000:
                [alphaX,alphaY] = estimate_alphaX_and_alphaY(data.tags_xc[i],data.tags_yc[i],data.tags_distance[i])
                cZ =  data.tags_distance[i]
                dX = cZ*np.sin(thetas[1]-alphaX)
                dY = -cZ*np.sin(thetas[0]-alphaY)

                tag_orientation = (data.tags_orientation[i] + 90.0)
                if tag_orientation >= 360:
                    tag_orientation -= 360

                br = tf.TransformBroadcaster()
                br.sendTransform((dX/100, dY/100, cZ/100),
                     tf.transformations.quaternion_from_euler(0, 0, math.radians(tag_orientation)),
                     rospy.Time.now(),
                     "odom",
                     "tag")
                # msg = Twist()
                # msg.linear.x = dX
                # msg.linear.y = dY
                # msg.linear.z = 0
                # msg.angular.x = 0
                # msg.angular.y = 0
                # msg.angular.z = tag_orientation
                # pub.publish(msg)


def estimate_alphaX_and_alphaY(xc,yc,zc):
    global P, width_and_height
    x = xc*width_and_height[0]/1000
    y = yc*width_and_height[1]/1000
    X = zc*(x-P[2])/P[0]
    Y = zc*(y-P[6])/P[5]
    return [np.arctan(X/zc),np.arctan(Y/zc)]

def estimate_roll_pitch(ax,ay,az):
    return[np.arctan(ax/az),np.arctan(ay/az)]

def camera_info_cb(data):
    global P, width_and_height
    P = data.P
    width_and_height[0] = data.width
    width_and_height[1] = data.height



# Intializes everything
def start():
    global pub
    # publishing to "turtle1/cmd_vel" to control turtle1
    rospy.init_node('pose_estimation')
    pub = rospy.Publisher('tag_delta', Twist, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)
    rospy.Subscriber("/ardrone/bottom/camera_info", CameraInfo, camera_info_cb)
    # starts the node

    rospy.spin()

if __name__ == '__main__':
    start()
