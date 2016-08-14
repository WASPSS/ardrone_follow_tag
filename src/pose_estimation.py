#!/usr/bin/env python
import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import CameraInfo
import numpy as np


P = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
width_and_height = [0.0,0.0]
acc = [0,0,0]
acc_bias = [-0.0905859717205,   0.0330284067355,   0.0430109761953]
thetas = [0.0,0.0]
def callback(data):
    global acc
    #print "number of tags",data.tags_count
    acc[0] = data.ax+acc_bias[0]
    acc[1] = data.ay+acc_bias[1]
    acc[2] = data.az+acc_bias[2]

    thetas[0] = np.arctan(acc[0]/acc[2])
    thetas[1] = np.arctan(acc[1]/acc[2])

    if data.tags_count > 0:
        print
        for i in range(data.tags_count):
            if data.tags_type > 100000:
                estimate_X_and_Y(data.tags_xc[i],data.tags_yc[i],data.tags_distance[i])
                # x_displacement = data.tags_xc[i] - 500
                # y_displacement = data.tags_yc[i] - 500
                # print "tag_x", x_displacement
                # print "tag_y", y_displacement
                # print "tags_distance", data.tags_distance[i]
                # tag_orientation = (data.tags_orientation[i] + 90.0)
                # if tag_orientation >= 360:
                #     tag_orientation -= 360
                # print "tags_orientation", tag_orientation

def estimate_X_and_Y(xc,yc,zc):
    global P, width_and_height
    x = xc*width_and_height[0]/1000
    y = yc*width_and_height[1]/1000
    X = zc*(x-P[2])/P[0]
    Y = zc*(y-P[6])/P[5]
    print "X: ", X, " Y: ",Y

def camera_info_cb(data):
    global P, width_and_height
    P = data.P
    width_and_height[0] = data.width
    width_and_height[1] = data.height



# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    rospy.init_node('pose_estimation')
    #pub = rospy.Publisher('', Twist, queue_size=10)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)
    rospy.Subscriber("/ardrone/bottom/camera_info", CameraInfo, camera_info_cb)
    # starts the node

    rospy.spin()

if __name__ == '__main__':
    start()
