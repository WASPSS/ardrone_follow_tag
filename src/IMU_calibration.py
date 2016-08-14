#!/usr/bin/env python
import rospy
from ardrone_autonomy.msg import Navdata
import numpy as np


acc = [0.0,0.0,0.0]
acc_bias = [0.0,0.0,0.0]
i = 0

def callback(data):
    global acc, acc_bias, i
    #print "number of tags",data.tags_count
    acc[0] = data.ax
    acc[1] = data.ay
    acc[2] = data.az
    if i < 1000:
        print "calibration step: ", i
        acc_bias[0] += acc[0]
        acc_bias[1] += acc[1]
        acc_bias[2] += acc[2]
        i += 1
        print 0-acc_bias[0]/i, " ",0-acc_bias[1]/i, " ",1-acc_bias[2]/i

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    rospy.init_node('pose_estimation')
    #pub = rospy.Publisher('', Twist, queue_size=10)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)
    # starts the node

    rospy.spin()

if __name__ == '__main__':
    start()
