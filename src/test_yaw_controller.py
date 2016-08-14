#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float64
import message_filters
import numpy as np
import tf

run_controller = 0
setpoint_yaw = 0

def joy_cb(data):
    global run_controller
    if(data.buttons[5]==1):
        run_controller = 1
    else:
        run_controller=0

def cb_x(x_cont):
    global twist_cmd
    twist_cmd.linear.x = x_cont.data

def cb_y(y_cont):
    global twist_cmd
    twist_cmd.linear.y = y_cont.data

# Intializes everything
def start():
    global cmd_pub, twist_cmd
    # publishing to "turtle1/cmd_vel" to control
    rospy.init_node('yaw_controller')
    x_state_pub = rospy.Publisher('/x_controller/state', Float64, queue_size=1)
    y_state_pub = rospy.Publisher('/y_controller/state', Float64, queue_size=1)
    x_setpoint_pub = rospy.Publisher('/x_controller/setpoint', Float64, queue_size=1)
    y_setpoint_pub = rospy.Publisher('/y_controller/setpoint', Float64, queue_size=1)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/joy", Joy, joy_cb)
    rospy.Subscriber('/x_controller/control_effort', Float64, cb_x)
    rospy.Subscriber('/y_controller/control_effort', Float64, cb_y)


    tf_listener = tf.TransformListener()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if(run_controller==1):
            twist_cmd = Twist()
            try:
                (trans,rot) = tf_listener.lookupTransform('/tag', '/odom', rospy.Time(0))
                x_state_pub.publish(trans[0])
                y_state_pub.publish(trans[1])
                x_setpoint_pub.publish(0.0)
                y_setpoint_pub.publish(0.0)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            cmd_pub.publish(twist_cmd)
        rate.sleep()



if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
