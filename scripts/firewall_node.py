#!/usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

speed_limit=0

def speed_cb(msg):
    global speed_limit
    speed_limit=msg.data


def vel_cb(vel):
    global speed_limit
    enable_car.publish(Empty())
    if speed_limit and speed_limit!=0:
        if vel.linear.x>speed_limit:
            vel.linear.x=speed_limit
            rospy.logwarn("too fast!")
            velocity_pub.publish(vel)
        else:
            rospy.logwarn("speed fine")
            velocity_pub.publish(vel)
    else:
        rospy.logwarn("no speed limit")
        velocity_pub.publish(vel)



if __name__ == '__main__':
    rospy.init_node('firewall', anonymous=False)
    enable_car = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
    velocity_pub = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)


    #from the pi

    rospy.Subscriber('/speed_limit', Int32, speed_cb)

    #from the car
    rospy.Subscriber('/test_vel',Twist,vel_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass