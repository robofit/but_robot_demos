#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

publisher = 0

def callback(data):
    if data is None or not data.data:
        return
    data.data.strip()
    rospy.loginfo(rospy.get_caller_id() + " received: " + data.data)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0     
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    if data.data == "pee are two go front" or data.data == "pee are two go forward" or data.data == "pee are two move front" or data.data == "pee are two move forward":
        twist.linear.x = 0.5
    elif data.data == "pee are two go back" or data.data == "pee are two go backward" or data.data == "pee are two move back" or data.data == "pee are two move backward":
        twist.linear.x = -0.5
    elif data.data == "pee are two turn left" or data.data == "pee are two rotate left":
        twist.angular.z = math.pi / 4
    elif data.data == "pee are two turn right" or data.data == "pee are two rotate right":
        twist.angular.z = -math.pi / 4
    else:
         print ("unknown - '" + data.data + "'")
    for i in range(200):
        publisher.publish(twist)
        rospy.sleep(0.01)

def listen():
    rospy.init_node('example', anonymous=True)
    global publisher
    publisher = rospy.Publisher('/base_controller/command', Twist)	# or /cmd_vel
    rospy.Subscriber("/voice_commands", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
