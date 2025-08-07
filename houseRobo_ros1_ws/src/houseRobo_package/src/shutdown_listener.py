#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess

def callback(msg):
    if msg.data.lower() == "shutdown":
        rospy.loginfo("Shutdown command received!")
        subprocess.call(["/home/ubuntu/save_and_shutdown.sh"])

def listener():
    rospy.init_node('shutdown_listener')
    rospy.Subscriber('/robot_command', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
