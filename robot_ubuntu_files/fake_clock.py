#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Clock
import time

def fake_clock_publisher():
    rospy.init_node('fake_clock_publisher', anonymous=True)
    pub = rospy.Publisher('/clock', Clock, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = time.time()

    while not rospy.is_shutdown():
        elapsed = time.time() - start_time
        clock_msg = Clock()
        clock_msg.clock = rospy.Time.from_sec(elapsed)
        pub.publish(clock_msg)
        rate.sleep()

if __name__ == '__main__':
    fake_clock_publisher()
