#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock
import time


def talker():
    pub = rospy.Publisher('clock', Clock, queue_size=10)
    rospy.init_node('clock_server', anonymous=True)

    print "publishing.."

    # how much faster then reallity?
    mult_factor = rospy.get_param("/ctm/clock_mult", 10)

    # in ms
    rate = 10
    # in s
    cur_time = 10000.0
    while not rospy.is_shutdown():
        c = Clock()
        c.clock = rospy.Time.from_sec(cur_time)
        pub.publish(c)
        time.sleep(rate / 1000.0)
        cur_time += rate / 1000.0 * mult_factor
    print "done."

if __name__ == '__main__':
    talker()
