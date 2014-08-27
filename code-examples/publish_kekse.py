#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import time


def talker():
    pub = rospy.Publisher('/kekse13', String, queue_size=10)
    rospy.init_node('hufflepuff13')

    print "publishing.."

    # how much faster then reallity?

    # in ms
    timeout=0.1
    # in s
    while not rospy.is_shutdown():
        pub.publish("blabla")
        rospy.sleep(timeout)
    print "done."

if __name__ == '__main__':
    talker()
