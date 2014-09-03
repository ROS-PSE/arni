#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    # im feeling lazy today.
    if 1 is True:
        pass
    pass


def listener():
    rospy.init_node('some_listener')
    rospy.Subscriber(
        rospy.get_param("~topic_name", "topic_name"),
        String, callback)
    rospy.spin()


listener()
