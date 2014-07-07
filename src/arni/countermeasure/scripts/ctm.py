#!/usr/bin/env python

import rospy
from arni_msgs.msg import RatedStatistics


def ctm():

    rospy.init_node('ctm')

    rospy.Subscriber("statistics_rated", RatedStatistics)