#!/usr/bin/env python

import rospy
from arni_countermeasure.countermeasure_node import *


def main():
    cn = CountermeasureNode()
#    rospy.loginfo(rospy.get_caller_id() + ": im on ")

    cn.loop()


if __name__ == '__main__':
    main()
