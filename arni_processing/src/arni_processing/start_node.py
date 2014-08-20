#!/usr/bin/env python
import rospy
from monitoring_node import MonitoringNode


def main():
    mn = MonitoringNode()
    mn.listener()

if __name__ == '__main__':
    rospy.init_node('monitoring_node', log_level=rospy.DEBUG)
    main()
