#!/usr/bin/env python
import rospy
from host_statistics_handler import HostStatisticsHandler
import os


def main():

    ip = os.getenv('ROS_IP', '127.0.0.1')
    ip = ip.replace('.', '_')

    rospy.init_node('host_statistics_%s' % ip, log_level=rospy.DEBUG)
    rospy.sleep(1)
    host = HostStatisticsHandler(ip)
    try:
        rospy.sleep(1)
        rospy.Timer(rospy.Duration(host.update_intervall), host.measure_status)
        rospy.Timer(
            rospy.Duration(host.publish_intervall), host.publish_status)
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
