#!/usr/bin/env python
import rospy
from host_statistics_handler import *
import os

def main():
    
    #howto ROS_IP ?

    rospy.init_node("HostStatistics", log_level=rospy.DEBUG)
    rospy.sleep(1)
    ip = os.getenv('ROS_IP', '127.0.0.1')
    ip = ip.replace('.','_')

    host = HostStatisticsHandler(ip)

    try:
        rospy.sleep(1)
        rospy.Timer(rospy.Duration(host.update_intervall), host.measure_status)
        rospy.Timer(rospy.Duration(host.publish_intervall),host.publish_status)
        while not rospy.is_shutdown():
            rospy.spin()
        #rospy.on_shutdown(host.shut_down_hook)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__' : main()