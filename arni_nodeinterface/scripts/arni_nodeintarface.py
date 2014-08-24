#!/usr/bin/env python
import rospy
from host_statistics_handler import *

def main():
    
    #howto ROS_IP ?
    host = HostStatisticsHandler('localhost')

    try:
        rospy.sleep(1)
        rospy.Timer(rospy.Duration(host.update_intervall), host.measure_status)
        rospy.Timer(rospy.Duration(host.publish_intervall),host.publish_status)
        while not rospy.is_shutdown():
            rospy.spin()
        rospy.on_shutdown(host.shut_down_hook)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__' : main()