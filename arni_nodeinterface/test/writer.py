#!/usr/bin/env python
import psutil
import time
import rospy

class Hosti(object):

    def __init__(self):
        super(Hosti, self).__init__()
        rospy.init_node('writer' )


    def write_test(self, event):
        with open("writetest.txt", "ab") as wf:
            wf.write("x"* 4096)


def main():

    h = Hosti()
    #rospy.Timer(rospy.Duration(60),h.diff)
    rospy.Timer(rospy.Duration(1), h.write_test )
    rospy.spin()

if __name__ == '__main__': main()