#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import String

pub = rospy.Publisher('topic_name', String, queue_size=10)


def floored_abs():
    counter = 1
    step_size = 5000
    maximum = 20000
    distance = maximum - counter
    duration = rospy.get_param("~duration", 10)
    time_step = distance / duration
    direction = 1
    while not rospy.is_shutdown():
        c = math.floor(counter / step_size) + 1
        counter += direction * time_step
        if counter >= maximum:
            direction = -1
        if direction < 0 and counter == 1:
            direction = 1
        c *= 1000
        print(c)
        pub.publish(("%s times, " % int(c)) + "publishing with floored_abs. " * int(c))
        rospy.Rate(rospy.get_param("~frequency", 10)).sleep()


def linear_abs():
    counter = 1
    maximum = 20000
    distance = maximum - counter
    duration = rospy.get_param("~duration", 10)
    time_step = distance / duration
    direction = 1
    while not rospy.is_shutdown():
        counter += direction * time_step
        print(counter)
        if counter >= maximum:
            direction = -1
        if direction < 0 and counter == 1:
            direction = 1
        pub.publish(("%s times, " % int(math.floor(counter))) + "publishing with linear_abs. " * int(math.floor(counter)))
        rospy.Rate(rospy.get_param("~frequency", 10)).sleep()


def sawtooth():
    high = rospy.get_param("~bandwidth_high", 200)
    low = rospy.get_param("~bandwidth_low", 1)
    distance = high - low
    steps = rospy.get_param("~steps", distance)
    step = distance / steps
    counter = 1
    while not rospy.is_shutdown():
        pub.publish("publishing with sawtooth. " * int(counter))
        counter += step
        if counter >= high:
            counter = 1
        rospy.Rate(rospy.get_param("~frequency", 10)).sleep()


def sine():
    mid = rospy.get_param("~bandwidth_mid", 100)
    var = rospy.get_param("~bandwidth_variation", 5000)
    counter = 0
    while not rospy.is_shutdown():
        c = math.sin(counter / 180 * math.pi)
        v = mid + c * var
        pub.publish("publishing with sine. " * int(v))
        counter += 1
        rospy.Rate(rospy.get_param("~frequency", 10)).sleep()


def constant():
    while not rospy.is_shutdown():
        pub.publish("publishing with constant. " * 100)
        rospy.Rate(rospy.get_param("~frequency", 10)).sleep()


def stop_publish():
    start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() - start < rospy.Duration(rospy.get_param("~timeout", 30)):
        pub.publish("publishing until it stops. " * int(random.randrange(100, 150)))
        rospy.Rate(rospy.get_param("~frequency", 10)).sleep()


'''
~mode = ("constant", "stop_publish", "sawtooth", "sine", "floored_abs", "linear_abs")
~frequency = 10

stop_publish:
  # stops random publishing after timeout seconds
  ~timeout = 30
sawtooth:
  # publishes more and more data then resets to bandwidth_low
  ~bandwidth_high = 200
  ~bandwidth_low = 1
  ~steps = bandwidth_high - bandwidth_low   # steps to go from low to high
sine:
  # publishes data in a sinus curve
  ~bandwidth_mid = 100
  ~bandwidth_variation = 5000    # sinus' amplitude
floored_abs:
  # publishes a floor(abs(x))-shape
linear_abs:
  # publishes a abs(x)-shape

'''
if __name__ == '__main__':
    rospy.init_node('node_name', log_level=rospy.DEBUG)
    mode = rospy.get_param("~mode", "constant")
    if mode == "constant":
        constant()
    elif mode == "stop_publish":
        stop_publish()
    elif mode == "sawtooth":
        sawtooth()
    elif mode == "sine":
        sine()
    elif mode == "floored_abs":
        floored_abs()
    elif mode == "linear_abs":
        linear_abs()
