import rospy
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
from arni_countermeasure.outcome import *
from arni_core.helper import *


def talker():
    pub = rospy.Publisher('statistics_rated', RatedStatistics, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    r = rospy.Rate(2)  # 10hz
    print "publishing.."
    while not rospy.is_shutdown():
        entity = create_statistic_entity(
            "cpu_usage_max", ["90"], ["0-50"], [Outcome.HIGH])

        msg = create_msg(
            "n%snode" % SEUID_DELIMITER, "127.0.0.1", [entity])
        pub.publish(msg)
        r.sleep()
    print "done."


def create_msg(seuid, host, entity):
    msg = RatedStatistics()
    msg.seuid = seuid
    msg.host = host

    now = rospy.Time.now()
    msg.window_start = now - rospy.Duration(1)
    msg.window_stop = now

    msg.rated_statistics_entity = entity
    return msg


def create_statistic_entity(s_type, actual, expected, state):
    msg = RatedStatisticsEntity()
    msg.statistic_type = s_type
    msg.actual_value = actual
    msg.expected_value = expected
    msg.state = state
    return msg

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
