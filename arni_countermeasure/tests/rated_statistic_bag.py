import rosbag
import rospy
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
from arni_core.helper import SEUID_DELIMITER


def create_simple_bag():
    bag = rosbag.Bag('bags/john.bag', 'w')
    msg = RatedStatistics()
    msg.seuid = "n" + SEUID_DELIMITER + "node1"
    msg.host = "host1"

    now = rospy.Time.now()
    msg.window_start = now - rospy.Duration(5)
    msg.window_stop = now

    msg.rated_statistics_entity = [create_statistic_entity()]
    try:
        bag.write('/statistics_rated', msg)
    finally:
        bag.close()


def create_statistic_entity():
    msg = RatedStatisticsEntity()
    msg.statistic_type = "cpu_usage_mean"
    msg.actual_value = ["56.6"]
    msg.expected_value = ["0 - 50"]
    msg.state = [0]
    return msg


rospy.init_node("temp_node")
create_simple_bag()
