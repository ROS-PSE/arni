from status import Status
from collections import namedtuple
import rospy

statistic_tuple = namedtuple('statistic', ['mean', 'stddev', 'max'])


class NodeStatus(Status):

    """
    Extension of Status , to store additional information used by nodes.
    """

    def __init__(self, start):

        super(NodeStatus, self).__init__(start)

        #: Network bandwidth used by the node in bytes.
        self.__node_bandwidth = {}

        #: Bytes read from disk by node.
        self.__node_read = []

        #: Bytes written to disk by node.
        self.__node_write = []

        #: Frequency of network calls by node.
        self.__node_msg_frequency = []
        self.last_write_update = rospy.Time.now()
        self.last_read_update = rospy.Time.now()

    def add_node_bandwidth(self, topic, bytes):
        """
        Adds another measured value in bytes, taken
        from ROS topics statistics, to node_bandwidth.

        :param bytes: Bytes measured.
        :type bytes: int
        """
        if topic not in self.__node_bandwidth:
            self.__node_bandwidth[topic] = []

        self.__node_bandwidth[topic].append(bytes)

    def add_node_write(self, write):
        """
        Adds another measured value to node_write

        :param write: Bytes written.
        :type write: int
        """
        t = rospy.Time.now()
        delta_t = (t - self.last_write_update).to_sec()
        write_rate = float(write) / delta_t
        self.__node_write.append(write_rate)
        self.last_write_update = t

    def add_node_read(self, read):
        """
        Adds another measured value to node_read

        :param read: Bytes read.
        :type read: int
        """
        t = rospy.Time.now()
        delta_t = (t - self.last_read_update).to_sec()
        read_rate = float(read) / delta_t
        self.__node_write.append(read_rate)
        self.last_read_update = t

    def add_node_msg_freq(self, freq):
        """
        Adds another measured value to node_msg_frequency,
        taken from ROS topics statistics.

        :param freq: frequency of network calls.
        :type bytes: int
        """
        self.__node_msg_frequency.append(freq)

    def reset_specific(self):
        """
        Resets the values specific to Host or Nodes
        """

        self.__node_bandwidth.clear()
        del self.__node_read[:]
        del self.__node_write[:]
        del self.__node_msg_frequency[:]

    def calc_stats_specific(self):
        """
        calculates statistics specific to nodes.
        and write them into the stats_dict.
        """

        self.__calc_net_stats()
        self.__calc_drive_stats()
        self.__rename_keys()

    def __calc_net_stats(self):
        """
        Calculate net I/O statistics for a node
        """
        bw = [self.calc_stat_tuple(
            self.__node_bandwidth[key]).mean for key in self.__node_bandwidth]
        node_bandwidth = self.calc_stat_tuple(bw)
        node_msg_frequency = self.calc_stat_tuple(self.__node_msg_frequency)

        self._stats_dict[
            'node_message_frequency_mean'] = node_msg_frequency.mean
        self._stats_dict[
            'node_message_frequency_stddev'] = node_msg_frequency.stddev
        self._stats_dict['node_message_frequency_max'] = node_msg_frequency.max

        self._stats_dict['node_bandwidth_mean'] = sum(bw)
        self._stats_dict['node_bandwidth_stddev'] = node_bandwidth.stddev
        self._stats_dict['node_bandwidth_max'] = node_bandwidth.max

    def __calc_drive_stats(self):
        """
        Calculate drive I/O statistics for a node.
        """
        node_read = self.calc_stat_tuple(self.__node_read)
        node_write = self.calc_stat_tuple(self.__node_write)

        self._stats_dict['node_read_mean'] = node_read.mean
        self._stats_dict['node_read_stddev'] = node_read.stddev
        self._stats_dict['node_read_max'] = node_read.max

        self._stats_dict['node_write_mean'] = node_write.mean
        self._stats_dict['node_write_stddev'] = node_write.stddev
        self._stats_dict['node_write_max'] = node_write.max

    def __rename_keys(self):
        """
        rename keys in statistics dictionary , prepending node_ prefix
        to fit NodeStatistics message field names.
        """
        statistic = ['mean', 'stddev', 'max']

        for i in statistic:
            self._stats_dict['node_cpu_usage_%s' %
                             i] = self._stats_dict.pop('cpu_usage_%s' % i)
            self._stats_dict['node_cpu_usage_core_%s' %
                             i] = self._stats_dict.pop('cpu_usage_core_%s' % i)
            self._stats_dict['node_ramusage_%s' %
                             i] = self._stats_dict.pop('ram_usage_%s' % i)
            self._stats_dict['node_gpu_usage_%s' %
                             i] = self._stats_dict.pop('gpu_usage_%s' % i)

    @property
    def node_bandwidth(self):
        return self.__node_bandwidth

    @property
    def node_read(self):
        return self.__node_read

    @property
    def node_write(self):
        return self.__node_write

    @property
    def node_msg_frequency(self):
        return self.__node_msg_frequency

    @node_bandwidth.setter
    def node_bandwidth(self, value):
        self.__node_bandwidth = value

    @node_read.setter
    def node_read(self, value):
        self.__node_read = value

    @node_write.setter
    def node_write(self, value):
        self.__node_write = value

    @node_msg_frequency.setter
    def node_msg_frequency(self, value):
        self.__node_msg_frequency = value
