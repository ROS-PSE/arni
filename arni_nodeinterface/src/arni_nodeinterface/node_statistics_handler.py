from statistics_handler import StatisticsHandler
from node_status import NodeStatus
from arni_msgs.msg import NodeStatistics
from rosgraph_msgs.msg import TopicStatistics
import psutil
import subprocess
import rospy


class NodeStatisticsHandler(StatisticsHandler):

    """
    Holds the statistics of an individual Node.
    """

    def __init__(self, host_id, node_id, node_process):

        super(NodeStatisticsHandler, self).__init__(node_id)

        #:identifier of this node
        self._id = node_id

        #:Host this node runs on
        self.__host_id = host_id

        #: Status of the node
        self._status = NodeStatus(rospy.Time.now())

        self.__node_process = node_process
        self.pub = rospy.Publisher('/statistics_node', NodeStatistics, queue_size=2)
        self.update_interval = rospy.get_param('~publish_interval', 10) /\
            float(rospy.get_param('~window_max_elements', 10))
        self.register_subscriber()
        self.__write_base = self.__node_process.io_counters().write_bytes
        self.__read_base = self.__node_process.io_counters().read_bytes

    def measure_status(self):
        """
        Collects information about the node's current status
        using psutils and rospy.statistics
        Triggered periodically.
        """
        try:
            # CPU
            self._status.add_cpu_usage(self.__node_process.cpu_percent())

            self._status.add_cpu_usage_core(self.__cpu_usage_per_core())

            # RAM
            self._status.add_ram_usage(self.__node_process.memory_percent())

            # Disk I/O
            node_io = self.__node_process.io_counters()

            delta_write = node_io.write_bytes - self.__write_base
            if delta_write != 0:
                self._status.add_node_write(delta_write)
                self.__write_base = node_io.write_bytes
            delta_read = node_io.read_bytes - self.__read_base
            if delta_read != 0:
                self._status.add_node_read(delta_read)
                self.__read_base = node_io.read_bytes
        except psutil.NoSuchProcess:
            pass

    def register_subscriber(self):
        """
        Register the subscriber for Topicstatistics
        """
        rospy.Subscriber("/statistics", TopicStatistics,
                         self.receive_statistics)

    def publish_status(self):
        """
        Publishes the current status to a topic using ROS's
        publisher-subscriber mechanism. Triggered periodically.
        """
        self._status.time_end = rospy.Time.now()
        stats = self.__calc_statistics()
        #rospy.logdebug('Publishing Node Status %s' % self._id)
        self.pub.publish(stats)
        self._status.reset()
        self._status.time_start = rospy.Time.now()

    def __calc_statistics(self):
        """
        Calculates statistics like mean, standard deviation
        and max from the status.
        Returns an instance of HostStatistics which can be published.

        :returns: NodeStatistics
        """
        stats_dict = self._status.calc_stats()

        node_status = NodeStatistics()
        node_status.host = self.__host_id
        node_status.node = self._id
        node_status.window_start = self._status.time_start
        node_status.window_stop = self._status.time_end
        for v in dir(node_status):
            if v in stats_dict:
                setattr(node_status, v, stats_dict[v])

        return node_status

    def receive_statistics(self, stats):
        """
        Receives the statistics published by ROS Topic statistics
        and attemps to calculate node net I/O stats with them
        """
        if self._id in stats.node_pub:
            dur = stats.window_stop - stats.window_start
            if dur.to_sec() != 0:
                self._status.add_node_bandwidth(stats.topic , stats.traffic)
            self._status.add_node_msg_freq(stats.period_mean.to_sec())

    def __cpu_usage_per_core(self):
        """
        Reads the cpu usage in percent per core, using 'ps'.
        Only works on linux.
        Most likely useless, as it's lifetime stats.
        """

        """Use ps to collect information about process,
        psr is the id of the cpu pcpu usage in percent"""
        pipe = subprocess.Popen(
            'ps -p %s -L -o psr,pcpu' % self.__node_process.pid,
            shell=True, stdout=subprocess.PIPE).stdout
        output = pipe.read()

        # format output string Format ,where format_output[2i] is psr
        # and format_output[2i+1] is corresponding pcpu
        format_output = output.split('\n', 1)
        format_output = format_output[1:]
        format_output = format_output[0].split()

        cpu_usage = [0 for i in range(psutil.cpu_count())]

        for i in range(0, len(format_output) / 2):
            psr = int(format_output[2 * i])
            pcpu = float(format_output[2 * i + 1])

            cpu_usage[psr] += pcpu

        return cpu_usage

    def get_pid(self):
        return self.__node_process.pid

    @property
    def id(self):
        return self._id

    @property
    def node_process(self):
        return self.__node_process

    @property
    def status(self):
        return self._status
