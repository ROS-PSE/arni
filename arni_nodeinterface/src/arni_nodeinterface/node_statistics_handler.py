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
        self.pub = rospy.Publisher('/statistics_node', NodeStatistics)
        self.update_intervall = rospy.rospy.get_param('/update_intervall')
        self.register_subscriber()
    
    def measure_status(self):
        """
        Collects information about the node's current status 
        using psutils and rospy.statistics
        Triggered periodically.
        """
        
        #CPU
        self._status.add_cpu_usage( self.__node_process.cpu_percent())

        self._status.add_cpu_usage_core(self.__cpu_usage_per_core())

        #RAM
        self._status.add_ram_usage( self.__node_process.memory_percent())

        #Disk I/O
        node_io = self.__node_process.io_counters()

        read_rate = node_io.read_bytes / self.update_intervall
        write_rate = node_io.write_bytes / self.update_intervall

        self._status.add_node_io(read_rate, write_rate)

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
        
        :topic: Topic to which the data should be published. 
        """
        
        self._status.time_end = rospy.Time.now()
        stats = self.__calc_statistics()
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

        ns = NodeStatistics()

        ns.node_cpu_usage_mean = stats_dict['cpu_usage_mean']
        ns.node_cpu_usage_stddev = stats_dict['cpu_usage_stddev']
        ns.node_cpu_usage_max = stats_dict['cpu_usage_max']

        ns.node_cpu_usage_core_mean = stats_dict['cpu_usage_core_mean']
        ns.node_cpu_usage_core_stddev = stats_dict['cpu_usage_core_stddev']
        ns.node_cpu_usage_core_max = stats_dict['cpu_usage_core_max']

        ns.node_ramusage_mean = stats_dict['ram_usage_mean']
        ns.node_ramusage_stddev = stats_dict['ram_usage_stddev']
        ns.node_ramusage_max = stats_dict['ram_usage_max']

        ns.node_message_frequency_mean = stats_dict['node_msg_frequency_mean']
        ns.node_message_frequency_stddev = stats_dict['node_msg_frequency_stddev']
        ns.node_message_frequency_max = stats_dict['node_msg_frequency_max']

        ns.node_bandwidth_mean = stats_dict['node_bandwidth_mean']
        ns.node_bandwidth_stddev = stats_dict['node_bandwidth_stddev']
        ns.node_bandwidth_max = stats_dict['node_bandwidth_max']
        
        ns.node_write_mean = stats_dict['node_write_mean']
        ns.node_write_stddev = stats_dict['node_write_stddev']
        ns.node_write_max = stats_dict['node_write_max']

        ns.node_read_mean = stats_dict['node_read_mean']
        ns.node_read_stddev = stats_dict['node_read_stddev']
        ns.node_read_max = stats_dict['node_read_max']

        return ns

    def receive_statistics(self, stats):
        """
        Receives the statistics published by ROS Topic statistics.
        """
        if self._id in stats.node_pub :
            dur = stats.window_stop - stats.window_start

            self._status.add_node_bandwidth(stats.traffic / dur)
                                            

            self._status.add_node_msg_freq(stats.period_mean)
        

    def __cpu_usage_per_core(self):
        """
        Reads the cpu usage in percent per core, using 'ps'. 
        Only works on linux.
        Most likely useless, as it's lifetime stats.
        """
        
        """Use ps to collect information about process, 
        psr is the id of the cpu pcpu usage in percent"""
        pipe = subprocess.Popen('ps -p %s -L -o psr,pcpu'%self.__node_process.pid, 
                                 shell = True, stdout = subprocess.PIPE).stdout
        output = pipe.read()

        #format output string Format ,where format_output[2i] is psr 
        #and format_output[2i+1] is corresponding pcpu
        format_output = output.split('\n',1)
        format_output = format_output[1:]
        format_output = format_output[0].split()


        cpu_usage = [0 for i in range(psutil.cpu_count())]

        for i in range(0, len(format_output)/2):
            psr = int(format_output[2*i])
            pcpu = float(format_output[2*i + 1])

            cpu_usage[psr] += pcpu

        return cpu_usage

    def get_pid(self):

        return self.__node_process.pid

    @property
    def id(self):
        return self._id
