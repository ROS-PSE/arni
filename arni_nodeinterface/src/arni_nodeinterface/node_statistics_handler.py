from statistics_handler import StatisticsHandler
from node_status import NodeStatus
import psutil
import subprocess



class NodeStatisticsHandler(StatisticsHandler):

    """
    Holds the statistics of an individual Node.
    """
    
    def __init__(self, host_id, node_id, node_process):
    
        super(NodeStatisticsHandler, self).__init__()
        
        #:identifier of this node
        self._id = node_id
        
        #:Host this node runs on
        self.__host_id = host_id
        
        #: Status of the node
        self._status = NodeStatus()
        
        self.__node_process = node_process
    
    def measure_status(self):
        """
        Collects information about the node's current status 
        using psutils and rospy.statistics
        Triggered periodically.
        """
        
        #CPU
        self._status.add_cpu( self.__node_process.cpu_percent())

        self._status.add_cpu_core(self.__cpu_usage_per_core())

        #RAM
        self._status.add_ram( self.__node_process.memory_percent())

        #Disk I/O
        node_io = self.__node_process.io_counters()

        read_rate = node_io.read_bytes / update_intervall
        write_rate = node_io.write_bytes / update_intervall

        self._status.add_node_io(read_rate, write_rate)

        #todo network stats



        
    def publish_status(self, topic):
        """
        Publishes the current status to a topic using ROS's publisher-subscriber mechanism.
        Triggered periodically.
        
        :topic: Topic to which the data should be published. 
        """
        pass
        
    def __calc_statistics(self):
        """
        Calculates statistics like mean, standard deviation and max from the status.
        Returns an instance of HostStatistics which can be published.
        
        :returns: NodeStatistics 
        """
        stats_dict = self._status.calc_stats()

        node_statistics = NodeStatistics()

        node_statistics.node_cpu_usage_mean = stats_dict['cpu_usage'].mean
        node_statistics.node_cpu_usage_stddev = stats_dict['cpu_usage'].stddev
        node_statistics.node_cpu_usage_max = stats_dict['cpu_usage'].max

        node_statistics.node_cpu_usage_core_mean = [stats_dict['cpu_usage_core'][i].mean for i in self._cpu_count]
        node_statistics.node_cpu_usage_core_stddev = [stats_dict['cpu_usage_core'][i].stddev for i in self._cpu_count]
        node_statistics.node_cpu_usage_core_max = [stats_dict['cpu_usage_core'][i].max for i in self._cpu_count]

        node_statistics.node_ramusage_mean = stats_dict['ram_usage'].mean
        node_statistics.node_ramusage_stddev = stats_dict['ram_usage'].stddev
        node_statistics.node_ramusage_max = stats_dict['ram_usage'].max

        node_statistics.node_message_frequency_mean = stats_dict['node_msg_frequency'].mean
        node_statistics.node_message_frequency_stddev = stats_dict['node_msg_frequency'].stddev
        node_statistics.node_message_frequency_max = stats_dict['node_msg_frequency'].max

        node_statistics.node_bandwidth_mean = stats_dict['node_bandwidth'].mean
        node_statistics.node_bandwidth_stddev = stats_dict['node_bandwidth'].stddev
        node_statistics.node_bandwidth_max = stats_dict['node_bandwidth'].max
        
        node_statistics.node_write_mean = stats_dict['node_write'].mean
        node_statistics.node_write_stddev = stats_dict['node_write'].stddev
        node_statistics.node_write_max = stats_dict['node_write'].max

        node_statistics.node_read_mean = stats_dict['node_read'].mean
        node_statistics.node_read_stddev = stats_dict['node_read'].stddev
        node_statistics.node_read_max = stats_dict['node_read'].max

        return node_statistics

    def __receive_statistics(self):
        """
        Receives the statistics published by ROS Topic statistics.
        """
        
        pass

    def __cpu_usage_per_core(self):
        """
        Reads the cpu usage in percent per core, using 'ps'. Only works on linux.
        Most likely useless, as it's lifetime stats.
        """
        
        #Use ps to collect information about process, psr is the id of the cpu pcpu usage in percent
        pipe = subprocess.Popen('ps -p ' + str(self.__node_process.pid()) + ' -L -o psr,pcpu').stdout
        output = pipe.read()

        #format output string Format , where format_output[2i] is psr and format_output[2i+1] is corresponding pcpu
        format_output = output.split('\n',1)
        format_output = format_output[1:]
        format_output = format_output[0].split()


        cpu_usage = [0 for i in psutil.cpu_count()]

        for i in range(0, len(format_output)/2):
            psr = int(format_output[2*i])
            pcpu = float(format_output[2*i + 1])

            cpu_usage[psr] += pcpu

        return cpu_usage
