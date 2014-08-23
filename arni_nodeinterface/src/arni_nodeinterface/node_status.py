from status import Status
from collections import namedtuple

statistic_tuple = namedtuple('statistic', ['mean', 'stddev','max'])


class NodeStatus(Status):
    """
    Extension of Status , to store additional information used by nodes.
    """
    
    def __init__(self, start):
    
        super(NodeStatus, self).__init__(start)
        
        #: Network bandwidth used by the node in bytes.
        self.__node_bandwidth = []
        
        #: Bytes read from disk by node.
        self.__node_read = []
        
        #: Bytes written to disk by node.
        self.__node_write = []
        
        #: Frequency of network calls by node.
        self.__node_msg_frequency = []
        

        
        
    def add_node_bandwidth(self, bytes):
        """
        Adds another measured value in bytes, taken
        from ROS topics statistics, to node_bandwidth. 
        
        :param bytes: Bytes measured.
        :type bytes: int
        """
        self.__node_bandwidth.append(bytes)
        
    def add_node_io(self, read, write):
        """
        Adds another pair of measured disk I/O values 
        to node_read and node_write. 
        
        :param read: Bytes read.
        :type read: int
        :param write: Bytes written.
        :type write: int
        """
        self.__node_read.append(read)
        self.__node_write.append(write)
        
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

        del self.__node_bandwidth[:]
        del self.__node_read[:]
        del self.__node_write[:]
        del self.__node_msg_frequency[:]

    def calc_stats_specific(self):
        """
        calculates statistics specific to nodes.
        """

        self.__calc_net_stats()
        self.__calc_drive_stats()

    def __calc_net_stats(self):
    
        node_bandwidth = self.calc_stat_tuple(self.__node_bandwidth)
        node_msg_frequency = self.calc_stat_tuple(self.__node_msg_frequency)

        self._stats_dict['node_message_frequency_mean'] = node_msg_frequency.mean
        self._stats_dict['node_message_frequency_stddev'] = node_msg_frequency.stddev
        self._stats_dict['node_message_frequency_max'] = node_msg_frequency.max

        self._stats_dict['node_bandwidth_mean'] = node_bandwidth.mean
        self._stats_dict['node_bandwidth_stddev'] = node_bandwidth.stddev
        self._stats_dict['node_bandwidth_max'] = node_bandwidth.max

    def __calc_drive_stats(self):

        node_read = self.calc_stat_tuple(self.__node_read)
        node_write = self.calc_stat_tuple(self.__node_write)

        self._stats_dict['node_read_mean'] = node_read.mean
        self._stats_dict['node_read_stddev'] = node_read.stddev
        self._stats_dict['node_read_max'] = node_read.max

        self._stats_dict['node_write_mean'] = node_write.mean
        self._stats_dict['node_write_stddev'] = node_write.stddev
        self._stats_dict['node_write_max'] = node_write.max       



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
    def node_msg_frequency(self,value):
        self.__node_msg_frequency = value