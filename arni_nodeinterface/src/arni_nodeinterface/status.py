from status import Status
from collections import namedtuple

statistic_tuple = namedtuple('statistic', ['mean', 'stddev','max'])


class NodeStatus(Status):
    """
    Extension of Status , to store additional information used by nodes.
    """
    
    def __init__(self):
    
        super(NodeStatus, self).__init__()
        
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

    def calc_stats_specific(self , dict):

        node_bandwidth = self.calc_stat_tuple(self.__node_bandwidth)
        node_msg_frequency = self.calc_stat_tuple(self.__node_msg_frequency)
        node_read = self.calc_stat_tuple(self.__node_read)
        node_write = self.calc_stat_tuple(self.__node_write)

        dict['node_bandwidth'] = node_bandwidth
        dict['node_msg_frequency'] = node_msg_frequency
        dict['node_read'] = node_read
        dict['node_write'] = node_write

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