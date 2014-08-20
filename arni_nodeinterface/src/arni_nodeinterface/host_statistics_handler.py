from statistics_handler import StatisticsHandler
from node_statistics_handler import NodeStatisticsHandler
from host_status import HostStatus
from node_manager import NodeManager
from threading import Thread
from arni_msgs.msg import HostStatistics
from arni_msgs.srv import NodeReaction
import psutil
import xmlrpclib
import rosnode
import rospy
import sensors

class HostStatisticsHandler( StatisticsHandler):

    """
    Represents a host , limited to one instance per host. 
    Collects statistics about the current state of the host and s
    ends them using the publisher-subscriber mechanism. 
    """
    
    def __init__(self, hostid):
        
        super(HostStatisticsHandler, self).__init__()

        rospy.init_node("HostStatistics", log_level=rospy.DEBUG)

        self.__init_params()
        self.__register_service()

        self.__update_intervall =  rospy.get_param('/update_intervall')

        self.__publish_intervall =  rospy.get_param('/publish_intervall')

        #set Topic statistics to same time_window as update_intervall
        rospy.set_param('/enable_statistics', True )
        rospy.set_param('/statistics_window_min_elements',
                        self.__update_intervall )
        rospy.set_param('/statistics_window_max_elements', 
                        self.__publish_intervall)

        #: Identifier of the host, using ROS IP as unique identifier.
        self._id = hostid
        
        #: Used to store information about the host's status.
        self._status = HostStatus(rospy.Time.now())
        
        #: Interface to restart and stop nodes 
        #or executing other commands.
        self.__node_manager = NodeManager()
        
        #: Dictionary holding all nodes currently running on the host.
        self.__node_list = {}
        
        #Base-stats for I/O
        self.__bandwidth_base = {}

        for interface in psutil.net_io_counters(True):
            
            total_bytes = psutil.net_io_counters(True)[interface].bytes_recv + psutil.net_io_counters(True)[interface].bytes_sent
            total_packages = psutil.net_io_counters(True)[interface].packets_sent + psutil.net_io_counters(True)[interface].packets.recv
            
            self.__bandwidth_base[interface] = total_bytes
            self.__msg_freq_base[interface] = total_packages

        self.__disk_write_base = {}
        self.__disk_read_base = {}

        for key in psutil.disk_io_counters(True):
             self.__disk_read_base[key] = psutil.disk_io_counters(True)[key].read_bytes
             self.__disk_write_base[key] = psutil.disk_io_counters(True)[key].write_bytes

    
    def __register_service(self):
        """
        Register all services
        """
        rospy.Service("/" + self._id + "/execute_node_reaction", 
                        NodeReaction, self.execute_reaction)

    def measure_status(self):
        """
        Collects information about the host's current status using psutils.
        Triggered periodically.
        """

        #update node list
        self.get_node_info()

        self.update_nodes()


        #CPU 
        self._status.add_cpu_usage(psutil.cpu_percent())
        self._status.add_cpu_usage_core(psutil.cpu_percent(None , True))

        #RAM
        self._status.add_ram_usage(psutil.virtual_memory().percent)

        #temp
        sensor_list = []
        self.get_sensors(sensor_list)        

        
        #Bandwidth and message frequency
        self.__measure_network_usage()

        #Disk usage        
        self.__measure_disk_usage()


    def __measure_network_usage(self):
        """
        measure current network_io_counters
        """
        network_interfaces = psutil.net_io_counters(True)

        for key in network_interfaces:
            
            total_bytes = self.__bandwidth_base[key] - (network_interfaces[key] + network_interfaces[key])
            total_packages = self.__msg_freq_base[key] - (network_interfaces[key].packets_sent + network_interfaces[key].packets.recv)

            self._status.add_bandwidth(key, total_bytes/self.__update_intervall)
            self._status.add_msg_frequency(key,total_packages/self.__update_intervall)

            #update base stats for next iteration
            self.__bandwidth_base[key] += total_bytes
            self.__msg_freq_base[key] += total_packages    
    
    def __measure_disk_usage(self):
        """
        measure current disk usage 
        """
        # Free Space on disks
        disks = psutil.disk_partitions(True)

        for x in disks:
            if 'cdrom' not in disks:
                free_space = psutil.disk_usage(disks[x].mountpoint).free

                self._status.add_drive_space( x , free_space)

        #Drive I/O
        drive_io = psutil.disk_io_counters(True)

        for key in drive_io:
            readb = self.__disk_read_base[key] - drive_io[key].read_bytes
            writeb = self.__disk_write_base[key] - drive_io[key].write_bytes

            read_rate = readb / self.__update_intervall
            write_rate = writeb / self.__update_intervall

            self._status.add_drive_read(key, read_rate) #eventually intervalls
            self._status.add_drive_write(key, write_rate)

            ##update base stats for next iteration
            self.__disk_read_base[key] += drive_io[key].read_bytes
            self.__disk_write_base[key] += drive_io[key].write_bytes


    def publish_status(self):
        """
        Publishes the current status to a topic using ROS's publisher-subscriber mechanism.
        Triggered periodically. 
        """
        pub = rospy.Publisher('/host_statistics', HostStatistics)
        self._status.time_end(rospy.Time.now())
        stats = self.__calc_statistics()

        self.__publish_nodes()

        pub.publish(stats)
        self._status.reset()
        self._status.time_start(rospy.Time.now())

    def __publish_nodes(self):
        """
        publishes current status of all nodes.
        """
        for node in self.__node_list:
            Thread(self.__node_list[node].publish_status, ()).start()   

        
    def __init_params(self):
        """
        Initializes params on the parameter server,
        """

        rospy.set_param('/update_intervall', 1)
        rospy.set_param('/publish_intervall', 10)


    def __calc_statistics(self):
        """
        Calculates statistics like mean, standard deviation and max from the status.
        Returns an instance of HostStatistics which can be published.
        
        :returns: HostStatistics 
        """
        stats_dict = self._status.calc_stats()

        hs = HostStatistics()

        
        hs.host = self._id

        hs.cpu_usage_mean = stats_dict['cpu_usage'].mean 
        hs.cpu_usage_stddev = stats_dict['cpu_usage'].stddev 
        hs.cpu_usage_max = stats_dict['cpu_usage'].max

        hs.cpu_temp_mean = stats_dict['cpu_temp'].mean
        hs.cpu_temp_stddev = stats_dict['cpu_temp'].stddev
        hs.cpu_temp_max = stats_dict['cpu_temp'].max

        hs.cpu_usage_core_mean = [stats_dict['cpu_usage_core'][i].mean
                                    for i in range(psutil.cpu_count())]
        hs.cpu_usage_core_stddev = [stats_dict['cpu_usage_core'][i].stddev 
                                    or i in range(psutil.cpu_count())]
        hs.cpu_usage_core_max = [stats_dict['cpu_usage_core'][i].max 
                                for i in range(psutil.cpu_count())]

        hs.cpu_temp_core_mean = [stats_dict['cpu_temp_core'][i].mean 
                                for i in range(psutil.cpu_count())]
        hs.cpu_temp_core_stddev = [stats_dict['cpu_temp_core'][i].stddev 
                                    for i in range(psutil.cpu_count())]
        hs.cpu_temp_core_max = [stats_dict['cpu_temp_core'][i].max 
                                for i in range(psutil.cpu_count())]

        hs.ram_usage_mean = stats_dict['ram_usage'].mean
        hs.ram_usage_stddev = stats_dict['ram_usage'].stddev
        hs.ram_usage_max = stats_dict['ram_usage'].max

        mfr = len(stats_dict['msg_frequency'])
        hs.interface_name = [key for key in psutil.network_io_counters(True)]
        hs.message_frequency_mean =  [stats_dict['msg_frequency'][i].mean 
                                        for i in range(mfr)]
        hs.message_frequency_stddev = [stats_dict['msg_frequency'][i].stddev 
                                        for i in range(mfr)]
        hs.message_frequency_max = [stats_dict['msg_frequency'][i].stddev 
                                    for i in range(mfr)]

        bwr = len(stats_dict['bandwidth'])
        hs.bandwidth_mean =  [stats_dict['bandwidth'][i].mean 
                            for i in range(bwr)]
        hs.bandwidth_stddev = [stats_dict['bandwidth'][i].stddev 
                                for i in range(bwr)]
        hs.bandwidth_max = [stats_dict['bandwidth'][i].stddev 
                            for i in range(bwr)]

        
        hs.drive_name = [key for key in self._status.drive_space]
        hs.drive_free_space = [self._status.drive_space(key) 
                                for key in self._status.drive_space]


        dwr = len(stats_dict['drive_write'])
        hs.drive_write_mean = [stats_dict['drive_write'][x].mean 
                                for x in range(dwr)]
        hs.drive_write_stddev = [stats_dict['drive_write'][x].stddev 
                                for x in range(dwr)]
        hs.drive_write_max = [stats_dict['drive_write'][x].max 
                                for x in range(dwr)]
        
        drr = len(stats_dict['drive_read'])
        hs.drive_read_mean = [stats_dict['drive_read'][x].mean 
                                for x in range(drr)]
        hs.drive_read_stddev = [stats_dict['drive_read'][x].stddev 
                                for x in range(drr)]
        hs.drive_read_max = [stats_dict['drive_read'][x].max 
                                for x in range(drr)]
    

        return hs


    def execute_reaction(self, reaction):
        """
        Parses through the reaction and 
        calls the appropriate method from the NodeManager. Uses ROS Services.
        Returns a message about operation's success.
        
        :param reaction: Reaction to be executed and node affected.
        :type reaction: NodeReaction.
        :returns: String
        """

        if reaction.action =='restart':
            node = self.__node_list[reaction.node]
            return self.__node_manager.restart_node(node)
        elif reaction.action == 'stop':     
            return self.__node_manager.restart_node(reaction.node)
        elif reaction.action == 'command':
            return self.__node_manager.execute_command(reaction.command)

        
    def remove_node(self, node):
        """
        Removes the Node with the given id from the host.
        
        :param node_id: id of the node to be removed.
        :type node_id: String
        """
        if node._id in self.__node_list:
            del self.__node_list[node._id]

    def get_node_info(self):
        """
        Get the list of all Nodes running on the host.
        Update the __node_list
        """ 
    
        for node_name in rosnode.get_node_names():
            if node_name not in self.__node_list:
                node_api = rosnode.get_api_uri(rospy.get_master(), node_name)
                try:
                    code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid('/NODEINFO')
                    node_process = psutil.Process(pid)
                    new_node = NodeStatisticsHandler(self._id, node_name, node_process)

                    self.__node_list[node_name] = new_node
                except xmlrpclib.socket.error:
                    return False

    def update_nodes(self):
        """
        update the status of each node in its own threading
        """

        for node in self.__node_list:
            Thread(self.__node_list[node].measure_status, ()).start()

    def get_sensors(self, sensor_list):

        sensors.init()
        try:
            for chip in sensors.iter_detected_chips():
                for feature in chip:
                    if feature.name.startswith(b'temp'):
                        sensor_list.append(feature)
        except sensors.SensorsError:
            pass


def main():
    
    #howto ROS_IP ?
    host = HostStatisticsHandler('localhost')

    try:
        while not rospy.is_shutdown():
            for i in range(host.__publish_intervall / host.__update_intervall):
                host.measure_status
                rospy.sleep(host.__update_intervall)

            host.publish_status()
    except rospy.ROSInterruptException:
        pass