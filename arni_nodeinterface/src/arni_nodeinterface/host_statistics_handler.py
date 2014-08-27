from statistics_handler import StatisticsHandler
from node_statistics_handler import NodeStatisticsHandler
from host_status import HostStatus
from node_manager import NodeManager
from threading import Thread
import arni_msgs
from arni_msgs.msg import HostStatistics
from arni_msgs.srv import NodeReaction
import xmlrpclib
import socket
import rosnode
import rospy
import sys
try:
    import sensors
    import psutil
except ImportError:
    sys.stderr.write(
        'External Packages PySensors and Psutil must be installed')
    sys.exit(1)

if psutil.__version__ < '2.1.1':
    sys.stderr.write(
        'Installed psutil version is outdated. Update Psutil to 2.1.1 or newer')
    sys.exit(1)

import threading


class HostStatisticsHandler(StatisticsHandler):

    """
    Represents a host , limited to one instance per host. 
    Collects statistics about the current state of the host and s
    ends them using the publisher-subscriber mechanism. 
    """

    def __init__(self, hostid):
        """
        Collects resource usage data of host
        and nodes.
        :param hostid: ROS_IP of this host
        :type hostid: string
        """
        super(HostStatisticsHandler, self).__init__(hostid)

        self.__update_interval = 0
        self.__publish_interval = 0
        self.__is_enabled = False
        self.__check_enabled = 0
        self.__search_nodes_inv = 0
        self.__init_params()
        self.__register_service()
        self.__lock = threading.Lock()
        self.__dict_lock = threading.Lock()

        self.pub = rospy.Publisher(
            '/statistics_host', HostStatistics, queue_size=500)
        #: Used to store information about the host's status.
        self._status = HostStatus(rospy.Time.now())

        #: Interface to restart and stop nodes
        # or executing other commands.
        self.__node_manager = NodeManager()

        #: Dictionary holding all nodes currently running on the host.
        self.__node_list = {}

        # Base-stats for I/O
        self.__bandwidth_base = {}
        self.__msg_freq_base = {}
        self.__disk_write_base = {}
        self.__disk_read_base = {}

        self.__set_bases()

    def __set_bases(self):
        """
        Calculate base stats for I/O at start of the node.
        Used to calculate difference to get per/s stats.
        """
        psutil.cpu_percent()
        psutil.cpu_percent(percpu=True)

        for interface in psutil.net_io_counters(True):

            netint = psutil.net_io_counters(True)[interface]
            total_bytes = netint.bytes_recv + netint.bytes_sent
            total_packages = netint.packets_sent + netint.packets_recv

            self.__bandwidth_base[interface] = total_bytes
            self.__msg_freq_base[interface] = total_packages

        dev_names = ''
        for disk in psutil.disk_partitions():
            if all(['cdrom' not in disk.opts, 'sr' not in disk.device]):
                dev_names += disk.device + ';'

        for key in psutil.disk_io_counters(True):
            if key in dev_names:
                disk = psutil.disk_io_counters(True)[key]
                self.__disk_read_base[key] = disk.read_bytes
                self.__disk_write_base[key] = disk.write_bytes

    def __register_service(self):
        """
        Register all services
        """
        ip = self._id.replace('.', '_')
        rospy.Service(
            "/execute_node_reaction/%s" % ip,
            NodeReaction, self.execute_reaction)

    def measure_status(self, event):
        """
        Collects information about the host's current status using psutils.
        Triggered periodically.
        """
        if not self.__is_enabled:
            rospy.logdebug('enable_statistics not set')
            pass
        if self.__is_enabled:
            self.__lock.acquire()
            # update node list
            self.__dict_lock.acquire()
            self.update_nodes()
            self.__dict_lock.release()
            rospy.logdebug('measuring host status')
            # CPU
            self._status.add_cpu_usage(psutil.cpu_percent())
            self._status.add_cpu_usage_core(psutil.cpu_percent(percpu=True))
            # RAM
            self._status.add_ram_usage(psutil.virtual_memory().percent)
            # temp

            self.get_sensors()

            # Bandwidth and message frequency
            self.__measure_network_usage()
            # Disk usage
            self.__measure_disk_usage()
            self.__lock.release()

    def __measure_network_usage(self):
        """
        measure current network_io_counters
        """
        network_interfaces = psutil.net_io_counters(True)

        for key in network_interfaces:
            total_bytes = (network_interfaces[
                           key].bytes_sent + network_interfaces[key].bytes_recv) - self.__bandwidth_base[key]
            total_packages = (network_interfaces[
                              key].packets_sent + network_interfaces[key].packets_recv) - self.__msg_freq_base[key]

            bandwidth = total_bytes / self.__update_interval
            msg_frequency = total_packages / self.__update_interval
            self._status.add_bandwidth(key, bandwidth)
            self._status.add_msg_frequency(key, msg_frequency)

            # update base stats for next iteration
            self.__bandwidth_base[key] += total_bytes
            self.__msg_freq_base[key] += total_packages

    def __measure_disk_usage(self):
        """
        measure current disk usage 
        """
        # Free Space on disks
        disks = psutil.disk_partitions()
        dev_name = ''
        for x in disks:
            if 'cdrom' not in x.opts and 'sr' not in x.device:
                free_space = psutil.disk_usage(x.mountpoint).free / 2 ** 20
                self._status.add_drive_space(x.device, free_space)
                dev_name += x.device + ';'

        # Drive I/O
        drive_io = psutil.disk_io_counters(True)
        for key in drive_io:
            if key in dev_name:
                readb = drive_io[key].read_bytes - self.__disk_read_base[key]
                writeb = drive_io[key].write_bytes - \
                    self.__disk_write_base[key]

                read_rate = readb / self.__update_interval
                write_rate = writeb / self.__update_interval
                self._status.add_drive_read(key, read_rate)
                self._status.add_drive_write(key, write_rate)
                # update base stats for next iteration
                self.__disk_read_base[key] += readb
                self.__disk_write_base[key] += writeb

    def publish_status(self, event):
        """
        Publishes the current status to a topic using ROS's publisher-subscriber mechanism.
        Triggered periodically. 
        """
        if not self.__is_enabled:
            rospy.logdebug('enable_statistics not set')
            pass
        if self.__is_enabled:
            self.__lock.acquire()
            self._status.time_end = rospy.Time.now()
            stats = self.__calc_statistics()
            self.__dict_lock.acquire()
            rospy.logdebug('Publishing Host ')
            self.__publish_nodes()
            self.__dict_lock.release()
            self.pub.publish(stats)
            self._status.reset()
            self._status.time_start = rospy.Time.now()
            self.__lock.release()

    def __publish_nodes(self):
        """
        publishes current status of all nodes.
        """
        for node in self.__node_list:
            Thread(target=self.__node_list[node].publish_status).start()

    def __init_params(self):
        """
        Initializes params on the parameter server,
        """
        self.__update_interval = rospy.get_param('~update_interval', 1)
        self.__publish_interval = rospy.get_param('~publish_interval', 10)
        self.__is_enabled = rospy.get_param('/enable_statistics', False)
        self.__check_enabled = rospy.get_param(
            '/arni/check_enabled_interval', 10)
        self.__search_nodes_inv = rospy.get_param('~search_nodes', 5)

    def __calc_statistics(self):
        """
        Calculates statistics like mean, standard deviation and max from the status.
        Returns an instance of HostStatistics which can be published.

        :returns: HostStatistics 
        """
        stats_dict = self._status.calc_stats()

        hs = HostStatistics()

        hs.host = self._id
        hs.window_start = self._status.time_start
        hs.window_stop = self._status.time_end

        hs.cpu_usage_mean = stats_dict['cpu_usage_mean']
        hs.cpu_usage_stddev = stats_dict['cpu_usage_stddev']
        hs.cpu_usage_max = stats_dict['cpu_usage_max']

        hs.cpu_temp_mean = stats_dict['cpu_temp_mean']
        hs.cpu_temp_stddev = stats_dict['cpu_temp_stddev']
        hs.cpu_temp_max = stats_dict['cpu_temp_max']

        hs.cpu_usage_core_mean = stats_dict['cpu_usage_core_mean']
        hs.cpu_usage_core_stddev = stats_dict['cpu_usage_core_stddev']
        hs.cpu_usage_core_max = stats_dict['cpu_usage_core_max']

        hs.cpu_temp_core_mean = stats_dict['cpu_temp_core_mean']
        hs.cpu_temp_core_stddev = stats_dict['cpu_temp_core_stddev']
        hs.cpu_temp_core_max = stats_dict['cpu_temp_core_max']

        hs.ram_usage_mean = stats_dict['ram_usage_mean']
        hs.ram_usage_stddev = stats_dict['ram_usage_stddev']
        hs.ram_usage_max = stats_dict['ram_usage_max']

        hs.interface_name = stats_dict['interface_name']
        hs.message_frequency_mean = stats_dict['message_frequency_mean']
        hs.message_frequency_stddev = stats_dict['message_frequency_stddev']
        hs.message_frequency_max = stats_dict['message_frequency_max']

        hs.bandwidth_mean = stats_dict['bandwidth_mean']
        hs.bandwidth_stddev = stats_dict['bandwidth_stddev']
        hs.bandwidth_max = stats_dict['bandwidth_max']

        hs.drive_name = stats_dict['drive_name']
        hs.drive_free_space = stats_dict['drive_free_space']
        hs.drive_write = stats_dict['drive_write_mean']
        #hs.drive_write_stddev = stats_dict['drive_write_stddev']
        #hs.drive_write_max = stats_dict['drive_write_max']
        hs.drive_read = stats_dict['drive_read_mean']
        #hs.drive_read_stddev = stats_dict['drive_read_stddev']
        #hs.drive_read_max = stats_dict['drive_read_max']

        hs.gpu_temp_mean = stats_dict['gpu_temp_mean']
        hs.gpu_temp_stddev = stats_dict['gpu_temp_stddev']
        hs.gpu_temp_max = stats_dict['gpu_temp_max']
        hs.gpu_usage_mean = stats_dict['gpu_usage_mean']
        hs.gpu_usage_stddev = stats_dict['gpu_usage_stddev']
        hs.gpu_usage_max = stats_dict['gpu_usage_max']

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

        msg = ''
        if reaction.node not in self.__node_list:
            return 'Specified node is not running on this host.'

        if reaction.action == 'restart':
            node = self.__node_list[reaction.node]
            msg = self.__node_manager.restart_node(node)
            self.remove_node(reaction.node)
        elif reaction.action == 'stop':
            msg = self.__node_manager.stop_node(reaction.node)
            self.remove_node(reaction.node)
        elif reaction.action == 'command':
            msg = self.__node_manager.execute_command(reaction.command)
        else:
            msg = 'Failed to execute reaction, %s is no valid argument' % reaction.action
        return msg

    def remove_node(self, node):
        """
        Removes the Node with the given id from the host.

        :param node_id: id of the node to be removed.
        :type node_id: String
        """
        if node in self.__node_list:
            del self.__node_list[node]

    def get_node_info(self, event):
        """
        Get the list of all Nodes running on the host.
        Update the __node_list
        """
        if not self.__is_enabled:
            rospy.logdebug('enable_statistics not set')
            pass

        if self.__is_enabled:
            nodes = []
            for node_name in rosnode.get_node_names():
                node_api = rosnode.get_api_uri(
                    rospy.get_master(), node_name, skip_cache=True)
                if self._id in node_api[2]:
                    nodes.append(node_name)

            for node in nodes:
                if node not in self.__node_list:
                    node_api = rosnode.get_api_uri(
                        rospy.get_master(), node, skip_cache=True)
                    try:
                        rospy.logdebug('Getting Nodeinfo %s' % node)
                        pid = self.node_server_proxy(node_api, node)
                        if not pid:
                            continue
                        node_process = psutil.Process(pid)
                        new_node = NodeStatisticsHandler(
                            self._id, node, node_process)
                        self.__dict_lock.acquire(True)
                        self.__node_list[node] = new_node
                        self.__dict_lock.release()
                    except psutil.NoSuchProcess:
                        rospy.loginfo('pid %d does not exit' % pid)
                        continue

            self.__dict_lock.acquire()
            to_remove = []
            for node_name in self.__node_list:
                if node_name not in nodes:
                    rospy.logdebug('removing %s from host' % node_name)
                    to_remove.append(node_name)
            for node_name in to_remove:
                self.remove_node(node_name)
            self.__dict_lock.release()

            rospy.logdebug([key for key in self.__node_list])

    def node_server_proxy(self, node_api, node):
        """
        Use the xmlrpclib Server Proxy to get
        node- pid
        :param node_api: API URI used for ServerProxy
        :type node_api: list
        :param node:  name of the node
        :type node: string
        """
        socket.setdefaulttimeout(1)
        try:
            rospy.logdebug('Node %s at API %s' % (node, node_api))
            code, msg, pid = xmlrpclib.ServerProxy(
                node_api[2]).getPid('/NODEINFO')

            return pid
        except socket.error:
            rospy.logdebug('Node %s is unreachable' % node)
            return False

    def update_nodes(self):
        """
        update the status of each node in its own threading
        """

        for node in self.__node_list:
            Thread(target=self.__node_list[node].measure_status).start()

    def get_sensors(self):
        """
        collects the current temperatur of CPU
        and each core
        """

        sensors.init()
        added = []
        cpu_temp_c = []
        try:
            for chip in sensors.iter_detected_chips():
                for feature in chip:
                    if feature.name.startswith('temp'):
                        if ((feature.label.startswith('Physical') or
                                feature.label.startswith('CPU')) and
                                feature.label not in added):
                            self._status.add_cpu_temp(feature.get_value())
                        elif (feature.label.startswith('Core')
                                and feature.label not in added):
                            cpu_temp_c.append(feature.get_value())
                        added.append(feature.label)
        except sensors.SensorsError:
            pass
        if cpu_temp_c:
            try:
                self._status.add_cpu_temp_core(cpu_temp_c)
            except IndexError:
                pass
        sensors.cleanup()

    def check_enabled(self, event):
        """
        Periodically checks if /enable_statistics is ture
        if false no data will be collected / published
        """
        rospy.logdebug('checking enable_statistics == true')
        self.__is_enabled = rospy.get_param('/enable_statistics', False)
        rospy.logdebug('enabled is %s' % self.__is_enabled)

    @property
    def update_interval(self):
        return self.__update_interval

    @property
    def publish_interval(self):
        return self.__publish_interval

    @property
    def check_enabled_interval(self):
        return self.__check_enabled

    @property
    def search_nodes_inv(self):
        return self.__search_nodes_inv
