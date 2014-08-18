from statistcs_handler import StatisticsHandler
from host_status import HostStatus
from node_manager import NodeManager
import psutil


class HostStatisticsHandler(StatisticsHandler):

    """
    Represents a host , limited to one instance per host. 
    Collects statistics about the current state of the host and s
    ends them using the publisher-subscriber mechanism. 
    """

    def __init(self, hostid):

        super(HostStatisticsHandler, self).__init__()

        #: Identifier of the host, using ROS IP as unique identifier.
        self._id = hostid

        #: Used to store information about the host's status.
        self._status = HostStatus()

        #: Interface to restart and stop nodes or executing other commands.
        self.__node_manager

        #: Dictionary holding all nodes currently running on the host.
        self.__node_list = {}

    def measure_status(self):
        """
        Collects information about the host's current status using psutils.
        Triggered periodically.
        """

        # CPU
        self._status.add_cpu_usage(psutil.cpu_percent())

        self._status.add_cpu_usage_core(psutil.cpu_percent(None, True))

        # RAM
        self._status.add_ram_usage(psutil.virtual_memory().percent)

        # todo: temp

        # Bandwidth and message frequency
        network_interfaces = psutil.net_io_counters(True)

        for key in network_interfaces:

            total_bytes = network_interfaces[
                key] + network_interfaces[key]  # todo proper intervalls
            total_packages = network_interfaces[
                key].packets_sent + network_interfaces[key].packets.recv

            self._status.add_bandwidth(key, total_bytes)
            self._status.add_msg_frequency(key, total_packages)

        # Free Space on disks
        disks = psutil.disk_partitions(True)

        for x in disks:
            if 'cdrom' not in disks:
                free_space = psutil.disk_usage(disks[x].mountpoint).free

                self._status.add_drive_space(disks.device, free_space)

        # Drive I/O
        drive_io = psutil.disk_io_counters(True)

        for key in drive_io:
            # eventually intervalls
            self._status.add_drive_read(key, drive_io[key].read_bytes)
            self._status.add_drive_write(key, drive_io[key].write_bytes)

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

        :returns: HostStatistics 
        """
        pass

    def execute_reaction(self, reaction):
        """
        Parses through the reaction and 
        calls the appropriate method from the NodeManager. Uses ROS Services.
        Returns a message about operation's success.

        :param reaction: Reaction to be executed and node affected.
        :type reaction: NodeReaction.
        :returns: String
        """
        pass

    def add_node(self, node_id):
        """
        Adds a Node with the given id to the host.

        :param node_id: id of the node to be added.
        :type node_id: String
        """
        pass

    def remove_node(self, node_id):
        """
        Removes the Node with the given id from the host.

        :param node_id: id of the node to be removed.
        :type node_id: String
        """
