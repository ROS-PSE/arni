from status import Status
import psutil
from collections import namedtuple

statistic_tuple = namedtuple('statistic', ['mean', 'stddev', 'max'])


class HostStatus(Status):

    """
    Extension of Status , to store
    additional information used by hosts.
    """

    def __init__(self, start):

        super(HostStatus, self).__init__(start)

        #: CPU temp in celsius.
        self.__cpu_temp = []

        #: CPU temp by core in celsius.
        self.__cpu_temp_core = [[] for x in range(self._cpu_count)]

        #: GPU temp by card in celsius.
        self.__gpu_temp = []

        #: Dictionary holding sets of Network interface - bandwidth in bytes
        self.__bandwidth = {}

        #: Dictionary holding sets of
        #: Network interface - frequency of network calls in hertz.
        self.__msg_frequency = {}

        #: Dictionary holding sets of drive name - free space.
        self.__drive_space = {}

        #: Dictionary holding sets of drive name - bytes/s written.
        self.__drive_write = {}

        #: Dictionary holding sets of drive name - bytes/s read.
        self.__drive_read = {}

    def add_cpu_temp(self, temp):
        """
        Adds another measured value to cpu_temp.

        :param temp: measured temperature in celsius
        :type temp: int
        """
        self.__cpu_temp.append(temp)

    def add_cpu_temp_core(self, temps):
        """
        Adds another set of measured values to cpu_temp_core.

        :param temps: measured temperatures in celsius
        :type temp: int[]
        """
        for x in range(self._cpu_count):
            self.__cpu_temp_core[x].append(temps[x])

    def add_gpu_temp(self, temps):
        """
        Adds another set of measured values to gpu_temp.

        :param temp: measured temperatures in celsius
        :type temp: int[]
        """
        pass

    def add_bandwidth(self, interface, bytes):
        """
        Adds another  measured value, in bytes, to bandwidth belonging
        to the given network interface.

        :param interface: name of the network interface
        :type interface: string
        :param bytes: measured bytes
        :type bytes: float
        """
        if interface not in self.__bandwidth:
            self.__bandwidth[interface] = []

        self.__bandwidth[interface].append(bytes)

    def add_msg_frequency(self, interface, freq):
        """
        Adds another  measured value, in hertz, to msg_frequency belonging
        to the given network interface.

        :param interface: name of the network interface
        :type interface: string
        :param freq: measured frequency
        :type freq: float
        """
        if interface not in self.__msg_frequency:
            self.__msg_frequency[interface] = []

        self.__msg_frequency[interface].append(freq)

    def add_drive_write(self, disk, byte):
        """
        Adds another  measured value, in bytes, to drive_write belonging 
        to the given disk.

        :param disk: name of the disk
        :type disk: string
        :param byte: bytes written
        :type byte: float
        """
        if disk not in self.__drive_write:
            self.__drive_write[disk] = []

        self.__drive_write[disk].append(byte)

    def add_drive_read(self, disk, byte):
        """
        Adds another  measured value, in bytes, to drive_read belonging 
        to the given disk.

        :param disk: name of the disk
        :type disk: string
        :param byte: bytes read
        :type byte: float
        """
        if disk not in self.__drive_read:
            self.__drive_read[disk] = []

        self.__drive_read[disk].append(byte)

    def add_drive_space(self, disk, space):
        """
        Adds the free space of a drive to the Dictionary

        :param disk: name of the disk
        :type disk: string
        :param space: free space
        :type byte: int 
        """
        self.__drive_space[disk] = space

    def reset_specific(self):
        """ 
        Resets the values specific to Host
        """
        del self.__cpu_temp[:]
        for i in self.__cpu_temp_core:
            del i[:]
        del self.__gpu_temp[:]

        self.__bandwidth.clear()
        self.__drive_read.clear()
        self.__drive_write.clear()
        self.__msg_frequency.clear()

    def calc_stats_specific(self):
        """
        Calculate statistical values specific to hosts
        and write them into the stats_dict.
        """
        self.__calc_temp_stats()
        self.__calc_net_stats()
        self.__calc_drive_stats()

        self._stats_dict['interface_name'] = [key for key in self.__bandwidth]
        self._stats_dict['drive_name'] = [key for key in self.__drive_space]
        self._stats_dict['drive_free_space'] = [self.__drive_space[key]
                                                for key in self.__drive_space]

    def __calc_temp_stats(self):
        """
        calculate statistics about temperatures.
        """
        cpu_temp = self.calc_stat_tuple(self.__cpu_temp)
        cpu_temp_core = [self.calc_stat_tuple(i) for i in self.__cpu_temp_core]
        gpu_temp = self.calc_stat_tuple(self.__gpu_temp)

        self._stats_dict['cpu_temp_mean'] = cpu_temp.mean
        self._stats_dict['cpu_temp_stddev'] = cpu_temp.stddev
        self._stats_dict['cpu_temp_max'] = cpu_temp.max

        self._stats_dict['cpu_temp_core_mean'] = [
            i.mean for i in cpu_temp_core]
        self._stats_dict['cpu_temp_core_stddev'] = [
            i.stddev for i in cpu_temp_core]
        self._stats_dict['cpu_temp_core_max'] = [i.max for i in cpu_temp_core]

        self._stats_dict['gpu_temp_mean'] = [gpu_temp.mean]
        self._stats_dict['gpu_temp_stddev'] = [gpu_temp.stddev]
        self._stats_dict['gpu_temp_max'] = [gpu_temp.max]

    def __calc_net_stats(self):
        """
        calculate statistics about network I/O.
        """
        bandwidth = [self.calc_stat_tuple(self.__bandwidth[key])
                     for key in self.__bandwidth]
        msg_frequency = [self.calc_stat_tuple(self.__msg_frequency[key])
                         for key in self.__msg_frequency]

        self._stats_dict['bandwidth_mean'] = [i.mean for i in bandwidth]
        self._stats_dict['bandwidth_stddev'] = [i.stddev for i in bandwidth]
        self._stats_dict['bandwidth_max'] = [i.max for i in bandwidth]

        self._stats_dict['message_frequency_mean'] = [
            i.mean for i in msg_frequency]
        self._stats_dict['message_frequency_stddev'] = [
            i.stddev for i in msg_frequency]
        self._stats_dict['message_frequency_max'] = [
            i.max for i in msg_frequency]

    def __calc_drive_stats(self):
        """
        Calculate statistics about Drive I/O
        """
        drive_write = [self.calc_stat_tuple(self.__drive_write[key])
                       for key in self.__drive_write]
        drive_read = [self.calc_stat_tuple(self.__drive_read[key])
                      for key in self.__drive_read]

        self._stats_dict['drive_write'] = [i.mean for i in drive_write]
        self._stats_dict['drive_read'] = [i.mean for i in drive_read]

    @property
    def cpu_temp(self):
        return self.__cpu_temp

    @property
    def cpu_temp_core(self):
        return self.__cpu_temp_core

    @property
    def gpu_temp(self):
        return self.__gpu_temp

    @property
    def bandwidth(self):
        return self.__bandwidth

    @property
    def msg_frequency(self):
        return self.__msg_frequency

    @property
    def drive_space(self, key):
        if key is None:
            return self.__drive_space
        else:
            return self.__drive_space[key]

    @property
    def drive_write(self):
        return self.__drive_write

    @property
    def drive_read(self):
        return self.__drive_read

    @cpu_temp.setter
    def cpu_temp(self, value):
        self.__cpu_temp = value

    @cpu_temp_core.setter
    def cpu_temp_core(self, value):
        self.__cpu_temp_core = value

    @gpu_temp.setter
    def gpu_temp(self, value):
        self.__gpu_temp = value

    @bandwidth.setter
    def bandwidth(self, value):
        self.__bandwidth = value

    @msg_frequency.setter
    def msg_frequency(self, value):
        self.__msg_frequency = value

    @drive_space.setter
    def drive_space(self, value):
        self.__drive_space = value

    @drive_read.setter
    def drive_read(self, value):
        self.__drive_read = value
