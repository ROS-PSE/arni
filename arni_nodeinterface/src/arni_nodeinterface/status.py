import psutil
from math import sqrt, ceil
from collections import namedtuple


statistic_tuple = namedtuple('statistic', ['mean', 'stddev', 'max'])


class Status(object):

    """
    Container Class to Store information about the current status.
    """

    def __init__(self, start):
        """
        :param start: start of time_window
        :type start: rospy.Time
        """
        #: Cpu usage in percent.
        self._cpu_usage = []

        self._cpu_count = psutil.cpu_count()

        #: Cpu usage per core in percent.
        self._cpu_usage_core = [[] for x in range(self._cpu_count)]

        #: Gpu usage per card
        self._gpu_usage = []

        #:Ram usage
        self._ram_usage = []

        #: Start of the time window
        self._time_start = start

        #:End of the time window
        self._time_end = 0
        self._stats_dict = {}

    def add_cpu_usage(self, usage):
        """
        Adds another measured value to cpu_usage.

        :param usage: measured percentage of cpu used.
        :type usage: float
        """
        self._cpu_usage.append(usage)

    def add_cpu_usage_core(self, usage):
        """
        Adds another set of measured values per core to  cpu_usage_core.

        :param usage: measured percentage of cpu used per core.
        :type usage: float[]
        """
        for x in range(self._cpu_count):
            self._cpu_usage_core[x].append(usage[x])

    def add_gpu_usage(self, usage):
        """
        Adds another set of measured values per card to  gpu. 

        :param usage: measured percentage of gpu used.
        :type usage: float[]
        """
        pass

    def add_ram_usage(self, usage):
        """
        Adds another measured value to ram_usage. 

        :param usage: measured percentage of ram used.
        :type usage: float
        """
        self._ram_usage.append(usage)

    def reset(self):
        """
        Resets the status .
        """

        del self._cpu_usage[:]
        for i in self._cpu_usage_core:
            del i[:]
        del self._gpu_usage[:]
        del self._ram_usage[:]

        self.reset_specific()

    def reset_specific(self):
        """
        Resets the values specific to Host or Nodes
        """

        pass

    def calc_stats(self):
        """
        returns a dictionary containing avg,stddev,max values.
        matching the fields in HostStatistics / NodeStatistics

        :returns: Dictionary
        """

        self.__calc_cpu_stats()
        self.__calc_ram_stats()
        self.__calc_gpu_stats()

        self.calc_stats_specific()
        return self._stats_dict

    def __calc_cpu_stats(self):
        """
        calculate statistics about cpu usage
        """
        cpu_usage = self.calc_stat_tuple(self._cpu_usage)
        cpu_usage_core = [self.calc_stat_tuple(i)
                          for i in self._cpu_usage_core]

        self._stats_dict['cpu_usage_mean'] = cpu_usage.mean
        self._stats_dict['cpu_usage_stddev'] = cpu_usage.stddev
        self._stats_dict['cpu_usage_max'] = cpu_usage.max

        self._stats_dict['cpu_usage_core_mean'] = [i.mean
                                                   for i in cpu_usage_core]
        self._stats_dict['cpu_usage_core_stddev'] = [i.stddev
                                                     for i in cpu_usage_core]
        self._stats_dict['cpu_usage_core_max'] = [i.max
                                                  for i in cpu_usage_core]

    def __calc_gpu_stats(self):
        """
        calculate statistics about gpu usage
        placeholder not implemented yet, returns 0 - values.
        """
        gpu_usage = self.calc_stat_tuple(self._gpu_usage)
        self._stats_dict['gpu_usage_mean'] = [gpu_usage.mean]
        self._stats_dict['gpu_usage_stddev'] = [gpu_usage.stddev]
        self._stats_dict['gpu_usage_max'] = [gpu_usage.max]

    def __calc_ram_stats(self):
        """
        calculate statistics about ram usage
        """
        ram_usage = self.calc_stat_tuple(self._ram_usage)

        for key in vars(ram_usage):
            self._stats_dict['ram_usage_%s' % key] = vars(ram_usage)[key]

    def calc_stat_tuple(self, slist):
        """
        Returns a named tuple containing mean , standard deviation and maximum
        of a given list. returns zero-tuple if list is empty.

        :returns: namedtuple
        """
        if not slist or all(not i for i in slist):
            return statistic_tuple(0, 0, 0)
        else:
            maxi = max(slist)
            mean = (sum(slist) / float(len(slist)))
            temp_sum = 0

            for i in slist:
                temp_sum += (i - mean) ** 2
            stddev = 0
            if len(slist) > 1:
                stddev = sqrt(float(1) / (len(slist) - 1) * temp_sum) 

            return statistic_tuple(mean, stddev, maxi)

    def calc_stats_specific(self, dict):

        pass

    @property
    def cpu_usage(self):
        return self._cpu_usage

    @property
    def cpu_usage_core(self):
        return self._cpu_usage_core

    @property
    def gpu_usage(self):
        return self._gpu_usage

    @property
    def ram_usage(self):
        return self._ram_usage

    @property
    def time_start(self):
        return self._time_start

    @property
    def time_end(self):
        return self._time_end

    @cpu_usage.setter
    def cpu_usage(self, value):
        self._cpu_usage = value

    @cpu_usage_core.setter
    def cpu_usage_core(self, value):
        self._cpu_usage_core = value

    @gpu_usage.setter
    def gpu_usage(self, value):
        self._gpu_usage = value

    @ram_usage.setter
    def ram_usage(self, value):
        self._ram_usage = value

    @time_start.setter
    def time_start(self, value):
        self._time_start = value

    @time_end.setter
    def time_end(self, value):
        self._time_end = value
