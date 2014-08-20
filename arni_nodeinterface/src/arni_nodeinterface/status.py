import psutil
from math import sqrt
from collections import namedtuple


statistic_tuple = namedtuple('statistic', ['mean', 'stddev','max'])


class Status(object):
    """
    Container Class to Store information about the current status.
    """
    

    def __init__(self, start):
        
        super(Status, self).__init__()
        
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
        self._time_start
        
        #:End of the time window
        self._time_end
        
        
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
        del self._cpu_usage_core[:]
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

        result_dict = {}
        
        cpu_usage = self.calc_stat_tuple(self._cpu_usage)
        cpu_usage_core = [self.calc_stat_tuple(self._cpu_usage_core[i]) for i in range(self._cpu_count)]
        ram_usage = self.calc_stat_tuple(self._ram_usage)

        self.calc_stats_specific(result_dict)

        result_dict['cpu_usage'] = cpu_usage        
        result_dict['cpu_usage_core'] =  cpu_usage_core     
        result_dict['ram_usage'] = ram_usage
        
        return result_dict


    def calc_stat_tuple(self, slist):
        """
        Returns a named tuple containing mean , standard deviation and maximum
        of a given list.

        :returns: namedtuple
        """
        if not slist:
            return slist
        else:    
            maxi = max(slist)
            mean = sum(slist) / float(len(slist))
            temp_sum = 0

            for i in range(slist):
                temp_sum += ( slist[i] - mean )**2

            stddev = sqrt( float(1)/(len(slist) - 1 ) * temp_sum )

            return statistic_tuple(mean , stddev , maxi)

    def calc_stats_specific(self , dict):

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
