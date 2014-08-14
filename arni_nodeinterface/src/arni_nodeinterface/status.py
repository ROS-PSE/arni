import psutil

class Status(object):

	"""
	Container Class to Store information about the current status.
	"""
	
	def __init__(self):
		
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

