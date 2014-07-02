

class Status(object):

	"""
	Container Class to Store information about the current status.
	"""
	
	def __init__(self):
		
		super(Status, self).__init__()
		
		#: Cpu usage in percent.
		self._cpu_usage = []
		
		#: Cpu usage per core in percent.
		self._cpu_usage_core = [[]]
		
		#: Gpu usage per card
		self._gpu_usage = [[]]
		
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
		pass
		
	def add_cpu_usage_core(self, usage):
		"""
		Adds another set of measured values per core to  cpu_usage_core.
		
		:param usage: measured percentage of cpu used per core.
		:type usage: float[]
		"""
		pass
		
	def add_gpu_usage(self, usage):
		"""
		Adds another set of measured values per card to  gpu. 
		
		:param usage: measured percentage of gpu used.
		:type usage: float[]
		"""
		pass
		
	def ram_usage(self, usage):
		"""
		Adds another measured value to ram_usage. 
		
		:param usage: measured percentage of ram used.
		:type usage: float
		"""
		pass