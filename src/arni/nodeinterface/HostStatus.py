from Status import Status

class HostStatus(Status):

	"""Extension of Status , to store 
	additional information used by hosts. """
	
	def __init__(self):
		
		super(HostStatus, self).__init__()
		
		#: CPU temp in celsius.
		self.__cpu_temp =  []
		
		#: CPU temp by core in celsius.
		self.__cpu_temp_core = [[]]
		
		#: GPU temp by card in celsius.
		self.__gpu_temp = [[]]
		
		#: Dictionary holding sets of Network interface - bandwidth in bytes
		self.__bandwidth
		
		#: Dictionary holding sets of 
		#: Network interface - frequency of network calls in hertz.
		self.__msg_frequency
		
		#: Dictionary holding sets of drive name - free space.
		self.__drive_space
		
		#: Dictionary holding sets of drive name - bytes/s written.
		self.__drive_write
		
		#: Dictionary holding sets of drive name - bytes/s read.
		self.__drive_read
		
	
	def add_cpu_temp(self, temp):
		"""Adds another measured value to cpu_temp. 
		
		:param temp: measured temperature in celsius
		:type temp: int
		"""
		pass
		
	def add_cpu_temp_core(self, temps):
		"""Adds another set of measured values to cpu_temp_core.
		
		:param temps: measured temperatures in celsius
		:type temp: int[]
		"""
		pass
		
	def add_gpu_temp(self, temps):
		"""Adds another set of measured values to gpu_temp.
		
		:param temp: measured temperatures in celsius
		:type temp: int[]
		"""
		pass
		
	def add_bandwidth(self, interface, bytes):
		"""Adds another  measured value, in bytes, to bandwidth belonging 
		to the given network interface.
		
		:param interface: name of the network interface
		:type interface: string
		:param bytes: measured bytes
		:type bytes: int		
		"""
		pass
		
	def add_msg_frequency(self, interface, freq):
		"""Adds another  measured value, in hertz, to msg_frequency belonging 
		to the given network interface.
		
		:param interface: name of the network interface
		:type interface: string
		:param freq: measured frequency
		:type freq: int	
		"""
		pass
		
	def add_drive_write(self, disk, byte):
		"""Adds another  measured value, in bytes, to drive_write belonging 
		to the given disk.
		
		:param disk: name of the disk
		:type disk: string
		:param byte: bytes written
		:type byte: int	
		"""
		pass
		
	def add_drive_read(self, disk, byte):
		"""Adds another  measured value, in bytes, to drive_read belonging 
		to the given disk.
		
		:param disk: name of the disk
		:type disk: string
		:param byte: bytes read
		:type byte: int	
		"""
		pass
		
	def reset(self):
		""" Resets the status """
		pass