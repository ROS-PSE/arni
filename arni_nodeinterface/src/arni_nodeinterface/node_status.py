import Status from Status


class NodeStatus(Status):

	"""
	Extension of Status , to store additional information used by nodes.
	"""
	
	def __init__(self):
	
		super(NodeStatus, self).__init__()
		
		#: Network bandwidth used by the node in bytes.
		self.__node_bandwidth
		
		#: Bytes read from disk by node.
		self.__node_read
		
		#: Bytes written to disk by node.
		self.__node_write
		
		#: Frequency of network calls by node.
		self.__node_msg_frequency
		
		
		
	def add_node_bandwidth(self, bytes):
		"""
		Adds another measured value in bytes, taken
		from ROS topics statistics, to node_bandwidth. 
		
		:param bytes: Bytes measured.
		:type bytes: int
		"""
		pass
		
	def add_node_io(self, read, write):
		"""
		Adds another pair of measured disk I/O values 
		to node_read and node_write. 
		
		:param read: Bytes read.
		:type read: int
		:param write: Bytes written.
		:type write: int
		"""
		pass
		
	def add_node_msg_freq(self, freq):
		"""
		Adds another measured value to node_msg_frequency,
		taken from ROS topics statistics. 
		
		:param freq: frequency of network calls.
		:type bytes: int
		"""
		
	def reset(self):
		"""
		Resets the current status. 
		"""