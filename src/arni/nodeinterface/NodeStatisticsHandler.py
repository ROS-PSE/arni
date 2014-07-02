import StatisticsHandler from StatisticsHandler

class NodeStatisticsHandler(StatisticsHandler):

	"""
	Holds the statistics of an individual Node.
	"""
	
	def __init__(self, host_id, node_id):
	
		super(NodeStatisticsHandler, self).__init__()
		
		#:identifier of this node
		self._id = node_id
		
		#:Host this node runs on
		self.__host_id = host_id
		
		#: Status of the node
		self._status = NodeStatus()
		
	
	def measure_status(self):
		"""
		Collects information about the node's current status 
		using psutils and rospy.statistics
		Triggered periodically.
		"""
		pass
		
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
		
		:returns: NodeStatistics 
		"""
		pass
		
	def __receive_statistics(self):
		"""
		Receives the statistics published by ROS Topic statistics.
		"""
		
		pass