import StatisticsHandler from StatisticsHandler
import HostStatus from HostStatus
import NodeManager from NodeManager

class HostStatisticsHandler( StatisticsHandler):

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
		self.__node_list
		
	
	def measure_status(self):
		"""
		Collects information about the host's current status using psutils.
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