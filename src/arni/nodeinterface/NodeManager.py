

class NodeManger(object):

	""" 
	Can restart or stop nodes or execute a countermeasure. 
	"""
	
	def __init__(self):
	
		super(NodeManager, self).__init__()
		
		
		
	def stop_node(self, node_id):
		"""
		Stops the node with the given id.
		Returns a message about operation's success.
		
		:param node_id: id of the node to be stopped.
		:type node_id: String.
		:returns: String
		"""
		pass
		
	def restart_node(self, node_id):
		"""
		Restarts a node with the given id.
		Returns a message about operation's success.
		
		:param node_id: id of the node to be restarted.
		:type node_id: String.
		:returns: String
		"""
		pass
		
	def execute_command(self, args):
		"""
		Executes a system call with the given arguments.
		Returns a message about operation's success.
		
		:param args: Arguments for the system call
		:type args: String
		:returns: String
		"""
		pass