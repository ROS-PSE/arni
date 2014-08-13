class NodeItem(AbstractItem):
"""A TopicItem represents a node with all of its data. It also has a interface to start/stop/restart nodes."""

    def __init__(self, list, parent=None):
	"""Initializes the NodeItem

	:param list: node list
	:type list: list
	:param parent: the parent-object
	:type parent: object
	"""
	pass

    def execute_action(self, action):
	"""Sends a signal to top or restart the node.

	:param action: action to be executed
	:type action: RemoteAction
	"""
	pass

