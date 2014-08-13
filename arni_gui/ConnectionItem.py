class ConnectionItem(AbstractItem):
"""A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on"""

    def __init__(self, list, parent=None):
	"""Initializes the ConnectionItem

	:param list: connection list
	:type list: list
	:param parent: the parent-object
	:type parent: object
	"""
	pass

    def execute_action(self, action):
	"""Not senseful, throws an exception

	:param action: action to be executed
	:type action: RemoteAction
	"""
	pass

