class CountermeasureNode(object):

	"""A ROS node. Evaluates incoming rated statistics with a list of constraints. If those constraints turn out to be true appropriate action is taken."""
	def __init__(self, arg):
		super(CountermeasureNode, self).__init__()
		self.arg = arg
		

	