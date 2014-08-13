class ItemFilterProxy:QSortFilterProxyModel:
"""The ItemFilterProxy which is a QSortFilterProxyModel helps to filter the data going to the view so the user only sees what he wants to see (which he can modified by telling the view)."""

    def __init__(self, parent):
	"""Initializes the ItemFilteRproxy
	
	:param parent: the parent-object
	:type parent: QObject
	"""
	pass

    def filterAcceptsRow(self, source_row, source_parent):
	"""Tells by analysing the given row if it should be shown or not. This behaviour can be modified via setFilterRegExp mehod so that e.g. only the entries of a specific host can be shown.

	:param source_row: the source row
	:type source_row: int
	:param source_parent: he source of the parent
	:type source_parent: QModellIndex

	:returns: bool
	"""
	pass

    def lessThan(self, left, right):
	"""Defines the sorting of behaviour when comparing two entries of model item by telling how to compare these.

	:param left: the left-hand side
	:type left: QModellIndex
	:param right: the right-hand side
	:type right: QModellIndex

	:returns: bool
	"""
	pass

    def show_hosts(self, show_hosts):
	"""Set true if hosts should be shown

	:param show_hosts: true if hosts should be shown
	:type show_hosts: bool
	"""
	pass

    def show_nodes(self, show_nodes):
	"""Set true if nodes should be shown

	:param show_nodes: true if nodes should be shown
	:type show_nodes: bool
	"""
	pass

    def show_connections(self, show_connections):
	"""Set true if connections should be shown

	:param show_connections: true if connections should be shown
	:type show_connections: bool
	"""
	pass

    def show_topics(self, show_topics):
	"""Set true if topics should be shown

	:param show_topics: true if topics should be shown
	:type show_topics: bool
	"""
	pass 
