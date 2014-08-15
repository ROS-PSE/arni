class LogFilterProxy(object):


    """The LogFilterProxy will espacially be used to filter the complete log e.g. by a specific node. This function is needed in the SelectionWidget where of course only the log of the current selection should be shown."""


def __init__(self, parent):
    """Initializes the LogFilterProxy

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


def filter_by_item(self, item):
    """Used to tell the filter by which item should filter. If the AbstractItem is None all log entries should be shown.

    :param item: the item by which the filter should filter
    :type item: AbstractItem
    """
    pass
