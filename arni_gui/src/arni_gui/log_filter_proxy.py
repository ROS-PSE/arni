import sys

from abstract_item import AbstractItem
try:  # Qt4 vs Qt5
  from python_qt_binding.QtGui import QSortFilterProxyModel
except ImportError:
  from python_qt_binding.QtCore import QSortFilterProxyModel
from python_qt_binding.QtCore import QObject, QRegExp, Qt

if sys.version_info[0] is 2 or (sys.version_info[0] is 3 and sys.version_info[1] < 2):
    from lru_cache import lru_cache
else:
    from functools import lru_cache

class LogFilterProxy(QSortFilterProxyModel):
    """
    The LogFilterProxy will especially be used to filter the complete log e.g. by a specific node.
    This function is needed in the SelectionWidget where of course only the log of the current selection
    should be shown.
    """

    def __init__(self, parent=None):
        """
        Initializes the LogFilterProxy
    
        :param parent: the parent-object
        :type parent: QObject
        """
        super(LogFilterProxy, self).__init__(parent)
        self.__current_item = None

    def invalidateFilter(self):
        """
        Invalidates the filter
        """
        QSortFilterProxyModel.invalidateFilter(self)
        #invalidate cache
        self.filterAcceptsRow.cache_clear()

    @lru_cache(None)
    def filterAcceptsRow(self, source_row, source_parent):
        """
        Tells by analysing the given row if it should be shown or not. This behaviour can be modified via setFilterRegExp
         method so that e.g. only the entries of a specific host can be shown.

        :param source_row: the source row
        :type source_row: int
        :param source_parent: the source of the parent
        :type source_parent: QModelIndex

        :returns: bool
        """
        return QSortFilterProxyModel.filterAcceptsRow(self, source_row, source_parent)


    def lessThan(self, left, right):
        """
        Defines the sorting of behaviour when comparing two entries of model item by telling how to compare these.

        :param left: the left-hand side
        :type left: QModelIndex
        :param right: the right-hand side
        :type right: QModelIndex

        :returns: bool
        """
        return left < right


    def filter_by_item(self, item):
        """
        Used to tell the filter by which item should filter. If the AbstractItem is None all log entries should be shown.

        :param item: the item by which the filter should filter
        :type item: AbstractItem
        """
        self.invalidateFilter()
        if item is not None:
            self.setFilterRegExp(QRegExp(".*" + item.get_seuid() + ".*"))
            self.setFilterKeyColumn(2)

    def setFilterRegExp(self, string):
        self.invalidateFilter()
        QSortFilterProxyModel.setFilterRegExp(self, string)
