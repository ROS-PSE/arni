try:  # Qt4 vs Qt5
  from python_qt_binding.QtGui import QSortFilterProxyModel
except ImportError:
  from python_qt_binding.QtCore import QSortFilterProxyModel
from python_qt_binding.QtCore import QObject, QModelIndex

from arni_gui.topic_item import TopicItem
from arni_gui.tree_topic_item import TreeTopicItem
from arni_gui.ros_model import ROSModel

from rqt_graph.dotcode import QUIET_NAMES

import sys

if sys.version_info[0] is 2 or (sys.version_info[0] is 3 and sys.version_info[1] < 2):
    from lru_cache import lru_cache
else:
    from functools import lru_cache


class ItemFilterProxy(QSortFilterProxyModel):
    """
    The ItemFilterProxy which is a QSortFilterProxyModel helps to filter the data going to the view so the user only sees what he wants to see (which he can modified by telling the view).
     """


    def __init__(self, parent=None):
        """
        Initializes the ItemFilterProxy

        :param parent: the parent-object
        :type parent: QObject
        """
        super(ItemFilterProxy, self).__init__(parent)
        self.__show_hosts = True
        self.__show_nodes = True
        self.__show_connections = True
        self.__show_topics = True
        self.__show_subscribers = True
        self.__hide_debug = True

        self.__filter_string = ""

        self.__quiet_names = []
        for entry in QUIET_NAMES:
            self.__quiet_names.append(entry[1:])
        # Note: One can add further entries to __quiet_names here - these will be used to pre-filter the gui item

    def invalidateFilter(self):
        """
        Invalidates the filter
        """
        QSortFilterProxyModel.invalidateFilter(self)
        # invalidate cache
        self.filterAcceptsRow.cache_clear()

    # creating cache with infinite size
    @lru_cache(None)
    def filterAcceptsRow(self, source_row, source_parent):
        """
        Tells by analysing the given row if it should be shown or not. This behaviour can be modified via
        setFilterRegExp method so that e.g. only the entries of a specific host can be shown.

        :param source_row: the source of the parent
        :type source_row: int
        :param source_parent: the source of the parent
        :type source_parent: QModelIndex

        :returns: True if the row should be shown
        :rtype: bool
        """
        entries = []
        item = source_parent.internalPointer()
        child = None
        if item is not None:
            if isinstance(item, TreeTopicItem):
                child = item.get_child(source_row, self.sourceModel().parent(
                    source_parent).internalPointer())
            else:
                child = source_parent.internalPointer().get_child(source_row)
            entries = [child.get_type(), child.get_seuid(), child.get_state(), child.get_short_data()]
        else:
            child = self.sourceModel().get_root_item().get_child(source_row)
            entries = [child.get_type(), child.get_seuid(), child.get_state(), child.get_short_data()]

        child_childs = child.get_childs( self.sourceModel().parent(
                    source_parent).internalPointer())

        for i in range(0, len(child_childs)):
            if self.filterAcceptsRow(i, self.sourceModel().index(source_row, 0, source_parent)):
                return True

        correct_type = False
        data = entries[0]

        if data[0] == "h":
            correct_type = True
        elif self.__show_nodes and data[0] == "n":
            correct_type = True
        elif self.__show_connections and data[0] == "c":
            if self.__show_subscribers:
                correct_type = True
            else:
                if child.is_subscriber:
                    correct_type = False
                else:
                    correct_type = True
        elif self.__show_topics is True:
            if data[0] == "t":
                correct_type = True

        if correct_type is False:
            return False

        if self.__hide_debug is True:
            for entry in self.__quiet_names:
                if entries[1].find(entry) is not -1:
                    return False


        # todo: speed this implementation a lot up by not using the model!!!
        if self.__filter_string is not "":
            for i in range(0, len(entries)):
                if self.__filter_string in entries[i]:
                    return QSortFilterProxyModel.filterAcceptsRow(self, source_row, source_parent)
            return False
        return QSortFilterProxyModel.filterAcceptsRow(self, source_row, source_parent)

    def setFilterRegExp(self, string):
        self.invalidateFilter()
        QSortFilterProxyModel.setFilterRegExp(self, string)

    def lessThan(self, left, right):
        """
        Defines the sorting of behaviour when comparing two entries of model item by telling how to compare these.

        :param left: the left-hand side
        :type left: QModellIndex
        :param right: the right-hand side
        :type right: QModellIndex

        :returns: bool
        """
        return left < right

    def show_hosts(self, show_hosts):
        """
        Set true if hosts should be shown

        :param show_hosts: true if hosts should be shown
        :type show_hosts: bool
        """
        print("function ItemFilterProxy::show_hosts is deprecated and should no longer be used.")
        self.__show_hosts = show_hosts
        self.invalidateFilter()

    def hide_debug(self, hide_debug):
        """
        Hides the debug gui entries if hide_debug is True.

        Note: By modifying this entry filters can be created.

        :type hide_debug: bool
        """
        if self.__hide_debug is not hide_debug:
            self.__hide_debug = hide_debug
            self.invalidateFilter()


    def show_nodes(self, show_nodes):
        """
        Set true if nodes should be shown

        :param show_nodes: true if nodes should be shown
        :type show_nodes: bool
        """
        self.__show_nodes = show_nodes
        self.invalidateFilter()

    def show_connections(self, show_connections):
        """
        Set true if connections should be shown

        :param show_connections: true if connections should be shown
        :type show_connections: bool
        """
        self.__show_connections = show_connections
        self.invalidateFilter()

    def show_topics(self, show_topics):
        """
        Set true if topics should be shown

        :param show_topics: true if topics should be shown
        :type show_topics: bool
        """
        self.__show_topics = show_topics
        self.invalidateFilter()

    def show_subscribers(self, show_subscribers):
        """
        Set true if subscriber should be shown

        :param show_subscriber: true if subscriber should be shown
        :type show_subscriber: bool
        """
        self.__show_subscribers = show_subscribers
        self.invalidateFilter()

    def set_filter_string(self, filter_string):
        self.invalidateFilter()
        self.__filter_string = filter_string
