from python_qt_binding.QtGui import QStandardItemModel, QAbstractItemModel
from python_qt_binding.QtCore import *
from threading import Lock
from size_delegate import SizeDelegate
from abstract_item import AbstractItem
import rospy

from arni_core import Singleton



from rosgraph_msgs.msg import TopicStatistics
from arni_msgs.msg import RatedStatistics
from arni_msgs.msg import NodeStatistics
from arni_msgs.msg import HostStatistics


class ROSModel(QAbstractItemModel):
    """
    Represents the data as a QtModel.
    This enables automated updates of the view.
    """

    # This ensures the singleton character of this class via metaclassing.
    __metaclass__ = Singleton

    def __init__(self, parent):
        """
        Defines the class attributes especially the root_item which later contains the
        list of headers e.g. for a TreeView representation.
        :param parent: the parent of the model
        :type parent:
        """
        QAbstractItemModel.__init__(parent)
        self.__parent = parent
        self.__model_lock = Lock()
        self.__root_item = AbstractItem()
        self.__item_delegate = SizeDelegate()
        self.__log_model = QStandardItemModel()
        self.__set_header_data()
        self.__mapping = {
            1: 'type',
            2: 'name',
            3: 'status',
            4: 'data'
        }


    def __set_header_data(self):
        self.__root_item.append_data({
            'type': 'type',
            'name': 'name',
            'status': 'status',
            'data:': 'data:',
        })

        # todo:is this correct
        self.headerDataChanged.emit()


    def data(self, index, role):
        """
        Returns the data of an item at the given index.

        :param index: the position from which the data is wanted
        :type index: QModelIndex
        :param role: the role that should be used
        :type role: int
        """
        if not index.isValid():
            return None
        elif role != Qt.DisplayRole:
            return None

        item = index.internalPointer()

        return item.__get_latest_data(self.__mapping[index.column()])


def flags(self, index):
    """
    Returns the flags of the item at the given index (like Qt::ItemIsEnabled).


    :param index:
    :type index: QModelIndex
    :returns: ItemFlags
    """
    if not index.isValid():
        return Qt.NoItemFlags
    return Qt.ItemIsEnabled | Qt.ItemIsSelectable


def headerData(self, section, orientation, role):
    """
    Returns the headerData at the given section.

    :param section:
    :type section: int
    :param orientation:
    :type orientation: Orientation
    :param role:
    :type role: int
    :returns: QVariant
    """
    if orientation == Qt.Horizontal and role == Qt.DisplayRole:
        return self.__root_item.get_latest_data(self.__mapping[section])
    raise IndexError("Illegal access to a non existent line.")


def index(self, row, column, parent):
    """
    Returns the index of an item at the given column/row.

    :param row:
    :type row: int
    :param column:
    :type column: int
    :param parent:
    :type parent: QModelIndex
    :returns: QModelIndex
    """
    if not self.hasIndex(row, column, parent):
        return QModelIndex()

    if not parent.isValid():
        parent_item = self.__root_item
    else:
        parent_item = parent.internalPointer()

    child_item = parent_item.get_child(row)
    if child_item:
        return self.createIndex(row, column, child_item)
    else:
        return QModelIndex()


def parent(self, index):
    """
    Returns the QModelIndex of the parent of the child item specied via its index.

    :param index:
    :type index:
    :returns: QModelIndex
    """
    if not index.isValid():
        return QModelIndex()

    child_item = index.internalPointer()
    parent_item = child_item.parent()

    if parent_item == self.rootItem:
        return QModelIndex()

    return self.createIndex(parent_item.row(), 0, parent_item)


def rowCount(self, parent):
    """
    Returns the amount of rows in the model.

    :param parent:
    :type parent: QModelIndex
    :returns: int
    """
    if parent.column() > 0:
        return 0

    if not parent.isValid():
        parent_item = self.rootItem
    else:
        parent_item = parent.internalPointer()

    return parent_item.childCount()


def columCount(self, parent):
    """
    Returns the amount of columns in the model.

    :param parent:
    :type parent: QModelIndex
    :returns: int
    """
    if parent.isValid():
       return parent.internalPointer().column_count()
    else:
       return self.rootItem.column_count()


def update_model(self, rated_statistics, topic_statistics, host_statistics, node_statistics):
    """
    Updates the model by using the items of the list. The items will be of the message types .

    :param rated_statistics:
    :type rated_statistics: list
    :param topic_statistics:
    :type topic_statistics: list
    :param node_statistics:
    :type node_statistics: list
    :param host_statistics:
    :type host_statistics: list
    """
    #todo: remove in productional code
    now = rospy.Time.now()

    for item in topic_statistics:
        self.__transform_topic_statistics_item(item)

    for item in node_statistics:
        self.__transform_node_statistics_item(item)

    for item in host_statistics:
        self.__transform_host_statistics_item(item)
    #rating last because it needs the time of the items before
    for item in rated_statistics:
        self.__transform_rated_statistics_item(item)

    #todo: does this work correctly?
    rospy.logdebug("update_model (in ros_model) took: %s ", rospy.Time.now() - now)


def __transform_rated_statistics_item(self, data):
    """
    Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

    :param data:
    :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
    """
    pass


def __transform_topic_statistics_item(self, data):
    """
    Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

    :param data:
    :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
    """
    pass


def __transform_node_statistics_item(self, data):
    """
    Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

    :param data:
    :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
    """
    pass


def __transform_host_statistics_item(self, data):
    """
    Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

    :param data:
    :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
    """
    pass


def add_log_item(self, list):
    """
    Adds the given list as a log entry to the model.

    :param list: accepts a list of strings and adds these to the log model
    :type list: list
    """
    pass