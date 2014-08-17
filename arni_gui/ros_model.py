from python_qt_binding.QtGui import QStandardItemModel, QAbstractItemModel
from python_qt_binding.QtCore import *
from threading import Lock
# todo: adapt these imports!!!
import SizeDelegate
import AbstractItem


# todo: import Statistics,HostStatistics,RatedStatistics,StatisticHistory from where????


class ROSModel(QAbstractItemModel):
    """
    Represents the data as a QtModel.
    This enables automated updates of the view.
    """

    # This ensures the singleton character of this class via metaclassing.
    __metaclass__ = Singleton

    def __init__(self, data):
        """
        Defines the class attributes especially the root_item which later contains the
        list of headers e.g. for a TreeView representation.
        :param data:
        :type data:list
        """
        QAbstractItemModel.__init__(self)
        self.__model_lock = Lock()
        self.__root_item = AbstractItem()
        self.__item_delegate = SizeDelegate()
        self.__log_model = QStandardItemModel()



    def data(self, index, role):
        """
        Returns the data of an item at the given index.

        :param index: the position from which the data is wanted
        :type index: QModelIndex
        :param role: the role that should be used
        :type role: int
        """
        pass

    def flags(self, index):
        """
        Returns the flags of the item at the given index (like Qt::ItemIsEnabled).


        :param index:
        :type index: QModelIndex
        :returns: ItemFlags
        """
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def header_data(self, section, orientation, role):
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
        pass

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
        pass

    def parent(self, index):
        """
        Returns the QModelIndex of the parent of the child item specied via its index.

        :param index:
        :type index:
        :returns: QModelIndex
        """
        pass

    def rowCount(parent):
        """
        Returns the amount of rows in the model.

        :param parent:
        :type parent: QModelIndex
        :returns: int
        """
        pass

    def columnCount(self, parent):
        """
        Returns the amount of columns in the model.

        :param parent:
        :type parent: QModelIndex
        :returns: int
        """
        pass

    def update_model(self, data, rated_data):
        """
        Updates the model by using the items of the list. The items will be of the message types .

        :param data:
        :type data: list
        :param rated_data:
        :type rated_data: list
        """
        pass

    def transform_data(self, data):
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