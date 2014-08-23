from python_qt_binding.QtGui import QStandardItemModel
from python_qt_binding.QtCore import QAbstractItemModel
from python_qt_binding.QtCore import *
from threading import Lock
from size_delegate import SizeDelegate
from abstract_item import AbstractItem

from connection_item import ConnectionItem
from topic_item import TopicItem
from host_item import HostItem
from node_item import NodeItem

import rospy
from rospy.rostime import Duration, Time

from arni_core.singleton import Singleton

from rosgraph_msgs.msg import TopicStatistics
from arni_msgs.msg import RatedStatistics
from arni_msgs.msg import NodeStatistics
from arni_msgs.msg import HostStatistics

from helper_functions import UPDATE_FREQUENCY

from buffer_thread import *

class QAbstractItemModelSingleton(Singleton, type(QAbstractItemModel)):
    pass

class ROSModel(QAbstractItemModel):
    """
    Represents the data as a QtModel.
    This enables automated updates of the view.
    """

    # This ensures the singleton character of this class via metaclassing.
    __metaclass__ = QAbstractItemModelSingleton

    def __init__(self, parent=None):
        """
        Defines the class attributes especially the root_item which later contains the
        list of headers e.g. for a TreeView representation.
        :param parent: the parent of the model
        :type parent:
        """
        #rospy.init_node('arni_gui_model', log_level=rospy.DEBUG)
        super(ROSModel, self).__init__(parent)
        self.__root_item = AbstractItem("root", self)

        self.__root_item.append_data_dict({
            'type': 'type',
            'name': 'name',
            'state': 'state',
            'data:': 'data',
            'window_end': rospy.get_rostime(),
            "total_traffic": 0,
            "connected_hosts": 0,
            "connected_nodes": 0,
            "topic_counter": 0,
            "connection_counter": 0,
            "cpu_usage_max": 0,
            "cpu_temp_stddev": 0,
            "average_ram_load": 0,
            "cpu_usage_stddev": 0,
            "cpu_temp_max": 0,
            "ram_usage_max": 0,
        })
        self.__parent = parent
        self.__model_lock = Lock()

        self.__identifier_dict = {"root": self.__root_item}
        self.__item_delegate = SizeDelegate()
        self.__log_model = QStandardItemModel(0, 4, None)
        self.__log_model.setHorizontalHeaderLabels(["type", "date", "location", "message"])
        #self.__set_header_data()
        self.__mapping = {
            1: 'type',
            2: 'name',
            3: 'state',
            4: 'data'
        }
        rospy.logdebug("Finished model initialization.")
        self.__buffer_thread = BufferThread(self)
        self.__last_time_error_occured = 0

        self.add_log_entry("info", rospy.Time.now(), "ROSModel", "ROSModel initilization finished")
        self.add_log_entry("error", rospy.Time.now(), "ROSModel", "Just testing")



#is no longer needed because the header data won't change while running
    # def __set_header_data(self):
    #
    #
    #     # todo:is this correct
    #     self.headerDataChanged.emit(Qt.Horizontal, 1, 4)


    def get_overview_data_since(self, time=None):
        """
        Return the info needed for the OverviewWidget as a dict.

        :param time:
        :type time: rospy.Time
        :return: dict of values
        """
        if time is None:
            data_dict = self.__root_item.get_latest_data()
        else:
            data_dict = self.__root_item.get_items_younger_than(time)

        return data_dict



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
        print("orientation is %s, Role is %s", orientation, role)
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
            parent_item = self.__root_item
        else:
            parent_item = parent.internalPointer()

        return parent_item.child_count()


    def columnCount(self, parent):
        """
        Returns the amount of columns in the model.

        :param parent:
        :type parent: QModelIndex
        :returns: int
        """
        if parent.isValid():
           return parent.internalPointer().column_count()
        else:
           return self.__root_item.column_count()


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
        self.layoutAboutToBeChanged.emit()
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

        data_dict = {
            "total_traffic": 0,
            "connected_hosts": 0,
            "connected_nodes": 0,
            "topic_counter": 0,
            "connection_counter": 0,
            "cpu_usage_max": 0,
            "cpu_temp_mean": 0,
            "average_ram_load": 0,
            "cpu_usage_mean": 0,
            "cpu_temp_max": 0,
            "ram_usage_max": 0,
        }
        # made it global
        #last_time_error_occured = 0

        connected_hosts = 0
        connected_nodes = 0
        topic_counter = 0
        connection_counter = 0

        #time window
        #todo: where should total_traffic be calculated? currently sum of the bandwidth of the nics
        #generate the general information
        for host_item in self.__root_item.get_childs():
            #hostinfo
            connected_hosts += 1
            data = host_item.get_items_younger_than(Time.now() - Duration(nsecs=UPDATE_FREQUENCY))
            #anpassen an host_item!!!!!
            for key in data:
                for entry in data[key]:
                    if key is not ["bandwidth_mean", "cpu_usage_max", "cpu_temp_mean", "average_ram_load", "cpu_usage_mean", "cpu_temp_max", "ram_usage_max"]:
                        pass
                    elif key is "bandwidth_mean":
                        data_dict["total_traffic"] += entry
                    else:
                        data_dict[key] += entry

            for node_item in host_item.get_childs():
                #nodeinfo
                connected_nodes += 1
                for topic_item in node_item.get_childs():
                    #topic info
                    topic_counter += 1
                    for connection_item in topic_item.get_childs():
                        #connection info
                        connection_counter += 1

        data_dict["connected_hosts"] = connected_hosts
        data_dict["connected_nodes"] = connected_nodes
        data_dict["topic_counter"] = topic_counter
        data_dict["connection_counter"] = connection_counter

        #now give this information to the root :)
        self.__root_item.append_data_dict(data_dict)

        #todo: does this work correctly?
        rospy.logdebug("update_model (in ros_model) took: %s ", rospy.Time.now() - now)

        self.layoutChanged.emit()


    def __transform_rated_statistics_item(self, item):
        """
        Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

        :param data:
        :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
        """
        # get identifier
        seuid = item.seuid
        # check if avaiable
        if seuid not in self.__identifier_dict:
            #having a problem, item doesn't exist but should not be created here
            raise UserWarning("Received rated statistics for an item that doesn't existit the database!")

        #update it
        current_item = self.__identifier_dict[seuid]
        data = {}
        for element in item.rated_statistics_entity:
            for number in range(0, len(element.statistic_type)):
                data[element.statistic_type + ".actual_value " + number] = element.actual_value
                data[element.statistic_type + ".expected_value " + number] = element.expected_value
                data[element.statistic_type+ ".state " + number] = element.state
                if element.state is not element.OK:
                    data["state"] = "error"

        current_item.update_data(data, item.window_start, item.window_end)



    def __transform_topic_statistics_item(self, item):
        """
        Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item
        (especially the TopicItem and the ConnectionItem).

        :param data:
        :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
        """
        # get identifier
        #todo: adapt this to the changes of the seuid!
        topic_seuid = "t!" + item.topic
        connection_seuid = "c!" + item.node_sub + "!" + item.topic + "!" + item.node_pub
        topic_item = None
        connection_item = None
        # check if avaiable
        if topic_seuid not in self.__identifier_dict:
            #creating a topic item
            parent = self.get_item_by_seuid(item.node_pub)
            if parent is None:
                # having a problem, there is no node with the given name
                raise UserWarning("The parent of the given topic statistics item cannot be found.")

            topic_item = TopicItem(topic_seuid, parent)
            self.__identifier_dict[topic_seuid] = connection_item
            #creating a connection item
            connection_item = ConnectionItem(connection_seuid, topic_item)
            self.__identifier_dict[connection_seuid] = connection_item
        elif connection_seuid not in self.__identifier_dict:
            #creating a new connection item
            connection_item = ConnectionItem(connection_seuid, topic_item)
            self.__identifier_dict[connection_seuid] = connection_item
        else:
            # get topic and connection item
            topic_item = self.get_item_by_seuid(topic_seuid)
            connection_item = self.get_item_by_seuid(connection_seuid)
            if topic_item is None or connection_item is None:
                raise UserWarning("The parent of the given topic statistics item cannot be found.")

        # now update these
        connection_item.append_data(item)
        topic_item.append_data(item)

    def __transform_node_statistics_item(self, item):
        """
        Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

        :param data:
        :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
        """
        node_item = None
        if item.seuid not in self.__identifier_dict:
            #create item
            node_item = NodeItem()
        else:
            node_item = self.get_item_by_seuid(item.seuid)

        node_item.append_data(item)

    def __transform_host_statistics_item(self, item):
        """
        Integrates a TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item (especially the TopicItem and the ConnectionItem).

        :param data:
        :type data: AbstractItem, Statistics, HostStatistics, RatedStatistics, StatisticHistory
        """
        host_item = None
        if item.seuid not in self.__identifier_dict:
            #create item
            host_item = NodeItem()
        else:
            host_item = self.get_item_by_seuid(item.seuid)

        host_item.append_data(item)


    def get_item_by_seuid(self, seuid):
        for item in self.__root_item.get_childs():
            value = self.__get_item_by_name(seuid, item)
            if value is not None:
                return value
        return None

    def __get_item_by_seuid(self, seuid, current_item):
        if current_item.get_seuid() == seuid:
            return current_item
        for item in current_item.get_childs():
            self.__get_item_by_name()
        return None

    def get_log_model(self):
        return self.__log_model

    def add_log_entry(self, type, date, location, message):
        #todo: doku
        """
        Adds the given list as a log entry to the model.

        :param list: accepts a list of strings and adds these to the log model
        :type list: list
        """
        self.__log_model.insertRow(0)
        self.__log_model.setData(self.__log_model.index(0, 0), type)
        self.__log_model.setData(self.__log_model.index(0, 1), str(date))
        self.__log_model.setData(self.__log_model.index(0, 2), location)
        self.__log_model.setData(self.__log_model.index(0, 3), message)