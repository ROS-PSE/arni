from python_qt_binding.QtCore import QAbstractItemModel, QModelIndex
from python_qt_binding.QtCore import QTranslator, Qt
from python_qt_binding.QtGui import qApp

from threading import Lock
from size_delegate import SizeDelegate
from abstract_item import AbstractItem

from connection_item import ConnectionItem
from topic_item import TopicItem
from host_item import HostItem
from node_item import NodeItem
from root_item import RootItem

import rospy
from rospy.rostime import Duration, Time
from rospy.timer import Timer

from arni_core.singleton import Singleton

from rosgraph_msgs.msg import TopicStatistics
from arni_msgs.msg import RatedStatistics
from arni_msgs.msg import NodeStatistics
from arni_msgs.msg import HostStatistics

from model_logger import ModelLogger

from arni_core.helper import SEUID, SEUID_DELIMITER

import time

from helper_functions import UPDATE_FREQUENCY, MAXIMUM_AMOUNT_OF_ENTRIES, MINIMUM_RECORDING_TIME, topic_statistics_state_to_string, find_qm_files

import rospkg
import os

from buffer_thread import *

from arni_core.host_lookup import HostLookup


class QAbstractItemModelSingleton(Singleton, type(QAbstractItemModel)):
    """
    Helper-Class which allow ROSModel to be a Singleton
    """
    pass


class ROSModel(QAbstractItemModel):
    """
    Enables automated updates of the view, it represents the data as a QtModel
    """
    # This ensures the singleton character of this class via metaclassing.
    __metaclass__ = QAbstractItemModelSingleton

    def __init__(self, parent=None):
        """
        Defines the class attributes especially the root_item which later contains the list of headers e.g. for a TreeView representation.
        :param parent: the parent of the model
        :type parent: QObject
        """
        super(ROSModel, self).__init__(parent)

        self.__logger = ModelLogger()

        self._translator = QTranslator()
        # internationalize everything including the 2 plugins
        self.rp = rospkg.RosPack()
        directory = os.path.join(self.rp.get_path('arni_gui'), 'translations')
        files = find_qm_files(directory)
        translator = self.get_translator()
        #todo: make this more intelligent
        print("chose translation " + files[0])
        translator.load(files[0])
        qApp.installTranslator(translator)
        # self._translator.load("language")

        self.__root_item = RootItem(self.__logger, "abstract", self)

        self.__parent = parent
        self.__model_lock = Lock()

        self.__identifier_dict = {"root": self.__root_item}
        self.__item_delegate = SizeDelegate()

        self.__mapping = {
            0: 'type',
            1: 'name',
            2: 'state',
            3: 'data'
        }
        
        self.__last_time_error_occured = 0
        self.__logger.log("info", Time.now(), "ROSModel", "ROSModel initialization finished")

        self.__seuid_helper = SEUID()

        self.__find_host = HostLookup()

        self.__buffer_thread = BufferThread(self)
        

    def get_overview_data_since(self, time=None):
        """
        Return the info needed for the OverviewWidget as a dict.

        :param time: the lower bound from the intervall
        :type time: rospy.Time
        
        :return: the overview data 
        :rtype: dict of values
        """
        if time is None:
            data_dict = self.__root_item.get_latest_data()
        else:
            data_dict = self.__root_item.get_items_younger_than(time)

        return data_dict


    def data(self, index, role=Qt.DisplayRole):
        """
        Returns the data of an item at the given index.

        :param index: the position from which the data is wanted
        :type index: QModelIndex
        :param role: the role that should be used
        :type role: int
        """
        if index is not None:
            if not index.isValid():
                return None
            elif role != Qt.DisplayRole:
                return None

            item = index.internalPointer()
            if item is None:
                raise IndexError("item is None")
            if self.__mapping[index.column()] is "data":
                return item.get_short_data()
            return item.get_latest_data(self.__mapping[index.column()])[self.__mapping[index.column()]]	  
        return None


    def flags(self, index):
        """
        Returns the flags of the item at the given index (like Qt::ItemIsEnabled).

        :param index: the index of the item
        :type index: QModelIndex
        
        :returns: the flags 
        :rtype: ItemFlags
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
        
        :returns: the header data 
        :rtype: QVariant
        """
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            if section is 0:
                return " " + self.tr('Type')
            elif section is 1:
                return " " + self.tr('Name')
            elif section is 2:
                return " " + self.tr('State')
            else:
                return " " + self.tr('Data')
        return None


    def index(self, row, column, parent):
        """
        Returns the index of an item at the given column/row.

        :param row: the index of the row
        :type row: int
        :param column: the index of the column
        :type column: int
        :param parent: the parent 
        :type parent: QModelIndex
        
        :returns: the index 
        :rtype: QModelIndex
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
        Returns the QModelIndex of the parent from the child item specied via its index.

        :param index: the index of the child
        :type index: QModelIndex
        
        :returns: the parent 
        :rtype: QModelIndex
        """
        if not index.isValid():
            return QModelIndex()

        child_item = index.internalPointer()
        parent_item = child_item.parent()

        if parent_item == self.__root_item:
            return QModelIndex()

        return self.createIndex(parent_item.row(), 0, parent_item)


    def rowCount(self, parent):
        """
        Returns the amount of rows in the model.

        :param parent: the parent
        :type parent: QModelIndex
        
        :returns: the nuber amount of rows 
        :rtype: int
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

        :param parent: the parent
        :type parent: QModelIndex
        
        :returns: int
        """
        if parent.isValid():
           return parent.internalPointer().column_count()
        else:
           return self.__root_item.column_count()


    def update_model(self, rated_statistics, topic_statistics, host_statistics, node_statistics):
        """
        Updates the model by using the items of the list. The items will be of the message types.

        :param rated_statistics: the rated_statistics buffer
        :type rated_statistics: list
        :param topic_statistics: the topic_statistics buffer
        :type topic_statistics: list
        :param node_statistics: the node_statistics buffer
        :type node_statistics: list
        :param host_statistics: the host_statistics buffer
        :type host_statistics: list
        """
        self.layoutAboutToBeChanged.emit()

        amount_of_entries = 0
        for item in self.__identifier_dict.values():
            if item is not self.__root_item:
                amount_of_entries += item.get_amount_of_entries()

        # enables "intellgent" updates when there are only few elements in the model so that most of the history is kept
        # maybe use something loglike for the range
        for i in range(5, 0, -1):
            if amount_of_entries > MAXIMUM_AMOUNT_OF_ENTRIES:
                for item in self.__identifier_dict.values():
                    if item is not self.__root_item:
                        item.delete_items_older_than(Time.now() - Duration(secs=i * MINIMUM_RECORDING_TIME))

        if self.__root_item.get_amount_of_entries() > MAXIMUM_AMOUNT_OF_ENTRIES:
            self.__root_item.delete_items_older_than(Time.now() - Duration(secs=360))

        # in order of their appearance in the treeview for always having valid parents
        for item in host_statistics:
            self.__transform_host_statistics_item(item)

        for item in node_statistics:
            self.__transform_node_statistics_item(item)

        for item in topic_statistics:
            self.__transform_topic_statistics_item(item)

        #rating last because it needs the time of the items before
        for item in rated_statistics:
            self.__transform_rated_statistics_item(item)

        data_dict = {
            "state": "ok",
            "total_traffic": 0,
            "connected_hosts": 0,
            "connected_nodes": 0,
            "topic_counter": 0,
            "connection_counter": 0,
            "cpu_usage_max": 0,
            "cpu_temp_mean": 0,
            "ram_usage_mean": 0,
            "cpu_usage_mean": 0,
            "cpu_temp_max": 0,
            "ram_usage_max": 0,
        }

        connected_hosts = 0
        connected_nodes = 0
        topic_counter = 0
        connection_counter = 0
        state = "ok"

        #todo: remove all items older than 5 minuten (or similar) here!
        #for item in self.__identifier_dict:
        #   item.delete_items_older_than(Time.now() - Duration(secs=360))

        #time window
        #todo: where should total_traffic be calculated? currently sum of the bandwidth of the nics
        #todo: extract in own method????
        #generate the general information
        for host_item in self.__root_item.get_childs():
            #hostinfo
            connected_hosts += 1
            if host_item.get_state() is "warning" and state is not "error":
                state = "warning"
            elif host_item.get_state() is "error":
                state = "error"

            last_entry = {}
            data = host_item.get_items_younger_than(Time.now() - Duration(secs=10), "bandwidth_mean", "cpu_usage_max",
                                                    "cpu_temp_mean", "cpu_usage_mean", "cpu_temp_max", "ram_usage_max",
                                                    "ram_usage_mean")
            if data["window_stop"]:
                for key in data:
		    if key is not "window_stop":
                        last_entry[key] = data[key][-1]
            else:
                data = host_item.get_latest_data("bandwidth_mean", "cpu_usage_max", "cpu_temp_mean", "cpu_usage_mean",
                                             "cpu_temp_max", "ram_usage_max", "ram_usage_mean")
                for key in data:
                    last_entry[key] = data[key]

            for key in last_entry:
                if last_entry[key]:
                    if key is "bandwidth_mean":
                        for entry in last_entry[key]:
                            if type(entry) is not unicode:
                                if entry is not 0:
                                    data_dict["total_traffic"] += entry
                    elif key is "cpu_temp_max" or key is "cpu_temp_mean":
                        # very unprobably the temp might be 0 then the programm is not showing this value!
                        if type(last_entry[key]) is not unicode:
                            if last_entry[key] is not 0:
                                data_dict[key] += last_entry[key]
                    else:
                        if type(last_entry[key]) is not unicode:
                            if last_entry[key] is not 0:
                                data_dict[key] += last_entry[key]
            for node_item in host_item.get_childs():
                #nodeinfo
                connected_nodes += 1

                if node_item.get_state() is "warning" and state is not "error":
                    state = "warning"
                elif node_item.get_state() is "error":
                    state = "error"

                for topic_item in node_item.get_childs():
                    #topic info
                    topic_counter += 1

                    if topic_item.get_state() is "warning" and state is not "error":
                        state = "warning"
                    elif topic_item.get_state() is "error":
                        state = "error"

                    for connection_item in topic_item.get_childs():
                        #connection info
                        connection_counter += 1

                        if connection_item.get_state() is "warning" and state is not "error":
                            state = "warning"
                        elif connection_item.get_state() is "error":
                            state = "error"

        data_dict["connected_hosts"] = connected_hosts
        data_dict["connected_nodes"] = connected_nodes
        data_dict["topic_counter"] = topic_counter
        data_dict["connection_counter"] = connection_counter
        data_dict["state"] = state
        data_dict["window_end"] = Time.now()

        #now give this information to the root :)
        self.__root_item.append_data_dict(data_dict)

        self.layoutChanged.emit()


    def __transform_rated_statistics_item(self, item):
        """
        Integrates RatedStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the RatedStatisics item
        :type item: RatedStatistics
        """
        # get identifier
        seuid = item.seuid
        # check if avaiable
        if seuid not in self.__identifier_dict:
            #having a problem, item doesn't exist but should not be created here
            pass
        else:
            #update it
            current_item = self.__identifier_dict[seuid]
            current_item.update_rated_data(item)


    def __transform_topic_statistics_item(self, item):
        """
        Integrates TopicStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the TopicStatistics item
        :type item: TopicStatisticsStatistics
        """
        # get identifier
        topic_seuid = "t" + SEUID_DELIMITER + item.topic
        connection_seuid = "c" + SEUID_DELIMITER + item.node_sub + SEUID_DELIMITER + item.topic \
                              + SEUID_DELIMITER + item.node_pub
        topic_item = None
        connection_item = None
        # check if avaiable
        if topic_seuid not in self.__identifier_dict:
            #creating a topic item
            try:
                parent = self.__identifier_dict[item.node_pub]
            except KeyError:
                host_seuid = "h" + SEUID_DELIMITER + self.__find_host.get_host(item.node_pub)
                host_item = None
                if host_seuid not in self.__identifier_dict:
                    host_item = HostItem(self.__logger, host_seuid, self.__root_item)
                    self.__identifier_dict[host_seuid] = host_item
                    self.__root_item.append_child(host_item)
                else:
                    host_item = self.__identifier_dict[host_seuid]

                node_seuid = "n" + SEUID_DELIMITER + item.node_pub
                node_item = None
                if node_seuid not in self.__identifier_dict:
                    node_item = NodeItem(self.__logger, node_seuid, host_item)
                    self.__identifier_dict[node_seuid] = node_item
                    host_item.append_child(node_item)
                else:
                    node_item = self.__identifier_dict[node_seuid]

                parent = self.__identifier_dict["n" + SEUID_DELIMITER + item.node_pub]
            if parent is None:
                # having a problem, there is no node with the given name
                raise UserWarning("The parent of the given topic statistics item cannot be found.")

            topic_item = TopicItem(self.__logger, topic_seuid, item, parent)
            parent.append_child(topic_item)
            self.__identifier_dict[topic_seuid] = topic_item
            #creating a connection item
            connection_item = ConnectionItem(self.__logger, connection_seuid, topic_item)
            topic_item.append_child(connection_item)
            self.__identifier_dict[connection_seuid] = connection_item
        elif connection_seuid not in self.__identifier_dict:
            topic_item = self.__identifier_dict[topic_seuid]
            #creating a new connection item
            connection_item = ConnectionItem(self.__logger, connection_seuid, topic_item)
            topic_item.append_child(connection_item)
            self.__identifier_dict[connection_seuid] = connection_item
        else:
            # get topic and connection item
            topic_item = self.__identifier_dict[topic_seuid]
            connection_item = self.__identifier_dict[connection_seuid]
            if topic_item is None or connection_item is None:
                for item in self.__identifier_dict:
                    if self.__identifier_dict[item] is None:
                        print(item)
                raise UserWarning("The parent of the given topic statistics item cannot be found.")

        # now update these
        connection_item.append_data(item)
        topic_item.append_data(item)


    def __transform_node_statistics_item(self, item):
        """
        Integrates NodeStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the NodeStatistics item
        :type item: NodeStatistics
        """
        item_seuid = "n" + SEUID_DELIMITER + item.node
        if item_seuid not in self.__identifier_dict:
            #create item
            host_seuid = "h" + SEUID_DELIMITER + item.host
            host_item = None
            if host_seuid not in self.__identifier_dict:
                host_item = HostItem(self.__logger, host_seuid, self.__root_item)
                self.__identifier_dict[host_seuid] = host_item
                self.__root_item.append_child(host_item)
            else:
                host_item = self.__identifier_dict[host_seuid]
            node_item = NodeItem(self.__logger, item_seuid, host_item)
            self.__identifier_dict[item_seuid] = node_item
            host_item.append_child(node_item)
        else:
            node_item = self.__identifier_dict[item_seuid]

        node_item.append_data(item)


    def __transform_host_statistics_item(self, item):
        """
        Integrates HostStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the HostStatistics item
        :type item: HostStatistics
        """
        #print("got host data")
        host_item = None
        item_seuid = "h" + SEUID_DELIMITER + item.host
        if item_seuid not in self.__identifier_dict:
            #create item
            host_item = HostItem(self.__logger, item_seuid, self.__root_item)
            self.__identifier_dict[item_seuid] = host_item
            self.__root_item.append_child(host_item)
        else:
            host_item = self.__identifier_dict[item_seuid]

        host_item.append_data(item)


    def get_item_by_seuid(self, seuid):
        """
        Returns an item according to the given seuid.

        :param seuid: the seuid of the wanted item
        :type seuid: str

        :returns: the item 
        :rtype: AbstractItem
        """
        try:
            return self.__identifier_dict[seuid]
        except KeyError:
            print("There is no item with this seuid")
            raise


    def __get_item_by_seuid(self, seuid, current_item):
        """
        Returns an item according to the given seuid and the currently selected item.

        :param seid: the seuid of the wanted item
        :type seuid: str
        :param current_item: the currently selected item
        :type current_item: Abstractitem

        :retuns: the item
        :rtype: AbstractItem
        """
        if current_item.get_seuid() == seuid:
            return current_item
        for item in current_item.get_childs():
            self.__get_item_by_name(item.seuid, item)
        return None


    def get_log_model(self):
        """
        Returns the log_model.

        :returns: the log-model 
        :rtype: QStandardItemModel
        """
        return self.__log_model

    def get_logger(self):
        """
        Returns the logger of the model.
        
        :returns: the logger
        :rtype: ModelLogger
        """
        return self.__logger

    def get_overview_text(self):
        """
        Returns the overview-data for the overview-widget.

        :returns: the overview-data 
        :rtype: str
        """
        return self.__root_item.get_detailed_data()


    def get_root_item(self):
        """
        returns the root item of the ROSModel.

        :returns: the root-item 
        :rtype: AbstractItem
        """
        return self.__root_item

    def get_translator(self):
        """
        returns the translator.

        :return: the translator
        :rtype: QTranslator
        """
        return self._translator