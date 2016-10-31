from python_qt_binding.QtCore import QAbstractItemModel, QModelIndex, QTranslator, Qt, Signal
try:  # Qt4 vs Qt5
  from python_qt_binding.QtGui import qApp
except ImportError:
  from python_qt_binding.QtWidgets import qApp

from threading import Lock
from size_delegate import SizeDelegate
from abstract_item import AbstractItem

from tree_connection_item import TreeConnectionItem
from connection_item import ConnectionItem
# from connection_item_sub import ConnectionItemSub
from topic_item import TopicItem
from host_item import HostItem
from node_item import NodeItem
from root_item import RootItem
from tree_topic_item import TreeTopicItem


import rospy
from rospy.rostime import Duration, Time
from rospy.timer import Timer
import std_msgs.msg

from arni_core.singleton import Singleton
import arni_msgs

from arni_msgs.msg import RatedStatistics
from arni_msgs.msg import NodeStatistics
from arni_msgs.msg import HostStatistics
from arni_core.host_lookup import HostLookup
from arni_msgs.msg import MasterApi


from model_logger import ModelLogger

from arni_core.helper import SEUID, SEUID_DELIMITER, generate_seuids_from_master_api_data

import rosgraph.impl.graph
import rosgraph.masterapi
from rosgraph_msgs.msg import TopicStatistics

_ROS_NAME = ''

import time

from helper_functions import UPDATE_FREQUENCY, MAXIMUM_AMOUNT_OF_ENTRIES, MINIMUM_RECORDING_TIME, \
    topic_statistics_state_to_string, find_qm_files

import rospkg
import os

from buffer_thread import *



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
    update_signal = Signal()

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
        # todo: make this more intelligent (should be done as soon as new languages are needed / implemented)
        print("chose translation " + files[0])
        translator.load(files[0])
        qApp.installTranslator(translator)

        self.__root_item = RootItem(self.__logger, "abstract", self)

        self.__parent = parent
        self.__model_lock = Lock()

        self.update_signal.connect(self.update_model)


        """
        IMPORTANT: Does not contain the models nodes but the logical nodes. E.g. a ConnectionItem splits up to two
        TreeConnectionItems where the identifier_dict only contains the connectionitem. This allows to push
        data into one item but to show it at two places in the Qt GUI.
        """
        self.__identifier_dict = {"root": self.__root_item}
        self.__item_delegate = SizeDelegate()

        # CAUTION: Do not change this mapping if not absolutely necessary. If you change it remember to change
        # item_filter_proxy (and maybe other classes) as well - sadly not all functions use the access function.
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
        self.__buffer_thread.start()

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

        if isinstance(parent_item, TreeTopicItem):
            child_item = parent_item.get_child(row, self.parent(parent).internalPointer())
        else:
            child_item = parent_item.get_child(row)
        if child_item:
            return self.createIndex(row, column, child_item)
        else:
            return QModelIndex()

    def parent(self, index):
        """
        Returns the QModelIndex of the parent from the child item specified via its index.

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

        # row of treetopicItems depends on their position in the model..
        if isinstance(parent_item, TreeTopicItem):
            return self.createIndex(parent_item.row(parent_item.parent()), 0, parent_item)
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

        if isinstance(parent_item, TreeTopicItem):
            return parent_item.child_count(parent_item.parent())
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

    def update_model(self):
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
        :param master_api_data: data from the master api
        :type master_api_data: MasterApi
        """
        self.__model_lock.acquire()
        self.layoutAboutToBeChanged.emit()

        if self.__buffer_thread:
            rated_statistics, topic_statistics, host_statistics, node_statistics, master_api_data = self.__buffer_thread.get_state()

            amount_of_entries = 0
            for item in self.__identifier_dict.values():
                if item is not self.__root_item:
                    amount_of_entries += item.get_amount_of_entries()

            # enables "intelligent" updates when there are only few elements in the model so that most of the history is kept
            # maybe use something loglike for the range
            for i in range(5, 0, -1):
                if amount_of_entries > MAXIMUM_AMOUNT_OF_ENTRIES:
                    for item in self.__identifier_dict.values():
                        if item is not self.__root_item:
                            item.delete_items_older_than(Time.now() - (Duration(secs=i * MINIMUM_RECORDING_TIME) if int(
                                Duration(secs=i * MINIMUM_RECORDING_TIME).to_sec()) <= int(Time.now().to_sec()) else Time(
                                0)))

            if self.__root_item.get_amount_of_entries() > MAXIMUM_AMOUNT_OF_ENTRIES:
                self.__root_item.delete_items_older_than(Time.now() - (
                    Duration(secs=360) if int(Duration(secs=360).to_sec()) <= int(Time.now().to_sec()) else Time(0)))

            # in order of their appearance in the treeview for always having valid parents
            for item in host_statistics:
                self.__transform_host_statistics_item(item)

            for item in node_statistics:
                self.__transform_node_statistics_item(item)

            for item in topic_statistics:
                self.__transform_topic_statistics_item(item)

            if master_api_data is not None:
                self.__transform_master_api_data(master_api_data)

            # rating last because it needs the time of the items before
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

            # generate the general information
            for host_item in self.__root_item.get_childs():
                # hostinfo
                connected_hosts += 1
                if host_item.get_state() is "warning" and state is not "error":
                    state = "warning"
                elif host_item.get_state() is "error":
                    state = "error"

                last_entry = {}
                data = host_item.get_items_younger_than(Time.now() - (
                    Duration(secs=10) if int(Duration(secs=10).to_sec()) <= int(Time.now().to_sec()) else Time(0)),
                                                        "bandwidth_mean", "cpu_usage_max",
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
                        elif key is "cpu_temp_max" or key is "ram_usage_max":
                            # very unprobably the temp might be 0 then the programm is not showing this value!
                            if type(last_entry[key]) is not unicode:
                                if last_entry[key] is not 0:
                                    if data_dict[key] < last_entry[key]:
                                        data_dict[key] = last_entry[key]
                        else:
                            if type(last_entry[key]) is not unicode:
                                if last_entry[key] is not 0:
                                    data_dict[key] += last_entry[key]
                for node_item in host_item.get_childs():
                    # nodeinfo
                    connected_nodes += 1

                    if node_item.get_state() is "warning" and state is not "error":
                        state = "warning"
                    elif node_item.get_state() is "error":
                        state = "error"

                    for topic_item in node_item.get_childs():
                        # topic info
                        topic_counter += 1

                        if topic_item.get_state() is "warning" and state is not "error":
                            state = "warning"
                        elif topic_item.get_state() is "error":
                            state = "error"

                        for connection_item in topic_item.get_childs():
                            # connection info
                            connection_counter += 1

                            if connection_item.get_state() is "warning" and state is not "error":
                                state = "warning"
                            elif connection_item.get_state() is "error":
                                state = "error"

            for key in data_dict:
                if key != "state" and key != "cpu_temp_max" and key != "total_traffic" and key != "ram_usage_max" \
                        and self.__root_item.child_count():
                    data_dict[key] /= self.__root_item.child_count()

            data_dict["connected_hosts"] = connected_hosts
            data_dict["connected_nodes"] = connected_nodes
            data_dict["topic_counter"] = topic_counter
            data_dict["connection_counter"] = connection_counter
            data_dict["state"] = state
            data_dict["window_end"] = Time.now()

            # now give this information to the root :)
            self.__root_item.append_data_dict(data_dict)
        self.__model_lock.release()
       # self.__checkIfAlive()

        self.layoutChanged.emit()

    # def __checkIfAlive(self):
    #     """
    #     Checks for any gui element if it is still alive. If not the state "unknown" is changed to "offline".
    #     """
    #     self.__model_lock.acquire()
    #     for item in self.__identifier_dict.values():
    #         if item is not self.__root_item:
    #             # get the timer
    #
    #
    #     self.__model_lock.release()


    def __transform_rated_statistics_item(self, item):
        """
        Integrates RatedStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the RatedStatisics item
        :type item: RatedStatistics
        """
        # get identifier
        seuid = item.seuid
        if item.seuid == "t!/image_raw":
            tmp = self.__identifier_dict[seuid]
            for i in range(0, len(item.rated_statistics_entity)):
                if item.rated_statistics_entity[i].statistic_type == "frequency":
                    print(item.rated_statistics_entity[i].actual_value)
                    print(tmp.get_latest_data()["frequency"])


        # check if avaiable
        if seuid not in self.__identifier_dict:
            # having a problem, item doesn't exist but should not be created here
            self.__logger.log("Warning", Time.now(), "RosModel", "A rating was received for an item the gui does not "
                                                                 "know about. This typically means a dead node/ host but"
                                                                 " could also indicate an error. If this happens at "
                                                                 "startup you can typically ignore it. ")
        else:
            # update it
            current_item = self.__identifier_dict[seuid]
            current_item.update_rated_data(item)

    def __transform_master_api_data(self, master_api_data):
        """
        Used to add further topics that have not been added by __transform_topic_statistics_item. A new TopicItem
        will only be created if it does not yet exist. This is especially the case for existing topics that don't send
        any data.
        :type master_api_data: MasterApi
        """
        # pubs, subs, srvs = master_api_data.pubs, master_api_data.subs, master_api_data.srvs

        seuids = generate_seuids_from_master_api_data(master_api_data)

        for seuid in seuids:
            self.get_or_add_item_by_seuid(seuid)

    def __transform_topic_statistics_item(self, item):
        """
        Integrates TopicStatistics in the model by modifying its item/s by adding a new dict to the corresponding item.

        :param item: the TopicStatistics item
        :type item: TopicStatistics
        """
        topic_seuid = self.__seuid_helper.from_message(item)
        connection_seuid = self.__seuid_helper.from_message(item)

        connection_item = self.get_or_add_item_by_seuid(connection_seuid)
        if connection_item is not None:
            connection_item.append_data(item)

    def __transform_node_statistics_item(self, item):
        """
        Integrates NodeStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the NodeStatistics item
        :type item: NodeStatistics
        """
        node_seuid = self.__seuid_helper.from_message(item)
        node_item = self.get_or_add_item_by_seuid(node_seuid)
        if node_item is not None:
            node_item.append_data(item)


    def __transform_host_statistics_item(self, item):
        """
        Integrates HostStatistics in the model by moding its item/s by adding a new dict to the corresponding item.

        :param item: the HostStatistics item
        :type item: HostStatistics
        """
        host_seuid = self.__seuid_helper.from_message(item)
        host_item = self.get_or_add_item_by_seuid(host_seuid)
        host_item.append_data(item)

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

    def get_or_add_item_by_seuid(self, seuid, node1="", node2=""):
        """
        Takes a seuid, checks if it already exists. If it does, the item is simply returned. If it does not
        a new item is generated and added to the tree. Does return the GUI items which is equal to the model items
        for host and node but different for topic and connections (2 gui items are created, whereas they access one
        model item)

        :param seuid:
        :return:
        """
        if seuid is None:
            raise UserWarning("seuid was None!")
        if seuid not in self.__identifier_dict or seuid[0] is 't':
            parent = None
            item = None
            if seuid[0] == "h":
                item = HostItem(self.__logger, seuid, self.__root_item)
                parent = self.__root_item
                parent.append_child(item)

            elif seuid[0] == "n":
                # does host exist
                # call helper to get the host
                node = self.__seuid_helper.get_field("n", seuid)
                host = self.__find_host.get_host(node)
                if host is None:
                    self.__logger.log("Warning", Time.now(), "RosModel", "The node " + node + " does probably no longer"
                                               " exist (Cannot find its host)."
                                               " Therefore it was not added to the GUI.")
                    item = None
                else:
                    host_seuid = self.__seuid_helper.from_string("h", host)
                    parent = self.get_or_add_item_by_seuid(host_seuid)
                    if parent is None:
                        return None
                    item = NodeItem(self.__logger, seuid, parent)
                    parent.append_child(item)
            elif seuid[0] == "t":
                if node1 == "" or node2 == "":
                    raise UserWarning("node was empty - topic does not know its parent!")
                else:
                    # use node information - first add publisher
                    node_seuid = self.__seuid_helper.from_string("n", node1)
                    parent = self.get_or_add_item_by_seuid(node_seuid)
                    if parent is None:
                        return None
                    if seuid not in self.__identifier_dict:
                        item = TopicItem(self.__logger, seuid, None, parent)
                    else:
                        item = self.__identifier_dict[seuid]

                    found = False
                    for child in parent.get_childs():
                        # add it
                        if child.seuid == seuid:
                            found = True
                    if not found:
                        topic_item1 = TreeTopicItem(parent, item, False)
                        parent.append_child(topic_item1)
                        item.tree_items.append(topic_item1)

                    node_seuid = self.__seuid_helper.from_string("n", node2)
                    parent = self.get_or_add_item_by_seuid(node_seuid)
                    if parent is None:
                        return None

                    found = False
                    for child in parent.get_childs():
                        # add it
                        if child.seuid == seuid:
                            found = True
                    if not found:
                        topic_item2 = TreeTopicItem(parent, item, False)
                        parent.append_child(topic_item2)
                        item.tree_items.append(topic_item2)

            elif seuid[0] == "c":
                topic_seuid = self.__seuid_helper.from_string("t", self.__seuid_helper.get_field("t", seuid))
                if self.__seuid_helper.publisher is None:
                    raise UserWarning()
                # getting back the logical parent - a TopicItem
                pub = self.__seuid_helper.publisher
                sub = self.__seuid_helper.subscriber
                parent = self.get_or_add_item_by_seuid(topic_seuid, pub, sub)
                if parent is None:
                        return None
                item = ConnectionItem(self.__logger, seuid, None, parent)
                parent.append_child(item)


                found = 0
                for tree_item in parent.tree_items:
                    # item is a TreeTopicItem
                    if tree_item.parent().seuid == self.__seuid_helper.from_string("n", pub):
                        found += 1
                        tree_item.tree_item1 = TreeConnectionItem(tree_item, item, False)
                        tree_item.append_child(tree_item.tree_item1)
                    if tree_item.parent().seuid == self.__seuid_helper.from_string("n", sub):
                        found += 1
                        tree_item.tree_item2 = TreeConnectionItem(tree_item, item, True)
                        tree_item.append_child(tree_item.tree_item2)
                if found != 2:
                    raise UserWarning()
            if item is not None:
                self.__identifier_dict[seuid] = item
            return item
        else:
            return self.__identifier_dict[seuid]
