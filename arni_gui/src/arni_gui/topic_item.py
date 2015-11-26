from rospy.rostime import Time
import rospy

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation, UPDATE_FREQUENCY, TOPIC_AGGREGATION_FREQUENCY
from arni_core.helper import SEUID, SEUID_DELIMITER
from node_item import NodeItem

from rospy.timer import Timer
from rospy.impl.tcpros_service import ServiceProxy
from rospy.rostime import Duration
from rospy.rostime import Time
from connection_item import ConnectionItem


import re

class TopicItem(AbstractItem):
    """
    A TopicItem represents a specific topic which contains many connections and has attributes like the number of sent messages.
    """

    def __init__(self, logger, seuid, first_message, parent=None):
        """Initializes the TopicItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        AbstractItem.__init__(self, logger, seuid, parent)
        self.__parent = parent
        self._type = "topic"

        self.add_keys=["dropped_msgs", "traffic", "bandwidth", "frequency"]
        self.avg_keys=["period_mean", "period_stddev", "stamp_age_mean", "stamp_age_stddev"]
        self.max_keys=["period_max", "stamp_age_max"]

        self._attributes = []
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                                 "stamp_age_stddev", "stamp_age_max", "bandwidth", "frequency"])
       #self._attributes.extend(["dropped_msgs", "traffic",
        #                         "stamp_age_mean", "stamp_age_max", "stamp_age_stddev", "period_max", "node_pub", "node_sub"])


        for item in self._attributes:
            self._add_data_list(item)

        #self._attributes.remove("traffic")
        #self._attributes.append("bandwidth")

        self.__rated_attributes = []
        self.__rated_attributes.append("alive.actual_value")
        self.__rated_attributes.append("alive.expected_value")
        self.__rated_attributes.append("alive.state")         

        self.__calculated_data = {}
        for key in self._attributes:
            self.__calculated_data[key] = []

        self.__calculated_data["window_start"] = []
        self.__calculated_data["window_stop"] = []

        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new TopicItem")

        self.__timer = Timer(Duration(nsecs=TOPIC_AGGREGATION_FREQUENCY), self.__aggregate_topic_data)

    # def is_subscriber(self, item, parent):
    #     """
    #     Checks for a ConnectionItem if it is
    #
    #     :param item: the item which shall be checked
    #     :type item: ConnectionItem
    #     :type parent: NodeItem
    #     :param parent: the node of the connection in the GUI
    #     :return:
    #     """
    #     if not isinstance(item, ConnectionItem):
    #         raise UserWarning()
    #     if not item in self.get_childs():
    #         raise UserWarning()
    #     else:
    #         seuid = parent.get_seuid()
    #         seuid_helper = SEUID()
    #         seuid_helper.identifier = seuid
    #         seuid_helper.set_fields()
    #         node = seuid_helper.node
    #
    #         child_seuid = item.get_seuid()
    #         seuid_helper.identifier = child_seuid
    #         seuid_helper.set_fields()
    #         node_comp = seuid_helper.publisher
    #         if node == node_comp:
    #             return False
    #         node_comp = seuid_helper.subscriber
    #
    #         if node == node_comp:
    #             return True
    #         else:
    #             raise UserWarning()


    def get_child(self, row, parent=None):
        """
        Returns the child at the position row.

        :param row: the index of the row
        :type row: int
        :param parent: the model parent at the given index (not global / logical parent)
        :type parent: NodeItem

        :returns: the child at the position row
        :rtype: AbstractItem
        """
        if not isinstance(parent, NodeItem):
            print(type(parent))
            raise UserWarning
        return self.__get_local_childs(parent)[row]

    def __get_local_childs(self, parent=None):
        """
        Returns all childs of the topic item at the given position in the gui.

        :param parent: the model parent at the given index (not global / logical parent)
        :type parent: NodeItem
        :param sub_activated: Defines if subscriber shall be shown too.
        :returns: the child at the position row
        :rtype: AbstractItem
        """
        childs = []
        if parent is not None:
            # a specific parent has been chosen - we have to use it to display the correct connection items
            # use the seuid to determine the node and compare this to the parts in the connections item (child of this
            # item.
            seuid = parent.get_seuid()

            seuid_helper = SEUID()
            seuid_helper.identifier = seuid
            seuid_helper.set_fields()
            node = seuid_helper.node
            for child in self.get_childs():
                child_seuid = child.get_seuid()
                seuid_helper.identifier = child_seuid
                seuid_helper.set_fields()
                node_comp = seuid_helper.publisher

                if seuid == "n!/mole":
                    print(seuid)
                    print(child_seuid)
                    print(self.seuid)
                    print(node)
                    print(node_comp)

                # do the check on the publisher
                if node == node_comp:
                    # match.
                    childs.append(child)
                    child.show_as_subscriber = False
                    continue

                node_comp = seuid_helper.subscriber

                if node == node_comp:
                    print("match2")
                    # match.
                    childs.append(child)
                    # adds a state to the connection so that it is shown in the gui as a subscriber
                    child.show_as_subscriber = True
            return childs
        else:
            print("none")
            print(self.seuid)
            print(len(self._child_items))
            return self._child_items

    def row(self, parent=None):
        """
        Returns the index of the Item.

        :returns: the index of the Item
        :rtype: int
        """
        if self.__parent:
            return self.__parent.get_childs().index(self)
        return 0


    def child_count(self, parent=None):
        """
        Returns the number of children from the AbstractItem.

        :returns: number of childs
        :rtype: int
        """
        return len(self.__get_local_childs(parent))

    def get_childs(self):
        """
        Returns a list with all children.
        WARNING: This is the same method as in AbstractItem (superclass) to warn you using this function in the gui
        context. Topic item shows only some connections depending on the parent node. This is *not* implemented for
        this function.

        :returns: list of children
        :rtype: list
        """
        return self._child_items



    def get_items_younger_than(self, time, *args):
        """
        Used to overwrite the standard implementation in AbstractItem. This method provides the data from the
        calculated data and *not* from the raw input. This is especially wanted when plotting
        :param time:
        :param args:
        :return:
        """
        self._data_lock.acquire()
        return_values = {}

        if args:
            for key in args:
                return_values[key] = None
            if "window_stop" not in args:
                return_values["window_stop"] = None
        else:
            for key in self.__calculated_data:
                return_values[key] = None

        breakpoint = 0
        list_of_time = self.__calculated_data["window_stop"]
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[0] >= time:
                for key in return_values:
                    try:
                        return_values[key] = self.__calculated_data[key][:]
                    except KeyError:
                        print("Accessed key was: " + key + ". Available keys are: ")
                        print(self.__calculated_data)
                        raise
            else:
                for i in range(length - 1, -1, -1):
                    if list_of_time[i] < time:
                        breakpoint = i + 1
                        for key in return_values:
                            if key in self.__calculated_data:
                                return_values[key] = self.__calculated_data[key][breakpoint:length]
                            else:
                                raise IndexError("IndexError! length of the list %s, accessed index %s. length of data"
                                                 " at given point %s, key is %s", length, i,
                                                 len(self.__calculated_data[key]), key)
                        break

        self._data_lock.release()
        return return_values


    def get_raw_items_younger_than(self, time, *args):
        """
        Returns all entries that are younger than time either in all keys of self._data or if args not empty in
        all key corresponding to args.
        Warning: Method assumes data is sorted by time if this is not true will return too few or too much data.

        :param time: the lower bound in seconds
        :type time: rospy.Time
        :param args: the keys to the dict
        :type args: str

        :returns: dict of lists
        :rtype: dict
        :raises KeyError: if an element in args cannot be found in any of the dictionaries (data vs rated data)
        """
        return_values = {}

        if args:
            for key in args:
                return_values[key] = None
            if "window_stop" not in args:
                return_values["window_stop"] = None
        else:
            for key in self._data:
                return_values[key] = None

        breakpoint = 0
        list_of_time = self._data["window_stop"]
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[0] >= time:
                for key in return_values:
                    try:
                        return_values[key] = self._data[key][:]
                    except KeyError:
                        print("Accessed key was: " + key + ". Available keys are: ")
                        print(self._data)
                        raise
            else:
                for i in range(length - 1, -1, -1):
                    if list_of_time[i] < time:
                        breakpoint = i + 1
                        for key in return_values:
                            if key in self._data:
                                return_values[key] = self._data[key][breakpoint:length]
                            else:
                                raise IndexError("IndexError! length of the list %s, accessed index %s. length of data"
                                                 " at given point %s, key is %s", length, i, len(self._data[key]), key)
                        break
        return return_values


    def __aggregate_topic_data(self, event):
        """
        Aggregates the topic every TOPIC_AGGREGATION_FREQUENCY nsecs and pushes the updated data to
        self.__calculated_data.

        :param event: containing information when this method was called - not used but needed for the interface
        """
        aggregated_data = {}
        for key in self._attributes:
            aggregated_data[key] = 0

        for key in self.__calculated_data.keys():
             self.__calculated_data[key].append(0)

        child_count = 0
        for connection in self.get_childs():  # !assuming all childs are connection items!
            values = connection.aggregate_data(5)  # average over N seconds

            if values:
                for key in self.add_keys:
                     aggregated_data[key] += values[key]
                for key in self.max_keys:
                    if values[key] > aggregated_data[key]:
                         aggregated_data[key] = values[key]
                for key in self.avg_keys:
                     aggregated_data[key] += values[key]
                child_count += 1


        for key in self.avg_keys:
            if child_count == 0:
                aggregated_data[key] = 0
            else:
                aggregated_data[key] /= child_count

        self._data_lock.acquire()

        for key in self._attributes:
            self.__calculated_data[key][-1] = aggregated_data[key]

        self.__calculated_data["window_start"][-1] = Time.now()
        self.__calculated_data["window_stop"][-1] = Time.now() - (Duration(secs=1) if int(Duration(secs=1).to_sec()) <= int(Time.now().to_sec()) else Time(0))


        self._data_lock.release()


    def execute_action(self, action):
        """
        Not senseful, Topics cannot execute actions.

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass


    def get_detailed_data(self):
        """
        Returns the detailed data of the HostItem.

        :returns: detailed data
        :rtype: str
        """
        data_dict = self.get_latest_data()
        for key in self.__calculated_data:
            if self.__calculated_data[key]:
                #print(self.__calculated_data[key][-1])
                data_dict[key] = self.__calculated_data[key][-1]
            else:
                data_dict[key] = self.tr("Currently no value available")

        data_dict["state"] = self.get_state()

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        content += "Rounded to a second:<br>"

        if "frequency" in self._attributes:
            content += self.tr("frequency") + ": " + prepare_number_for_representation(data_dict["frequency"]) \
                   + " " + self.tr("frequency_unit") + " <br>"
        content += self.tr("bandwidth") + ": " + prepare_number_for_representation(data_dict["bandwidth"]) \
                   + " " + self.tr("bandwidth_unit") + " <br>"

        content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) \
                   + " " + self.tr("dropped_msgs_unit") + " <br>"
        content += self.tr("period_max") + ": " + prepare_number_for_representation(data_dict["period_max"]) \
                   + " " + self.tr("period_max_unit") + " <br>"
        content += self.tr("stamp_age_max") + ": " + prepare_number_for_representation(data_dict["stamp_age_max"]) \
                   + " " + self.tr("stamp_age_max_unit") + " <br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.

        :returns: str[]
        """
        if "frequency" in self.__calculated_data:
            return ["dropped_msgs","stamp_age_max", "period_max",
                    "bandwidth", "frequency"]

        else:
            return ["dropped_msgs", "traffic", "stamp_age_max", "period_max", "bandwidth"]


    def get_short_data(self):
        """
        Returns a shortend version of the item data.

        :returns: data of the item
        :rtype: str
        """
        data_dict = {}
        for key in self.__calculated_data:
            if self.__calculated_data[key]:
                data_dict[key] = self.__calculated_data[key][-1]
            else:
                data_dict[key] = self.tr("Currently no value available")

        data_dict["state"] = self.get_state()

        content = ""
        if data_dict["state"] is "error":
            content += self.get_erroneous_entries_for_log()
        else:
            content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(
                data_dict["dropped_msgs"]) + " " \
                       + self.tr("dropped_msgs_unit") + " - "
            content += self.tr("stamp_age_mean") + ": " + prepare_number_for_representation(data_dict["stamp_age_mean"]) \
                       + " " + self.tr("stamp_age_mean_unit")

        return content


    def can_execute_actions(self):
        """
        This item cannot execute actions, so it returns False

        :return: False
        """
        return False


    def get_list_items(self):
        return []


    def get_time_items(self):
        return ["stamp_age_mean", "stamp_age_max"]
