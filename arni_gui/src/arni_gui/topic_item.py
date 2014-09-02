from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation


class TopicItem(AbstractItem):
    """
    A TopicItem represents a specific topic which contains many connections and has attributes like the number of sent messages.
    """

    def __init__(self, logger, seuid, parent=None):
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

        self._attributes = []
        #todo: currently probably only these 4 implemented
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "stamp_age_mean", "stamp_age_max"])

        for item in self._attributes:
            self._add_data_list(item)

        self._attributes.remove("traffic")
        self._attributes.append("bandwidth")

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new TopicItem")


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
        #todo: fill the content sensefully!
        data_dict = self.get_latest_data()

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) \
                   + " " + self.tr("dropped_msgs_unit") + " <br>"
        content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) \
                   + " " + self.tr("traffic_unit") + " <br>"
        content += self.tr("stamp_age_mean") + ": " + str(data_dict["stamp_age_mean"]) \
                   + " " + self.tr("stamp_age_mean_unit") + " <br>"
        content += self.tr("stamp_age_max") + ": " + str(data_dict["stamp_age_max"]) \
                   + " " + self.tr("stamp_age_max_unit") + " <br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["dropped_msgs", "traffic", "stamp_age_mean", "stamp_age_max"]

    def get_short_data(self):
        """
        Returns a shortend version of the item data.
        
        :returns: data of the item
        :rtype: str
        """
        data_dict = self.get_latest_data()

        content = ""
        if data_dict["state"] is not "ok":
            #todo: print the wrong data here
            content += "something is wrong"
            pass
        else:
            content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(
                data_dict["dropped_msgs"]) + " " \
                       + self.tr("dropped_msgs_unit") + " - "
            content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) + " " \
                       + self.tr("traffic_unit") + " - "
            #content += self.tr("period_mean") + ": " + str(data_dict["period_mean"]) \
            #           + " " + self.tr("period_mean_unit") + "  - "
            content += self.tr("stamp_age_mean") + ": " + str(data_dict["stamp_age_mean"]) \
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
