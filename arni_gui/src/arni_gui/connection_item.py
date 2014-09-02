from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation


class ConnectionItem(AbstractItem):
    """
    A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on
    """

    def __init__(self, logger, seuid, parent=None):
        """
        Initializes the ConnectionItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param type: the type of the item
        :type type: str
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        AbstractItem.__init__(self, logger, seuid, parent)
        self.__parent = parent
        self._type = "connection"

        self._attributes = []
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                                 "stamp_age_stddev", "stamp_age_max"])

        for item in self._attributes:
            self._add_data_list(item)

        # todo: do these really not get any rating?!?
        for element in ["traffic", "stamp_age_mean", "stamp_age_stddev", "stamp_age_max"]:
            self._attributes.remove(element)

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new ConnectionItem")


    def execute_action(self, action):
        """
        Not senseful, Connection cannot execute actions.

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass


    def get_detailed_data(self):
        """
        Returns the detailed data of the ConnectionItem.
        
        :returns: str
        """
        # todo: fill the content sensefully!
        data_dict = self.get_latest_data()

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) + " " \
                   + self.tr("dropped_msgs_unit") + " <br>"
        content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) + " " \
                   + self.tr("traffic_unit") + " <br>"
        content += self.tr("period_mean") + ": " + prepare_number_for_representation(data_dict["period_mean"]) \
                   + " " + self.tr("period_mean_unit") + " <br>"
        content += self.tr("period_stddev") + ": " + prepare_number_for_representation(data_dict["period_stddev"]) \
                   + " " + self.tr("period_stddev_unit") + " <br>"
        content += self.tr("period_max") + ": " + prepare_number_for_representation(data_dict["period_max"]) + " " \
                   + self.tr("period_max_unit") + " <br>"
        content += self.tr("stamp_age_mean") + ": " + prepare_number_for_representation(data_dict["stamp_age_mean"]) \
                   + " " + self.tr("stamp_age_mean_unit") + " <br>"
        content += self.tr("stamp_age_stddev") + ": " + prepare_number_for_representation(data_dict["stamp_age_stddev"]) \
                   + " " + self.tr("stamp_age_stddev_unit") + " <br>"
        content += self.tr("stamp_age_max") + ": " + prepare_number_for_representation(data_dict["stamp_age_max"]) \
                   + " " + self.tr("stamp_age_max_unit") + " <br>"
        content += "</p>"

        return content

    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["dropped_msgs", "traffic", "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                "stamp_age_stddev", "stamp_age_max"]


    def get_short_data(self):
        """
        Returns a shortend version of the item data.
        
        :returns: data of the item
        :rtype: str
        """
        data_dict = self.get_latest_data()

        content = ""
        if data_dict["state"] is "error":
            content += self.get_erroneous_entries().replace("<br>", " - ")
            pass
        else:
            content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(
                data_dict["dropped_msgs"]) + " " \
                       + self.tr("dropped_msgs_unit") + " - "
            content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) + " " \
                       + self.tr("traffic_unit") + " - "
            content += self.tr("period_mean") + ": " + str(data_dict["period_mean"]) \
                       + " " + self.tr("period_mean_unit") + "  - "
            content += self.tr("stamp_age_mean") + ": " + str(data_dict["stamp_age_mean"]) \
                       + " " + self.tr("stamp_age_mean_unit")

        return content

    def get_list_items(self):
        return []

    def get_time_items(self):
        return ["period_mean", "period_stddev", "period_max", "stamp_age_mean",
                "stamp_age_stddev", "stamp_age_max"]
