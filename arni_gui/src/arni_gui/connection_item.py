from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem


class ConnectionItem(AbstractItem):
    """A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on"""

    def __init__(self, logger, seuid, parent=None):
        """Initializes the ConnectionItem.
        
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
        # super(ConnectionItem, self).__init__(seuid, parent)
        self.__parent = parent
        self._type = "connection"

        self._attributes = []
        # add the content
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                                 "stamp_age_stddev", "stamp_age_max"])

        for item in self._attributes:
            self._add_data_list(item)

        #todo: do these really not get any rating?!?
        for element in ["traffic", "stamp_age_mean", "stamp_age_stddev", "stamp_age_max"]:
            self._attributes.remove(element)

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

            # del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new ConnectionItem")


    def execute_action(self, action):
        """Not senseful

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

        #todo: add rated data here if wrong!!!

        content += QTranslator.translate("AbstractItem", "dropped_msgs") + ": " + str(data_dict["dropped_msgs"]) + " " \
                   + QTranslator.translate("AbstractItem", "dropped_msgs_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "traffic") + ": " + str(data_dict["traffic"]) + " " \
                   + QTranslator.translate("AbstractItem", "traffic_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "period_mean") + ": " + str(data_dict["period_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "period_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "period_stddev") + ": " + str(data_dict["period_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "period_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "period_max") + ": " + str(data_dict["period_max"]) + " " \
                   + QTranslator.translate("AbstractItem", "period_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "stamp_age_mean") + ": " + str(data_dict["stamp_age_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "stamp_age_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "stamp_age_stddev") + ": " + str(data_dict["stamp_age_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "stamp_age_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "stamp_age_max") + ": " + str(data_dict["stamp_age_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "stamp_age_max_unit") + " <br>"

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
        return "connection_item"

