from abstract_item import AbstractItem


class ConnectionItem(AbstractItem):
    """A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on"""

    def __init__(self, seuid, parent=None):
        """Initializes the ConnectionItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param type: the type of the item
        :type type: str
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        super(ConnectionItem, self).__init__(seuid, parent)
        self.__parent = parent
        self._type = "connection"

        # add the content
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max"])
	#, "stamp_age_mean", "stamp_age_stddev",
                                 #"stamp_age_max"])

        for item in self._attributes:
            self._add_data_list(item)

        self._attributes.remove("traffic")
        #self._attributes.append("bandwidth")

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)


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

        content = "<p style=\"font-size:15px\">"

        content += "dropped_msgs: " + str(data_dict["dropped_msgs"]) + "<br>"
        content += "traffic: " + str(data_dict["traffic"]) + "<br>"
        content += "period_mean:" + str(data_dict["period_mean"]) + "<br>"
        content += "period_stddev" + str(data_dict["period_stddev"]) + "<br>"
        content += "period_max: " + str(data_dict["period_max"]) + "<br>"
        content += "stamp_age_mean: " + str(data_dict["stamp_age_mean"]) + "<br>"
        content += "stamp_age_stddev: " + str(data_dict["stamp_age_stddev"]) + "<br>"
        content += "stamp_age_max: " + str(data_dict["stamp_age_max"]) + "<br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        #todo: append more items here
        return ["traffic", "dropped_msgs", "period_mean"]

    def get_short_data(self):
        return "connection_item"
