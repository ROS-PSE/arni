from abstract_item import AbstractItem

class TopicItem(AbstractItem):


    """A TopicItem represents a specific topic which contains many connections and has attributes like the number of sent messages"""


    def __init__(self, seuid, parent=None):
        """Initializes the ConnectionItem

        :param list: connection list
        :type list: list
        :param parent: the parent-object
        :type parent: object
        """
        super(TopicItem, self).__init__(seuid, parent)
        self.__type = "topic"

        #add the content
        self._attributes.extend([#"delivered_msgs",
                                 "dropped_msgs",
                                 "traffic", "stamp_age_mean", "stamp_age_stddev", "stamp_age_max"])

        for item in self._attributes:
            self._add_data_list(item)

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)
#todo: make the append_data methods "intelligent" here? buffer data or similar?
        #yes, pleaaaaaaaaaaaaaaaaaaaaaase!

    def execute_action(self, action):
        """Not senseful, throws an exception

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass

    def get_detailed_data(self):
        #todo: fill the content sensefully!
        data_dict = self.get_latest_data()

        content = "<p style=\"font-size:15px\">"

        content += "dropped_msgs: " + str(data_dict["dropped_msgs"]) + "<br>"
        content += "traffic: " + str(data_dict["traffic"]) + "<br>"
        # content += "connected_nodes:" + str(data_dict["connected_nodes"]) + "<br>"
        # content += "topic_counter" + str(data_dict["topic_counter"]) + "<br>"
        # content += "connection_counter: " + str(data_dict["connection_counter"]) + "<br>"
        # content += "cpu_usage_max: " + str(data_dict["cpu_usage_max"]) + "<br>"
        # content += "cpu_temp_mean: " + str(data_dict["cpu_temp_mean"]) + "<br>"
        # content += "average_ram_load: " + str(data_dict["average_ram_load"]) + "<br>"
        # content += "cpu_usage_mean:" + str(data_dict["cpu_usage_mean"]) + "<br>"
        # content += "cpu_temp_max: " + str(data_dict["cpu_temp_max"]) + "<br>"
        # content += "ram_usage_max: " + str(data_dict["ram_usage_max"]) + "<br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        return ["dropped_msgs", "traffic"]