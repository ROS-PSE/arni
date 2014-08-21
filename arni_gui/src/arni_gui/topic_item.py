from abstract_item import AbstractItem

class TopicItem(AbstractItem):


    """A TopicItem represents a specific topic which contains many connections and has attributes like the number of sent messages"""


    def __init__(self, seuid, type, can_execute_actions, parent=None):
        """Initializes the ConnectionItem

        :param list: connection list
        :type list: list
        :param parent: the parent-object
        :type parent: object
        """
        self.__type = "topic"

        AbstractItem.__init__(self, seuid, parent)
        #add the content
        self.__attributes = ["""topic",""" "window_start", "window_stop", "dropped_msgs", "traffic", "period_mean",
                      "period_stddev", "period_max", "stamp_age_mean", "stamp_age_stddev", "stamp_age_max"]
        for item in self.__attributes:
            self.__add_data_list(item)

#todo: make the append_data methods "intelligent" here? buffer data or similar?
        #yes, pleaaaaaaaaaaaaaaaaaaaaaase!

    def execute_action(self, action):
        """Not senseful, throws an exception

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass

