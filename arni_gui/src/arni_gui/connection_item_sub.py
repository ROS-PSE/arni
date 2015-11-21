from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation


class ConnectionItemSub(AbstractItem):
    """
    A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on
    """

    def __init__(self, logger, seuid, first_message, parent=None):
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
        self._type = "connection-sub"
        
        self.__publisher = self.seuid.split("!")

        self._attributes = []
        self._logger.log("info", Time.now(), seuid, "Created a new ConnectionItemSub")

    def get_items_younger_than(self, time, *args):
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
        return_values["null"] = None
      
        return return_values


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
        content = "<p class=\"detailed_data\">"
        
        content += "Publisher: "
        content += self.__publisher[3][:-5]
        
        content += "</p>"

        return content
      

    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["null", "None"]


    def get_short_data(self):
        """
        Returns a shortend version of the item data.
        
        :returns: data of the item
        :rtype: str
        """
        content = ""
        
        content += "Publisher: "
        content += self.__publisher[3][:-5]
        
        return content


    def get_list_items(self):
        return ["null"]


    def get_time_items(self):
        return [""]

    def aggregate_data(self, period):
        """
        :param period: The amount in seconds over which the data should be aggregated.
        :return:
        """

        values = {}
        for key in self._attributes:
            values[key] = 0

        return values