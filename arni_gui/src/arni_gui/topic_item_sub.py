from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation, UPDATE_FREQUENCY, TOPIC_AGGREGATION_FREQUENCY

from rospy.timer import Timer
from rospy.impl.tcpros_service import ServiceProxy
from rospy.rostime import Duration
from rospy.rostime import Time


class TopicItemSub(AbstractItem):
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
        self._type = "topic-sub"        

        self._attributes = []
        self.__rated_attributes = []
        self.__calculated_data = {}

        self._logger.log("info", Time.now(), seuid, "Created a new TopicItemSub")
        

    def get_items_younger_than(self, time, *args):
        """
        Used to overwrite the standart implementation in AbstractItem. This method provides the data from the
        calculated data and *not* from the raw input. This is especially wanted when plotting
        :param time:
        :param args:
        :return:
        """
        #self._data_lock.acquire()
        return_values = {}
        return_values["null"] = None
      
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
        return_values["null"] = None
      
        return return_values


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
        return ""


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
        return ""


    def can_execute_actions(self):
        """
        This item cannot execute actions, so it returns False

        :return: False
        """
        return False


    def get_list_items(self):
        return ["null"]


    def get_time_items(self):
        return [""]
