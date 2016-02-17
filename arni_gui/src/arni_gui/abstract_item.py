from threading import Lock
import time as tm

from rospy.rostime import Duration, Time
import rospy

from python_qt_binding.QtCore import QTranslator, QObject

from helper_functions import prepare_number_for_representation, topic_statistics_state_to_string, \
    ALIVE_TIMER_CALLBACK, MAXIMUM_OFFLINE_TIME, WARNING_TIMEOUT


class AbstractItem(QObject):
    """ 
    Provides a unified interface to access the items of the model.
    INTERNAL: WARNING! Whenever the key-values at the beginning are not set right, the oddest things may occur!
    """

    def __init__(self, logger, seuid, parent=None):
        """
        Initializes the AbstractItem.
        
        :param seuid: the seuid of the AbstractItem
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        super(AbstractItem, self).__init__(parent)
        self._logger = logger
        self._data = {}

        self.counter = 0
        """
        _rated_data is dict containing the rated data. state, window_start and window_end are simply lists
         with the corresponding entries. Any other values typically is a list containing lists which however contain the
         values. This is equivalent to the representation in the RatedStatistics/Entity.
        """
        self._rated_data = {}
        self._child_items = []
        self.__parent = parent
        self.seuid = seuid
        self._type = "type"
        self.__data_attribute = "data"
        self.__state = []
        # self.__last_update = Time.now()
        self.__creation_time = Time.now()

        self.marked = False
        # self.markation_date = Time.now()

        self._add_data_list("window_start")
        self._add_data_list("window_stop")
        self._add_rated_data_list("window_start")
        self._add_rated_data_list("window_stop")

        self._length_of_data = 0
        self._length_of_rated_data = 0

        self._data_lock = Lock()
        self._rated_data_lock = Lock()
        self._rated_attributes = []
        self._rated_attributes.append("alive.actual_value")
        self._rated_attributes.append("alive.expected_value")
        self._rated_attributes.append("alive.state")

        #self._alive_timer = rospy.Time.now()
        #self.alive = True
        #rospy.Timer(rospy.Duration(ALIVE_TIMER_CALLBACK), self._updateTimer)
        #self._offline_time = rospy.Duration(MAXIMUM_OFFLINE_TIME)

        self.is_subscriber = False


    # def _updateTimer(self, event):
    #     """
    #     Updates the timer to the last changed status. If it
    #     :return:
    #     """
    #     #self._alive_timer = self.get_latest_data("window_stop")["window_stop"]
    #     if (Time.now() - self._alive_timer) > self._offline_time:
    #         print(self.seuid)
    #         print(Time.now() - self._alive_timer)
    #         print(self._offline_time)
    #         self.alive = False
    #         self.set_state("no recent data")
    #     else:
    #         self.alive = True


    def get_type(self):
        """
        Returns the type of the item
        :return: the type
        :rtype: str
        """
        return self._type

    def get_seuid(self):
        """
        Returns the seuid as a string.

        :returns: seuid of the item
        :rtype: str
        """
        return self.seuid

    def add_state(self, state):
        """
        Used to simply add a state to the list of states.
        """
        self.__state.append(state)

    def set_state(self, state):
        if len(self.__state) is not 0:
            self.__state[-1] = state
        else:
            self.__state.append(state)

    def get_state(self):
        """
        Returns the state as a string.

        :returns: state of the item
        :rtype: str
        """
        if self.__state:
            return self.__state[-1]
        return "unknown"

    def _add_data_list(self, name):
        """
        Adds keys to the data_list.

        :param name: the key to be added
        :type name: str
        """
        self._data[name] = []

    def _add_rated_data_list(self, name):
        """
        Adds keys to the rated_data_list.

        :param name: the key to be added
        :type name: str
        """
        self._rated_data[name] = []

    def append_child(self, child):
        """
        Append a child to the list of childs.

        :param child: the child item
        :type child: AbstractItem
        """
        self._child_items.append(child)

    def _update_current_state(self):
        """
        This method updates the current state of the AbstractItem.
        
        :raises TypeError: at the initialization, it's possible that last_states["state"] has no entries and a TypeError occures
        """
        if self.get_state():
            if self.get_state() is not "error":
                last_states = self.get_rated_items_younger_than(Time.now() - (
                    Duration(secs=WARNING_TIMEOUT) if int(Duration(secs=5).to_sec()) <= int(Time.now().to_sec()) else Time(0)),
                                                                "state")
                try:
                    for i in range(0, len(last_states["state"])):
                        if last_states["state"][i] is "error":
                            self.set_state("warning")
                            break
                except TypeError:
                    return

    def append_data(self, message):
        """
        Appends data to the data of the AbstractItem.

        :param message: the message to append
        :type message: one of the different message types TopicStatistics, HostStatistics or NodeStatistics
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        self._data_lock.acquire()
        #self._alive_timer = rospy.Time.now()
        for attribute in self._data:
            try:
                if attribute is "frequency":
                    self._data[attribute].append(message.delivered_msgs / (message.window_stop - message.window_start).to_sec())
                elif attribute is "bandwidth":
                    self._data[attribute].append(message.traffic / (message.window_stop - message.window_start).to_sec())
                else:
                    self._data[attribute].append(getattr(message, attribute))
            except KeyError:
                print("KeyError occurred when trying to access %s", attribute)
                raise

        self._length_of_data += 1
        self._data_lock.release()

    def update_rated_data(self, data):
        """
        Appends data to the rated_data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: RatedStatistics
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        self._rated_data_lock.acquire()

        self._rated_data["window_start"].append(data.window_start)
        self._rated_data["window_stop"].append(data.window_stop)

        last_state = self.get_state()
        new_state = "unknown"

        for element in data.rated_statistics_entity:
            self._rated_data[element.statistic_type + ".actual_value"].append(element.actual_value)
            self._rated_data[element.statistic_type + ".expected_value"].append(element.expected_value)

            for i in range(0, len(element.state)):
                state = topic_statistics_state_to_string(element, element.state[i])
                self._rated_data[element.statistic_type + ".state"].append(state)
                if (state is "low" or state is "high") and state is not "ok" and state is not "unkown":
                    new_state = "error"
                elif state is "ok" and new_state is not "error":
                    new_state = "ok"

        self.add_state(new_state)
        self._update_current_state()
        if new_state is "error" and last_state is not "error":
            self._logger.log("error", Time.now(), self.seuid, self.get_erroneous_entries_for_log())
        self._rated_data_lock.release()

    def child_count(self, parent=None):
        """
        Returns the number of children from the AbstractItem.

        :returns: number of childs
        :rtype: int
        """
        return len(self._child_items)

    def column_count(self):
        """
        Returns the number of columns.

        :returns: the number of columns
        :rtype: int
        """
        return 4

    def get_childs(self, parent=None):
        """
        Returns a list with all children.

        :returns: list of children
        :rtype: list
        """
        return self._child_items

    def get_child(self, row, parent=None):
        """
        Returns the child at the position row.

        :param row: the index of the row
        :type row: int

        :returns: the child at the position row
        :rtype: AbstractItem
        """
        return self._child_items[row]

    def row(self, parent=None):
        """
        Returns the index of the Item.

        :returns: the index of the Item
        :rtype: int
        """
        if self.__parent:
            return self.__parent.get_childs().index(self)
        return 0

    def get_amount_of_entries(self):
        """
        Returns the amount of entries in the data part of the item

        :return: amount of entries
        :rtype: int
        """
        return self._length_of_data

    def get_latest_data(self, *args):
        """
        Returns the latest dict of the data_list or the item of the dict with the given key.

        :param kwargs: the keys to the dict
        :type kwargs: str

        :returns: dict of the item
        :rtype: dict
        :raises KeyError: if an element in args cannot be found in any of the dictionaries (data vs rated data) or attributes (namely name, type, data and state)
        """
        self._data_lock.acquire()
        return_dict = {}

        if args:
            for key in args:
                if key is 'name':
                    return_dict['name'] = self.seuid
                elif key is 'type':
                    return_dict['type'] = self._type
                    # elif key is 'data':
                    # return_dict['data'] = self.get_short_data()
                elif key is 'state':
                    if len(self.__state) is not 0:
                        return_dict['state'] = self.get_state()
                    else:
                        return_dict["state"] = "unknown"
                else:
                    if key in self._data:
                        if self._data[key]:
                            return_dict[key] = self._data[key][-1]
                        else:
                            if key == 'window_stop':
                                return_dict[key] = Time(0)
                            elif key in self.get_list_items():
                                return_dict[key] = [self.tr("Currently no value available")]
                            else:
                                return_dict[key] = self.tr("Currently no value available")
                    elif key in self._rated_data:
                        if self._rated_data[key]:
                            return_dict[key] = self._rated_data[key][-1]
                    else:
                        return_dict[key] = self.tr("Currently no value available")
                        # raise KeyError("item " + key + "was not found")
        else:
            return_dict['name'] = self.seuid
            return_dict['type'] = self._type
            # return_dict['data'] = self.get_short_data()
            for entry in self._data:
                if self._data[entry]:
                    return_dict[entry] = self._data[entry][-1]
                else:
                    if entry == 'window_stop':
                        return_dict[entry] = Time(0)
                    elif entry in self.get_list_items():
                        return_dict[entry] = [self.tr("Currently no value available")]
                    else:
                        return_dict[entry] = self.tr("Currently no value available")
            for entry in self._rated_data:
                if entry == 'window_start' or entry == 'window_stop':
                    continue
                if self._rated_data[entry]:
                    return_dict[entry] = self._rated_data[entry][-1]
                else:
                    return_dict[entry] = self.tr("Currently no value available")
            if len(self.__state) is not 0:
                return_dict['state'] = self.get_state()
            else:
                return_dict['state'] = "unknown"

        self._data_lock.release()
        return return_dict

    def parent(self):
        """
        Returns the parent of this or None if there is no parent.

        :returns: parent
        :rtype: AbstractItem
        """
        return self.__parent

    def get_items_older_than(self, time):
        """
        Returns all items which are older than time.
        Warning: Method assumes data is sorted by time if this is not true will return too few or too much data.
        WARNING: This method is only thread-safe if used via delete_items_older_than() otherwise the
        method may result in undetermined behaviour.

        :param time: the upper bound in seconds
        :type time: rospy.Time

        :returns: dict of lists with the data
        :rtype: dict
        """
        return_values = {}
        breakpoint = 0
        list_of_time = self._data["window_stop"]
        return_values["window_stop"] = []
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[-1] < time:
                for key in return_values:
                    return_values[key] = self._data[key]
            else:
                i = length - 1
                while i > 0 and list_of_time[i] > time:
                    i -= 1
                breakpoint = i
                for key in self._data:
                    return_values[key] = self._data[key][0:breakpoint]
                # todo: currently this is not right for rated data... FIX!!! --> probably move this to another function!
                return_values["state"] = self.__state[breakpoint:length]
        # self._data_lock.release()
        return return_values

    def delete_items_older_than(self, time):
        """
        Deletes all items which are older than time.

        :param time: the upper bound
        :type time: rospy.Time
        """
        self._data_lock.acquire()
        self._rated_data_lock.acquire()
        list_of_time = self._data["window_stop"]

        if len(list_of_time) is not 0:
            i = 0
            entries_to_delete = self.get_items_older_than(time)

            i += len(entries_to_delete["window_stop"])
            for j in range(0, len(entries_to_delete["window_stop"])):
                for value in self._data.values():
                    del value[0]
            self._length_of_data -= i
        self.delete_rated_items_older_than(time)
        self._rated_data_lock.release()
        self._data_lock.release()

    def get_rated_items_older_than(self, time):
        """
        Returns all items which are older than time.
        Warning: Method assumes data is sorted by time if this is not true will return too few or too much data.
        WARNING: This method is only thread-safe if used via delete_items_older_than() otherwise the
        method may result in undetermined behaviour.

        :param time: the upper bound in seconds
        :type time: rospy.Time

        :returns: dict of lists with the data
        :rtype: dict
        """
        return_values = {}
        breakpoint = 0
        list_of_time = self._rated_data["window_stop"]
        return_values["window_stop"] = []
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[-1] < time:
                for key in return_values:
                    return_values[key] = self._rated_data[key]
            else:
                i = length - 1
                while i > 0 and list_of_time[i] > time:
                    i -= 1
                breakpoint = i
                for key in self._rated_data:
                    return_values[key] = self._rated_data[key][0:breakpoint]
                return_values["state"] = self.__state[breakpoint:length]
        return return_values

    def delete_rated_items_older_than(self, time):
        """
        Deletes all items which are older than time.

        :param time: the upper bound
        :type time: rospy.Time
        
        :raises IndexError: Because in most cases not all values are monitored, it is possible that a reated_data_value is empty
        """
        list_of_time = self._rated_data["window_stop"]

        if len(list_of_time) is not 0:
            i = 0
            entries_to_delete = self.get_rated_items_older_than(time)

            i += len(entries_to_delete["window_stop"])
            for j in range(0, len(entries_to_delete["window_stop"])):
                for value in self._rated_data.values():
                    try:
                        del value[0]
                    except IndexError:
                        j += 1
            self._length_of_rated_data -= i

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
        self._data_lock.acquire()
        return_values = {}

        if args:
            for key in args:
                return_values[key] = None
            if "window_stop" not in args:
                return_values["window_stop"] = None
        else:
            return_values["window_stop"] = None
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

        self._data_lock.release()
        return return_values

    def get_rated_items_younger_than(self, time, *args):
        """
        Returns all entries that are younger than time either in all keys of self._rated_data or if args not empty in
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
            for key in self._rated_data:
                return_values[key] = None

        return_values["state"] = None

        breakpoint = 0
        list_of_time = self._rated_data["window_stop"]
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[0] >= time:
                for key in return_values:
                    if key is 'state':
                        return_values[key] = self.__state
                    else:
                        try:
                            return_values[key] = self._rated_data[key]
                        except KeyError:
                            print("Accessed key was: " + key + ". Available keys are: ")
                            print(self._rated_data)
                            raise
            else:
                for i in range(length - 1, -1, -1):
                    if list_of_time[i] < time:
                        breakpoint = i + 1
                        for key in return_values:
                            if key in self._rated_data:
                                return_values[key] = self._rated_data[key][breakpoint:length]
                            elif key is "state":
                                return_values[key] = self.__state[breakpoint:length]
                            else:
                                raise IndexError("IndexError! length of the list %s, accessed index %s. length of data"
                                                 " at given point %s, key is %s", length, i, len(self._data[key]), key)
                        break

        return return_values

    def execute_action(self, action):
        """
        Executes a action on the current item like stop or restart. Calls to this method should be redirected to the remote host on executed there.

        :param action: the action which should be executed
        :type action: RemoteAction
        """
        pass

    def get_detailed_data(self):
        """
        Returns detailed description of current state as html text. Has to be implemented in subclasses.

        :returns: detailed data
        :return: str
        """
        raise NotImplementedError()

    def get_plotable_items(self):
        """
        Returns the plotable entries in the item. Has to be implemented in subclasses.

        :return: list of the items(str)
        :rtype: list
        """
        raise NotImplementedError()

    def get_erroneous_entries(self):
        """
        Returns the erroneous entries as a html string

        :returns: an html string containing the erroneous entries yet preformatted
        :rtype: str
        """
        self._data_lock.acquire()
        content = "<p class=\"get_erroneous_entries\">"
        return_values = {}

        if self.__state:
            if self.get_state() is not "ok" and self.get_state() is not "unknown":
                if self._rated_data["alive.state"]:
                    if self._rated_data["alive.state"][-1] is "high" or self._rated_data["alive.state"][-1] is "low":
                        content += self.tr("alive actual_value:") + \
                                   " <span class=\"erroneous_entry\">" + prepare_number_for_representation(
                            self._rated_data["alive.actual_value"][-1][0]) + "</span>" + \
                                   "<br>"
                        content += self.tr("alive expected_value:") + \
                                   " <span class=\"erroneous_entry\">" + str(
                            self._rated_data["alive.expected_value"][-1][0]) + "</span>" + \
                                   "<br>"
                        content += self.tr("alive state:") + \
                                   " <span class=\"erroneous_entry\">" + str(
                            self._rated_data["alive.state"][-1]) + "</span>" + "<br>"

                for entry in self._attributes:
                    if self._rated_data[entry + ".state"]:
                        if self._rated_data[entry + ".state"][-1] is "high" or self._rated_data[entry + ".state"][
                                -1] is "low":
                            content += self.tr(entry) + \
                                       self.tr(" actual_value:") + \
                                       " <span class=\"erroneous_entry\">" + prepare_number_for_representation(
                                self._rated_data[entry + ".actual_value"][-1][0]) + "</span> " + \
                                       self.tr(entry + "_unit") + "<br>"
                            content += self.tr(entry) + \
                                       self.tr(" expected_value:") + \
                                       " <span class=\"erroneous_entry\">" + str(
                                self._rated_data[entry + ".expected_value"][-1][0]) + "</span> " + \
                                       self.tr(entry + "_unit") + "<br>"
                            content += self.tr(entry) + \
                                       self.tr(" state:") + \
                                       " <span class=\"erroneous_entry\">" + str(
                                self._rated_data[entry + ".state"][-1]) + "</span>" + "<br>"
                content += "<br>"
        content += "</p>"
        self._data_lock.release()
        return content

    def can_execute_actions(self):
        """
        This item cannot execute actions

        :return: False
        """
        return False

    def get_short_data(self):
        return self.get_erroneous_entries_for_log()

    def get_erroneous_entries_for_log(self):
        """
        Returns the erroneous entries for the log as a string

        :returns: an string containing the erroneous entries yet preformatted
        :rtype: str
        """
        self._data_lock.acquire()
        content = ""
        if len(self._rated_data["window_stop"]) != 0:
            if self.get_state() is not "ok" and self.get_state() is not "unknown":
                if self._rated_data["alive.state"][-1] == "high" or self._rated_data["alive.state"][-1] == "low":
                    content += self.tr("alive") + ": " + str(self._rated_data["alive.actual_value"][-1][-1]) + ", "

                for entry in self._attributes:
                    if self._rated_data[entry + ".state"]:
                        if self._rated_data[entry + ".state"][-1] == "high" or self._rated_data[entry + ".state"][-1] is "low":
                            content += self.tr(entry) + ": " + str(self._rated_data[entry + ".state"][-1]) + ", "
        else:
            return "Not sufficient rated data yet"
        self._data_lock.release()
        return content