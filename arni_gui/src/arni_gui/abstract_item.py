from threading import Lock
import time as tm

from rospy.rostime import Duration, Time

from python_qt_binding.QtCore import QTranslator, QObject

from helper_functions import prepare_number_for_representation

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
        """
        __rated_data is dict containing the rated data. state, window_start and window_end are simply lists
         with the corresponding entries. Any other values typically is a list containing lists which however contain the
         values. This is equivalent to the representation in the RatedStatistics/Entity.
        """
        self.__rated_data = {}
        self.__child_items = []
        self.__parent = parent
        self.seuid = seuid
        self._type = "type"
        self.__data_attribute = "data"
        self.__state = []
        self.__last_update = Time.now()
        self.__creation_time = Time.now()

        self._add_data_list("window_start")
        self._add_data_list("window_stop")
        self._add_rated_data_list("window_start")
        self._add_rated_data_list("window_stop")

        self.__length_of_data = 0
        self.__length_of_rated_data = 0


    def get_seuid(self):
        """
        Returns the seuid of the Abstractitem.

        :returns: str
        """
        return self.seuid


    def get_state(self):
        """
        Returns the state as a string.

        :returns: state of the item
        :rtype: str
        """
        if self.__state:
            return self.__state[-1]


    def _add_data_list(self, name):
        """
        Adds keys to the data_list.

        :param name: the key to be added
        :type name: str
        """
        # print("added " + name)
        self._data[name] = []


    def _add_rated_data_list(self, name):
        """
        Adds keys to the rated_data_list.

        :param name: the key to be added
        :type name: str
        """
        self.__rated_data[name] = []


    def append_child(self, child):
        """
        Append a child to the list of childs.

        :param child: the child item
        :type child: AbstractItem
        """
        self.__child_items.append(child)

#todo: not needed and wrong!
    # def append_rated_data_dict(self, data):
    #     """
    #     Appends data to the rate_data of the AbstractItem.
    #
    #     :param data: the data to append in key value from
    #     :type data: dict
    #     :raises KeyError: if an entry is in the global rated data dictionary but not found in the given dictionary
    #     """
    #     if "window_stop" not in data:
    #         data["window_stop"] = Time.now()
    #     for attribute in self.__rated_data:
    #         if attribute in data:
    #             self.__rated_data[attribute].append(data[attribute])
    #         else:
    #             # todo: is there something better than None in this case? like "" ?
    #             self.__rated_data[attribute].append(None)
    #
    #     if "state" in data:
    #         self.__state.append(data["state"])
    #     else:
    #         self.__state.append(data["ok"])
    #     self.__length_of_rated_data += 1
    #     self._update_current_state()


    def append_data_dict(self, data):
        """
        Appends data to the data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: dict
        :raises KeyError: if an entry is in the global data dictionary but not found in the given dictionary
        """
        if "window_stop" not in data:
            data["window_stop"] = Time.now()
        for attribute in self._data:
            if attribute in data:
                self._data[attribute].append(data[attribute])
            else:
                self._data[attribute].append(None)
        if "state" in data:
            self.__state.append(data["state"])
        else:
            self.__state.append("unknown")
        self.__length_of_data += 1
        self._update_current_state()


    def _update_current_state(self):
        """
        This method updates the current state of the AbstractItem.
        """
        length = len(self.__state)
        if self.__state:
            if self.__state[-1] is not "error" and self.__state[-1] is not "warning" \
                    and self.__state[-1] is not "unknown":
                for i in range(length - len((self.get_items_younger_than(Time.now() - Duration(secs=5), "window_stop"))["window_stop"]), length):
                    if self.__state[i] == "error":
                        self.__state[-1] = "warning"
                        break
        self.__last_update = Time.now()


    # TODO append_data or append_data_dict is correct
    def append_data(self, data):
        """
        Appends data to the data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: one of the different message types names TopicStatistics, HostStatistics or NodeStatistics
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        for attribute in self._data:
            try:
                self._data[attribute].append(getattr(data, attribute))
            except KeyError:
                print("KeyError occurred when trying to access %s", attribute)
                raise

        #todo: is the state sensefull here? I THINK NOT!!!
        self.__state.append("unknown")
        self.__length_of_data += 1
        self._update_current_state()


    def update_rated_data(self, data):
        """
        Appends data to the rated_data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: dict
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        for entry in self.__rated_data:
            try:
                self.__rated_data[entry].append(data[entry])
            except KeyError:
                print("An entry found in the object dictionary of the rated data was not found in the given rated data")
                raise
        if self.__state:
            if "state" in data:
                self.__state[-1] = data["state"]
        else:
            #todo: now there is one entry too much in self.__state... is that a problem?
            self.__state.append("unknown")
            if "state" in data:
                self.__state[-1] = data["state"]
        self.__length_of_rated_data += 1
        self._update_current_state()


    def child_count(self):
        """
        Returns the number of children from the AbstractItem.

        :returns: int
        """
        return len(self.__child_items)


    def column_count(self):
        """
        Returns the number of columns.

        :returns: the number of columns
        :rtype: int
        """
        return 4

    def get_childs(self):
        """
        Returns a list with all children.

        :returns: list of children
        :rtype: list
        """
        return self.__child_items


    def get_child(self, row):
        """
        Returns the child at the position row.

        :param row: the index of the row
        :type row: int

        :returns: the child at the position row
        :rtype: AbstractItem
        """
        return self.__child_items[row]


    def row(self):
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
        return self.__length_of_data


    def get_latest_data(self, *args):
        """
        Returns the latest dict of the data_list or the item of the dict with the given key.

        :param kwargs: the keys to the dict
        :type kwargs: str

        :returns: dict of the item
        :rtype: dict
        :raises KeyError: if an element in args cannot be found in any of the dictionaries (data vs rated data) or
        attributes (namely name, type, data and state)
        """
        return_dict = {}
        if args:
            for key in args:
                if key is 'name':
                    return_dict['name'] = self.seuid
                elif key is 'type':
                    return_dict['type'] = self._type
                #elif key is 'data':
                    #return_dict['data'] = self.get_short_data()
                elif key is 'state':
                    if self.__state:
                        return_dict['state'] = self.__state[-1]
                    else:
                        return_dict["state"] = "state unknown"
                else:
                    if key in self._data:
                        if self._data[key]:
                            return_dict[key] = self._data[key][-1]
                    elif key in self.__rated_data:
                        if self.__rated_data[key]:
                            return_dict[key] = self.__rated_data[key][-1]
                    else:
                        raise KeyError("item " + key + "was not found")
        else:
            return_dict['name'] = self.seuid
            return_dict['type'] = self._type
            #return_dict['data'] = self.get_short_data()
            for entry in self._data:
                if self._data[entry]:
                    return_dict[entry] = self._data[entry][-1]
                else:
                    return_dict[entry] = "Currently no value available"
            for entry in self.__rated_data:
                if self.__rated_data[entry]:
                    return_dict[entry] = self.__rated_data[entry][-1]
                else:
                    return_dict[entry] = "Currently no value available"
            if self.__state:
                return_dict['state'] = self.__state[-1]
            else:
                return_dict['state'] = "unknown"
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

        :param time: the upper bound in seconds
        :type time: rospy.Time

        :returns: dict of lists with the data
        :rtype: dict
        """
        return_values = {}
        #for key in self._data:
        #    return_values[key] = []

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
                #todo: currently this is not right for rated data... FIX!!! --> probably move this to another function!
                #for key in self.__rated_data:
                #    return_values[key] = self.__rated_data[key][0:breakpoint]
                return_values["state"] = self.__state[breakpoint:length]

        return return_values


    def delete_items_older_than(self, time):
        """
        Deletes all items which are older than time.

        :param time: the upper bound
        :type time: rospy.Time
        """
        list_of_time = self._data["window_stop"]
        if len(list_of_time) is not 0:
            #length = len(list_of_time)
            i = 0
            entries_to_delete = self.get_items_older_than(time)

            i += len(entries_to_delete["window_stop"])
            for j in range(0, len(entries_to_delete["window_stop"])):
                for value in self._data.values():
                    del value[0]
                del self.__state[0]

            self.__length_of_data -= i



    def get_items_younger_than(self, time, *args):
        """
        Returns all entries that are younger than time either in all keys of self._data or if args not empty in
        all key corresponding to args.
        Warning: Method assumes data is sorted by time if this is not true will return too few or too much data.

        :param time: the lower bound in seconds
        :type time: rospy.Time
        :param kwargs: the keys to the dict
        :type kwargs: str
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
            for key in self.__rated_data:
                return_values[key] = None

        breakpoint = 0
        list_of_time = self._data["window_stop"]
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[0] >= time:
                for key in return_values:
                    return_values[key] = self._data[key]
            else:
                for i in range(length - 1, -1, -1):
                    if list_of_time[i] < time:
                        breakpoint = i + 1
                        for key in return_values:
                            if key in self._data:
                                return_values[key] = self._data[key][breakpoint:length]
                            elif key in self.__rated_data:
                                return_values[key] = self.__rated_data[key][breakpoint:length]
                            elif key is "state":
                                return_values[key] = self.__state[breakpoint:length]
                            else:
                                raise IndexError("IndexError! length of the list %s, accessed index %s. length of data"
                                                 " at given point %s, key is %s", length, i, len(self._data[key]), key)
                        break

        return return_values


    def execute_action(self, action):
        """
        Executes a action on the current item like stop or restart. Calls to this method should be
        redirected to the remote host on executed there.

        :param acion: the action which should be executed
        :type action: RemoteAction
        """
        pass


    def get_detailed_data(self):
        """
        Returns detailed description of current state as html text. Has to be implemented in subclasses.

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

#todo: has to be updated, the rated_values can also be lists!!!
    def get_erroneous_entries(self):
        """
        Returns the erroneous entries as a html string

        :return: an html string containing the erroneous entries yet preformatted
        :rtype: str
        """
        content = "<p class=\"get_erroneous_entries\">"
        return_values = {}
        if self.__state is not "ok":
            for entry in self._attributes:
                if self.__rated_data[entry + ".state"]:
                    #todo: should probably only be error, countercheck this please :)
                    for i in range(0, len(self.__rated_data[entry + ".state"][-1])):
                        if self.__rated_data[entry + ".state"][-1][i] is "error":
                            content += QTranslator.translate("AbstractItem", entry) +\
                                       QTranslator.translate("AbstractItem", "actual_value") +\
                                       " <strong class=\"erroroneous_entry\">" + prepare_number_for_representation(
                                       self.__rated_data[entry + ".actual_value"][i]) + "</strong>" + \
                                       QTranslator.translate("AbstractItem", entry + "_unit")
                            content += QTranslator.translate("AbstractItem", entry) +\
                                       QTranslator.translate("AbstractItem", "expected_value") +\
                                       " <strong class=\"erroroneous_entry\">" + prepare_number_for_representation(
                                       self.__rated_data[entry + ".expected_value"][i]) + "</strong>" + \
                                       QTranslator.translate("AbstractItem", entry + "_unit")
                            content += QTranslator.translate("AbstractItem", entry) +\
                                       QTranslator.translate("AbstractItem", "state") +\
                                       " <strong class=\"erroroneous_entry\">" + prepare_number_for_representation(
                                       self.__rated_data[entry + ".state"][i]) + "</strong>"

        content += "</p>"
        return content

    def can_execute_actions(self):
        """
        This item cannot execute actions

        :return: False
        """
        return False

    def get_short_data(self):
        raise NotImplementedError()