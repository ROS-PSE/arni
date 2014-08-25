from rospy.rostime import Duration, Time
from python_qt_binding.QtCore import QObject

class AbstractItem(QObject):
    """ Provides a unified interface to access the items of the model
        INTERNAL: WARNING! Whenever the key-values at the beginning are not set right, the oddest things may occur!
    """

    def __init__(self, seuid, parent=None, *args):
            #todo:doku is missing here
            """Initializes theAbstractItem

            :param parent: the parent-object
            :type parent: object
            """
            super(AbstractItem, self).__init__(parent)

            self.__data = {}
            self.__child_items = []
            self.__parent = parent
            self.seuid = seuid
            self.__type = "type"
            self.__data_attribute = "data"
            self._attributes = ['state', 'window_stop', 'window_start']
            self._attributes.extend(args)


            for item in self._attributes:
                self._add_data_list(item)
            
           # for item in args:
            #    self.__add_data_list(item)


    def get_seuid(self):
        return self.seuid

    def get_state(self):
        return self.__data['state'][-1]

    def _add_data_list(self, name):
        self.__data[name] = []

    def append_child(self, child):
        """Append a child to the list of childs

        :param child: the child item
        :type child: AbstractItem
        """
        self.__child_items.append(child)

    def append_data_dict(self, data):
        """Append data to the data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: dict
        """
        if "window_stop" not in data:
            data["window_stop"] = Time.now()
        for attribute in self._attributes:
            if attribute in data:
                self.__data[attribute].append(data[attribute])
            else:
                #todo: is there something better than None in this case? like "" ?
                self.__data[attribute].append(None)

        self.__update_current_state()

    def __update_current_state(self):
        length = len(self.__data["state"])
        for i in range(length - len((self.get_items_younger_than(Time.now() - Duration(secs=5)))["window_stop"]), length):
            if self.__data["state"][i] == "error":
                self.__data["state"][-1] = "warning"
                break

    def append_data(self, data):
        """Append data to the data of the AbstractItem.

        :param data: the data to append in key value form
        :type data:
        """
        for attribute in self._attributes:
            #todo: correct?
            if attribute is 'state':
	        self.__data[attribute].append("")
	        continue
            try:
                self.__data[attribute].append(getattr(data, attribute))
            except KeyError:
                print("KeyError occurred when trying to access %s", attribute)
                raise
        self.__update_current_state()

    def update_data(self, data, window_start, window_stop):
        """

        :param data:
        :type data:
        :param time:
        :type time: rostime?
        :return:
        """
        found = False
        #todo: are these all bad cases?
        for current in range(0, len(self.__data["window_start"])):
            if window_stop < self.__data[window_start][current]:
                continue
            if window_start > self.__data[window_stop][current]:
                continue
            found = True
            for attribute in self._attributes:
                self.__data[attribute][current] = data.getattr(attribute, None)

            # for key in data:
            #     self.__data[key][current] = data[key]

        if found is not True:
            raise UserWarning("No matching time window was found. Could not update the AbstractItem")


    def child_count(self):
        sum = 0
        for item in self.__child_items:
            sum += 1
            sum += item.child_count()

        return sum
        #return len(self.child_items)


    def column_count(self):
        # todo: return !not! a concrete number here ?!?!
        return 4

    def get_childs(self):
        """

        :return:
        """
        return self.__child_items


    def get_child(self, row):
        """Returns the child at the position row

        :param row: the index of the row
        :type row: int

        :returns: AbstractItem
        """
        return self.__child_items[row]


    def row(self):
        """
        todo: document!
        """
        if self.__parent:
            return self.__parent.childItems.index(self)

        return 0


    def get_latest_data(self, *kwargs):
        #todo:update docu, now more mighty
        """Returns the latest dict of the data_list or the item of the dict with the given key

        :param kwargs: the keys to the dict
        :type kwargs: str
        :returns: dict or the item
        """
        if kwargs is not None:
            for key in kwargs:
                if key is 'name':
                    return self.seuid
                elif key is 'type':
                    return self.__type
                else:
                    try:
                        return self.__data[key][-1]
                    except KeyError:
                        print("KeyError caught when accessing element %s.", key)
                        raise

        return_dict = {}
        # return dict of latest item
        for entry in self.__data:
            return_dict[entry] = self.__data[entry][-1]
        return return_dict


    def parent(self):
        """Returns the parent of this or None if there is no parent

        :returns: AbstractItem
        """
        return self.__parent

#todo: what are the following 3 methods for and how can they be done better?
    def get_items_older_than(self, time):
        """Returns all items which are older than time

        :param time: the upper bound in seconds
        :type time: rospy.Time

        :returns: dict of lists
        """
        return_values = []
        for key in self.__data:
            return_values[key] = []

        list_of_time = self.__data["window_stop"]
        for i in range(0, len(list_of_time)):
            # check timestamp
            #end_time = Time.now() - Duration(nsecs=time)
            if list_of_time[i] < time:
                #return_values.append()
                for key in self.__data:
                    return_values[key].append(self.__data[key][i])
        return return_values


    def delete_items_older_than(self, time):
        """Deletes all items which are older than time

        :param time: the upper bound
        :type time: rospy.Time
        """
        list_of_time = self.__data["window_stop"]
        for i in range(0, len(list_of_time)):
            # check timestamp
            #end_time = Time.now() - Duration(nsecs=time)
            if list_of_time[i] < time:
                #return_values.append()
                for key in self.__data:
                    del self.__data[key][i]


    def get_items_younger_than(self, time):
        """Returns all items which are younger than time

        :param time: the lower bound in seconds
        :type time: rospy.Time
 
        :returns: dict of lists
        """
        return_values = {}
        for key in self.__data:
            return_values[key] = []

        list_of_time = self.__data["window_stop"]
        for i in range(0, len(list_of_time) - 1):
            # check timestamp
            #start_time = Time.now() - Duration(nsecs=time)
            if list_of_time[i] > time:
                for key in self.__data:
                    try:
                        return_values[key].append(self.__data[key][i])
                    except IndexError:
                        print("IndexError! length of the list %s, accessed index %s. length of data at given point %s, key is %s",
                              len(list_of_time), i, len(self.__data[key]), key)
                        raise
        return return_values

    def execute_action(self, action):
        """Executes a action on the current item like stop or restart. Calls to this method should be
        redirected to the remote host on executed there."""
        pass

    def get_detailed_data(self):
        """
        Returns detailed description of current state as html text.

        :return:
        """
        raise NotImplemented()