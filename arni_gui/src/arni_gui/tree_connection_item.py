from connection_item import ConnectionItem

from python_qt_binding.QtCore import QObject

class TreeConnectionItem(QObject):
    def __init__(self, parent, connection_item, is_subscriber):
        """
        has to match the constructor of abstract_item for simplicity

        :param logger:
        :param seuid:
        :param first_message:
        :param parent:
        :return:
        """
        super(TreeConnectionItem, self).__init__(None)
        self.connection_item = connection_item
        self.is_subscriber = is_subscriber
        self.__parent = parent
        
        self.marked = False
        self.seuid = self.connection_item.seuid
        #self.alive = True

    def append_data(self, message):
        """
        Appends data to the data of the AbstractItem.

        :param message: the message to append
        :type message: one of the different message types TopicStatistics, HostStatistics or NodeStatistics
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        """
        IMPORTANT: If connection item is subscriber the data will not be actualized since it is assumed that there
        is also an publisher. When the publisher receives data it will be pushed into the connection_item.
        """
        if not self.is_subscriber:
            self.connection_item.append_data(message)

    def aggregate_data(self, period):
        """
        :param period: The amount in seconds over which the data should be aggregated.
        :return:
        """
        return self.connection_item.aggregate_data(period)


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
        return self.connection_item.get_detailed_data()

    def get_plotable_items(self):
        """
        Returns items for the plot.

        :returns: str[]
        """
        return self.connection_item.get_plotable_items()


    def get_short_data(self):
        """
        Returns a shortend version of the item data.

        :returns: data of the item
        :rtype: str
        """
        return self.connection_item.get_short_data()

    def get_list_items(self):
        return self.connection_item.get_list_items()


    def get_time_items(self):
        return self.connection_item.get_time_items()


    def get_type(self):
        """
        Returns the type of the item
        :return: the type
        :rtype: str
        """
        return self.connection_item.get_type()

    def get_seuid(self):
        """
        Returns the seuid as a string.

        :returns: seuid of the item
        :rtype: str
        """
        return self.connection_item.seuid

    def add_state(self, state):
        """
        Used to simply add a state to the list of states.
        """
        if not self.is_subscriber:
            self.connection_item.add_state(state)

    def set_state(self, state):
        self.connection_item.set_state(state)

    def get_state(self):
        """
        Returns the state as a string.

        :returns: state of the item
        :rtype: str
        """
        return self.connection_item.get_state()

    def append_child(self, child):
        """
        Append a child to the list of childs.

        :param child: the child item
        :type child: AbstractItem
        """
        self.connection_item.append_child(child)

    def update_rated_data(self, data):
        """
        Appends data to the rated_data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: RatedStatistics
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        self.connection_item.update_rated_data(data)

    def child_count(self, parent=None):
        """
        Returns the number of children from the AbstractItem.

        :returns: number of childs
        :rtype: int
        """
        return self.connection_item.child_count(parent)

    def can_execute_actions(self):
        return False

    def column_count(self):
        """
        Returns the number of columns.

        :returns: the number of columns
        :rtype: int
        """
        return self.connection_item.column_count()

    def get_childs(self, parent=None):
        """
        Returns a list with all children.

        :returns: list of children
        :rtype: list
        """
        return self.connection_item.get_childs(parent)

    def get_child(self, row, parent=None):
        """
        Returns the child at the position row.

        :param row: the index of the row
        :type row: int

        :returns: the child at the position row
        :rtype: AbstractItem
        """
        return self.connection_item.get_child(row, parent)

    def row(self, parent=None):
        """
        Returns the index of the Item.

        :returns: the index of the Item
        :rtype: int
        """
        return self.connection_item.row(parent)

    def get_amount_of_entries(self):
        """
        Returns the amount of entries in the data part of the item

        :return: amount of entries
        :rtype: int
        """
        return self.connection_item.get_amount_of_entries()

    def get_latest_data(self, *args):
        """
        Returns the latest dict of the data_list or the item of the dict with the given key.

        :param kwargs: the keys to the dict
        :type kwargs: str

        :returns: dict of the item
        :rtype: dict
        :raises KeyError: if an element in args cannot be found in any of the dictionaries (data vs rated data) or attributes (namely name, type, data and state)
        """
        return_dict = self.connection_item.get_latest_data(*args)
        if self.is_subscriber and 'name' in args:
            return_dict['name'] += "--sub"
        if self.is_subscriber and 'type' in args:
            return_dict['type'] += "-sub"
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
        return self.connection_item.get_items_older_than(time)

    def delete_items_older_than(self, time):
        """
        Deletes all items which are older than time.

        :param time: the upper bound
        :type time: rospy.Time
        """
        return self.connection_item.delete_items_older_than(time)

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
        return self.connection_item.get_rated_items_older_than(time)

    def delete_rated_items_older_than(self, time):
        """
        Deletes all items which are older than time.

        :param time: the upper bound
        :type time: rospy.Time

        :raises IndexError: Because in most cases not all values are monitored, it is possible that a reated_data_value is empty
        """
        return self.connection_item.delete_rated_items_older_than(time)

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
        return self.connection_item.get_items_younger_than(time, *args)

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
        return self.connection_item.get_rated_items_younger_than(time, *args)


    def get_erroneous_entries(self):
        """
        Returns the erroneous entries as a html string

        :returns: an html string containing the erroneous entries yet preformatted
        :rtype: str
        """
        return self.connection_item.get_erroneous_entries()

    def get_erroneous_entries_for_log(self):
        """
        Returns the erroneous entries for the log as a string

        :returns: an string containing the erroneous entries yet preformatted
        :rtype: str
        """
        return self.connection_item.get_erroneous_entries_for_log()
