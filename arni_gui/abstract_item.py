from rospy.rostime import Duration, Time


class AbstractItem:
    """ Provides a unified interface to access the items of the model"""


def __init__(self, identifier, type, can_execute_actions, parent=None, ):
    #todo:doku is missing here
    """Initializes theAbstractItem

    :param parent: the parent-object
    :type parent: object
    """
    self.__data = []
    self.__child_items = []
    self.__parent = parent
    self.__identifier = identifier
    self.__type = type
    self.__can_execute_actions = can_execute_actions


def append_child(self, child):
    """Append a child to the list of childs

    :param child: the child item
    :type child: AbstractItem
    """
    self.__child_items.append(child)


def append_data(self, data):
    """Append data to the data_list of the AbstractItem

    :param data: the data to appen
    :type data: object
    """
    self.__data.append(data)


def child_count(self):
    # todo: should this be "intelligent"?
    sum = 0
    for item in self.__child_items:
        sum += 1
        sum += item.child_count()

    return sum
    #return len(self.child_items)


def column_count(self):
    # todo: return !not! a concrete number here ?!?!
    return 4


def get_child(self, row):
    """Returns the child at the position row

    :param row: the index of the row
    :type row: int

    :returns: AbstractItem
    """
    return self.__child_itemsitems[row]


def row(self):
    """
    todo: document!
    """
    if self.__parent:
        return self.__parent.childItems.index(self)

    return 0


def get_latest_data(self, key=None):
    """Returns the latest dict of the data_list or the item of the dict with the given key

    :param key: the key for the dict
    :type key: str
    :returns: dict or the item
    """
    if key is not None:
        if key is 'name':
            return self.__identifier
        elif key is 'type':
            return self.__type
        else:
            try:
                return self.__data[key]
            except KeyError:
                print("KeyError catched when accesing element %s.", key)
                raise
    return self.__data[-1]


def parent(self):
    """Returns the parent of this or None if there is no parent

    :returns: AbstractItem
    """
    return self.__parent


def get_items_older_than(self, time):
    """Returns all items which are older than time

    :param time: the upper bound in seconds
    :type time: Time

    :returns: AbstractItem
    """
    return_values = []
    for item in self.__data:
        # check timestamp
        end_time = Time.now() - Duration(secs=time)
        if item.timestamp < end_time:
            return_values.append(item)
    return return_values


def delete_items_older_than(self, time):
    """Deletes all items which are older than time

    :param time: the upper bound
    :type time: rospy.Time
    """
    for item in self.get_items_older_than(self, time):
        del item


def get_items_younger_than(self, time):
    """Returns all items which are younger than time

    :param time: the lower bound
    :type time: rospy.Time

    :returns: AbstractItem
    """
    return_values = []
    for item in self.__data:
        # check timestamp
        start_time = Time.now() - Duration(secs=time)
        if item.timestamp > start_time:
            return_values.append(item)
    return return_values


# todo: no longer abstract method
def execute_action(self, action):
    """Executes a action on the current item like stop or restart. Calls to this method should be
    redirected to the remote host on executed there."""
    raise NotImplementedError
    if self.__can_execute_action:
        #todo: open service and send the message
        #todo: find out how the service wants the parameters
        pass