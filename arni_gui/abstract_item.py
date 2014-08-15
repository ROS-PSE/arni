class AbstractItem:


    """ Provides a unified interface to access the items of the model"""


def __init__(self, list, parent=None):
    """Initializes theAbstractItem

    :param list: item list
    :type list: list
    :param parent: the parent-object
    :type parent: object
    """
    pass


def append_child(self, child):
    """Append a child to the list of childs

    :param child: the child item
    :type child: AbstractItem
    """
    pass


def append_data(self, data):
    """Append data to the data_list of the AbstractItem

    :param data: the data to appen
    :type data: object
    """
    pass


def get_child(self, row):
    """Returns the child at the position row

    :param row: the index of the row
    :type row: int

    :returns: AbstractItem
    """
    pass


def get_latest_data(self):
    """Returns the latest dict of the data_list

    :returns: dict
    """
    pass


def parent(self):
    """Returns the parent of this or None if there is no parent

    :returns: AbstractItem
    """
    pass


def get_items_older_than(self, time):
    """Returns all items which are older than time

    :param time: the upper bound
    :type time: rospy.Time

    :returns: AbstractItem
    """
    pass


def delete_items_older_than(self, time):
    """Deletes all items which are older than time

    :param time: the upper bound
    :type time: rospy.Time
    """
    pass


def get_items_younger_than(self, time):
    """Returns all items which are younger than time

    :param time: the lower bound
    :type time: rospy.Time

    :returns: AbstractItem
    """
    pass


@abstractmethod
def execute_action(self, action):
    """Executes a action on the current item like stop or restart. Calls to this method should be redirected to the remote host on executed there."""
