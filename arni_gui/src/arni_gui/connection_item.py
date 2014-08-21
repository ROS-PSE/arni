from abstract_item import AbstractItem

class ConnectionItem(AbstractItem):


    """A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on"""


def __init__(self, seuid, type, can_execute_actions, parent=None):
    """Initializes the ConnectionItem

    :param list: connection list
    :type list: list
    :param parent: the parent-object
    :type parent: object
    """
    if type is not "connection":
        raise UserWarning("New connection item got a wrong type assigned")

    AbstractItem.__init__(self, seuid, type, can_execute_actions, parent)
    #add the content
    attributes = ["topic", "node_pub",
                            "node_sub", "window_start", "window_stop", "dropped_msgs", "traffic", "period_mean",
                            "period_stddev", "period_max", "stamp_age_mean", "stamp_age_stddev", "stamp_age_max"]
    for item in attributes:
        self.__add_data_list(item)

def execute_action(self, action):
    """Not senseful, throws an exception

    :param action: action to be executed
    :type action: RemoteAction
    """
    pass

