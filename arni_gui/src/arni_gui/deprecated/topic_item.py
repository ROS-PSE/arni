import AbstractItem

class TopicItem(AbstractItem):


    """A TopicItem represents a specific topic which contains many connections and has attributes like the number of sent messages"""


def __init__(self, list, parent=None):
    """Initializes the TopicItem

    :param list: topic list
    :type list: list
    :param parent: the parent-object
    :type parent: object
    """
    pass


def execute_action(self, action):
    """Not senseful, throws an exception

    :param action: action to be executed
    :type action: RemoteAction
    """
    pass

